"""Sim-to-real gap study of the F3 payload on the ASMR robot: which differences between the simulation and the real robot
most likely stop the real robot from walking, and how much the payload matters.

    uv run python sim/sim2sim/gap_study.py                 all cases, policy baseline -> results/gap_study.*
    uv run python sim/sim2sim/gap_study.py --policy NAME   all cases for another policy of policies/, added to results/gap_study.*
    uv run python sim/sim2sim/gap_study.py --cases ID,ID   only these cases (for --policy NAME if given), added
    uv run python sim/sim2sim/gap_study.py --report        tables and figure again from results/gap_study.json, 1 s
    uv run python sim/sim2sim/gap_study.py --quick         self-check, about 1 min

Each case runs hw_risks.simulate() (the run.py policy loop with the drive model of hw_risks.py) on the ASMR robot with a
payload, with model changes and drive settings: 4 commands x 10 seeds of 20 s, and a standing push test.
All cases use the same seeds, so each run of a case pairs with one run of the F3 nominal case.
The ASMR model already has measured leg masses and fitted armature and joint friction (its MJCF), so these are the
nominal values here, and the cases move them up and down.
"""
import json
import math
import sys
import tempfile
import time
from functools import lru_cache
from multiprocessing import Pool
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path[:0] = [str(HERE), str(HERE.parent / "payload")]
sys.path.insert(0, str(ROOT))
from policies import policy_select  # noqa: E402
ASKED_POLICY = None  # --policy NAME adds a policy; hw_risks.py would read it as its own policy, so take it out first
if "--policy" in sys.argv:
    ASKED_POLICY = sys.argv[sys.argv.index("--policy") + 1]
    del sys.argv[sys.argv.index("--policy"):sys.argv.index("--policy") + 2]
import run as s2s  # noqa: E402  the ASMR runner: policy, contract, model, observation
import hw_risks as hw  # noqa: E402  sim/sim2sim/hw_risks.py: drive model
import report  # noqa: E402  sim/payload/report.py: palette and plot style

plt = report.plt
JSON = s2s.OUT / "gap_study.json"
POLICIES = {policy_select.DEFAULT: policy_select.onnx(policy_select.DEFAULT, robot="asmr2026")}
WORKERS = 6  # shared machine: a GPU training job and a render run at the same time
SEEDS = hw.SEEDS  # 10 seeds x 4 commands = 40 runs per case and policy, as hw_risks.py
PUSH_DIRS = {"forward": 0.0, "back": math.pi, "left": math.pi / 2, "right": -math.pi / 2}
PUSH_T = (3.0, 3.3)  # s, standing push times, one per push seed
PUSH_HI, PUSH_ITERS = 12.0, 5  # N s, bisection resolution 0.375 N s
JOINT_TYPES = ["hip roll", "hip yaw", "hip pitch", "knee", "ankle pitch", "ankle roll"]
F3 = "f3"  # payload option of source/robot_models/robots/asmr2026_payload.py


def torque(s):
    """Torque per amp x s: the drive works in amps, so K, D and the torque limit all scale with s."""
    return {"K": s2s.KP * s, "D": s2s.KD * s, "lim": s2s.EFFORT_LIMITS * s}


# Nominal (MJCF of the ASMR robot): armature 0.0228 hip and knee / 0.0056 ankle kg m², joint friction 0.38 / 0.32 Nm, measured leg
# masses, foot friction 1.0 (floor). R: the realistic robot, a guess. P: every value at its pessimistic value.
REAL = ({"friction": 0.6},
        {"obs_ms": 10, "cmd_ms": 20, "imu_deg": 2.0, "backlash_deg": 0.5, "hose_n": 1.0, "hose_var": True, "zero_deg": 1.0})
PESS = ({"armature": (0.034, 0.0085), "frictionloss": 0.5, "leg_mass": 1.1, "com_dx": -0.02, "friction": 0.4},
        {**torque(0.85), "obs_ms": 20, "cmd_ms": 40, "imu_deg": 4.0, "backlash_deg": 1.0, "hose_n": 3.0, "zero_deg": 2.0})
# R with one value moved to its P value: (label, model change keys, drive change keys)
R_PLUS = {"armature": ("armature 0.034 / 0.0085", ["armature"], []), "friction": ("joint friction 0.5 Nm", ["frictionloss"], []),
          "legs": ("legs +10 %", ["leg_mass"], []), "com": ("torso CoM 2 cm back", ["com_dx"], []),
          "foot": ("foot friction 0.4", ["friction"], []), "torque": ("torque per amp × 0.85", [], ["K", "D", "lim"]),
          "obs": ("observation latency 20 ms", [], ["obs_ms"]), "cmd": ("command latency 40 ms", [], ["cmd_ms"]),
          "imu": ("IMU tilt 4°", [], ["imu_deg"]), "play": ("gear play 1°", [], ["backlash_deg"]),
          "hose": ("hose pull 3 N, constant", [], ["hose_n", "hose_var"]), "zero": ("zero error ±2°", [], ["zero_deg"])}
WALK_CMDS = ("vx 0.3", "vx 0.2 + yaw 0.5")  # the R+ cases run only these, without the push test (compute time)


def r_plus(e_keys, d_keys):
    e, d = dict(REAL[0]), {k: v for k, v in REAL[1].items() if k not in d_keys}
    return {**e, **{k: PESS[0][k] for k in e_keys}}, {**d, **{k: PESS[1][k] for k in d_keys if k in PESS[1]}}


# id: (label, factor, payload option, model changes, drive settings on top of hw_risks case 1)
CASES = {
    "bare": ("bare robot, no payload", "model", "none", {}, {}),
    "box_1kg": ("1 kg box (what the baseline trained with)", "model", "box_1kg", {}, {}),
    "F3": ("F3 parts, 4.14 kg (nominal)", "model", F3, {}, {}),
    "arm_low": ("armature 0.007 / 0.002 kg m² (BHL training value)", "armature", F3, {"armature": (0.007, 0.002)}, {}),
    "arm_mid": ("armature 0.014 / 0.0047 kg m²", "armature", F3, {"armature": (0.014, 0.0047)}, {}),
    "arm_high": ("armature 0.034 / 0.0085 kg m² (1.5 x nominal)", "armature", F3, {"armature": (0.034, 0.0085)}, {}),
    "fric_0.3": ("joint friction 0.3 Nm (nominal 0.38 / 0.32)", "joint friction", F3, {"frictionloss": 0.3}, {}),
    "fric_0.5": ("joint friction 0.5 Nm", "joint friction", F3, {"frictionloss": 0.5}, {}),
    "torque_0.85": ("torque per amp × 0.85", "torque per amp", F3, {}, torque(0.85)),
    "torque_1.15": ("torque per amp × 1.15", "torque per amp", F3, {}, torque(1.15)),
    "leg_mass": ("leg links +10 % mass", "leg mass", F3, {"leg_mass": 1.1}, {}),
    "com_fwd": ("torso CoM 2 cm forward", "torso CoM", F3, {"com_dx": 0.02}, {}),
    "com_back": ("torso CoM 2 cm back", "torso CoM", F3, {"com_dx": -0.02}, {}),
    "foot_0.6": ("foot friction 0.6", "foot friction", F3, {"friction": 0.6}, {}),
    "foot_0.4": ("foot friction 0.4", "foot friction", F3, {"friction": 0.4}, {}),
    "obs_10": ("observation latency 10 ms", "observation latency", F3, {}, {"obs_ms": 10}),
    "obs_20": ("observation latency 20 ms", "observation latency", F3, {}, {"obs_ms": 20}),
    "cmd_20": ("command latency 20 ms", "command latency", F3, {}, {"cmd_ms": 20}),
    "cmd_40": ("command latency 40 ms", "command latency", F3, {}, {"cmd_ms": 40}),
    "imu_2": ("IMU tilt 2°", "IMU tilt", F3, {}, {"imu_deg": 2.0}),
    "imu_4": ("IMU tilt 4°", "IMU tilt", F3, {}, {"imu_deg": 4.0}),
    "play_0.5": ("gear play 0.5°", "gear play", F3, {}, {"backlash_deg": 0.5}),
    "play_1": ("gear play 1°", "gear play", F3, {}, {"backlash_deg": 1.0}),
    "hose_1": ("hose pull 1 N, constant", "hose pull", F3, {}, {"hose_n": 1.0}),
    "hose_3": ("hose pull 3 N, constant", "hose pull", F3, {}, {"hose_n": 3.0}),
    "hose_1v": ("hose pull 0 to 1 N, 4 s period", "hose pull", F3, {}, {"hose_n": 1.0, "hose_var": True}),
    "hose_3v": ("hose pull 0 to 3 N, 4 s period", "hose pull", F3, {}, {"hose_n": 3.0, "hose_var": True}),
    "zero_2": ("joint zero error ±2°", "zero error", F3, {}, {"zero_deg": 2.0}),
    "R": ("realistic: the middle values together", "combined", F3, *REAL),
    "P": ("pessimistic: the worst values together", "combined", F3, *PESS),
    **{f"R+{k}": (f"R with {lab}", "R plus one", F3, *r_plus(e, d)) for k, (lab, e, d) in R_PLUS.items()},
}
MODEL_IDS = [c for c in CASES if CASES[c][1] == "model"]
OAT_IDS = [c for c in CASES if CASES[c][1] not in ("model", "combined", "R plus one")]
COMBINED_IDS = [c for c in CASES if CASES[c][1] == "combined"]
R_PLUS_IDS = [c for c in CASES if CASES[c][1] == "R plus one"]


@lru_cache(maxsize=None)
def model(cid):
    """The model of a case: the ASMR robot with the payload of the case, and the case changes."""
    _, _, name, e, _ = CASES[cid]
    m = hw.load_model(name)
    if not e:
        return m
    body = [m.body(i).name for i in range(m.nbody)]
    if "armature" in e:  # hip and knee (6512 motor), ankle (5010 motor)
        m.dof_armature[6:18] = np.where(hw.IS_6512, *e["armature"])
    if "frictionloss" in e:
        m.dof_frictionloss[6:18] = e["frictionloss"]
    if "leg_mass" in e:
        legs = [i for i, b in enumerate(body) if b.startswith("leg_")]
        m.body_mass[legs] *= e["leg_mass"]
        m.body_inertia[legs] *= e["leg_mass"]
    if "com_dx" in e:  # the torso and all payload parts, so the CoM of the whole upper body moves by com_dx
        m.body_ipos[[i for i, b in enumerate(body) if b == "base" or b.startswith("payload_")], 0] += e["com_dx"]
    if "friction" in e:  # MuJoCo takes the larger friction of the two geoms in contact, so set all of them
        m.geom_friction[:, 0] = e["friction"]
    mujoco.mj_setConst(m, mujoco.MjData(m))  # derived masses and constraint scales after the changes
    return m


def drive(cid):
    return {**hw.BASE, **CASES[cid][4]}


@lru_cache(maxsize=None)
def session(path):
    return hw.session(path)


def job(kind, cid, pol, path, arg, seed, seconds=20.0):
    """kind "walk": one run with the command arg. kind "push": the largest standing push (N s) in direction arg
    with no fall in the next 3 s, by bisection. 0 when the robot falls without a push."""
    env = (model(cid), session(path))
    if kind == "walk":
        r = hw.simulate(drive(cid), s2s.COMMANDS[arg], seed, seconds, env=env)
        keep = ("fall", "vx", "wz", "sat", "peak", "rate", "qd_peak") + (("sat_j", "peak_j") if arg == "vx 0.3" else ())
        return {"kind": kind, "case": cid, "policy": pol, "cmd": arg, "seed": seed,
                **{k: None if r[k] is None else np.round(r[k], 4).tolist() for k in keep}}
    t0 = PUSH_T[seed]
    alive = lambda j: hw.simulate(drive(cid), (0.0, 0.0, 0.0), seed, t0 + 3.0, push=(t0, j, PUSH_DIRS[arg]),
                                  env=env)["fall"] is None
    lo, hi = 0.0, PUSH_HI
    if alive(0.0):
        for _ in range(PUSH_ITERS):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if alive(mid) else (lo, mid)
    return {"kind": kind, "case": cid, "policy": pol, "dir": arg, "seed": seed, "impulse": lo}


def jobs(policies, cases):
    out = [("push", c, p, str(f), k, s) for p, f in policies.items() for c in cases if c not in R_PLUS_IDS
           for k in PUSH_DIRS for s in range(len(PUSH_T))]
    return out + [("walk", c, p, str(f), k, s) for p, f in policies.items() for c in cases
                  for k in (WALK_CMDS if c in R_PLUS_IDS else s2s.COMMANDS) for s in range(SEEDS)]


def model_info(cid):
    """Mass (kg) and CoM height (m) in the standing pose."""
    m = model(cid)
    d = mujoco.MjData(m)
    d.qpos[:3], d.qpos[7:19] = s2s.DEFAULT_BASE_POSITION, s2s.DEFAULT
    mujoco.mj_forward(m, d)
    return float(m.body_subtreemass[1]), float(d.subtree_com[1][2])


def rel(path):
    p = Path(path).resolve()
    return str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p)


def main(policies, cases, merge):
    t0 = time.time()
    js = jobs(policies, cases)
    with Pool(WORKERS) as pool:
        rows = pool.starmap(job, js, chunksize=1)
    old = json.loads(JSON.read_text()) if merge and JSON.exists() else {"policies": {}, "walk": [], "push": [], "minutes": 0}
    done = {(c, p) for c in cases for p in policies}
    res = {"policies": {**old["policies"], **{p: rel(f) for p, f in policies.items()}},
           "cases": {c: {"label": v[0], "factor": v[1], "model": v[2], "changes": v[3], "drive": v[4]} for c, v in CASES.items()},
           "model_info": {c: model_info(c) for c in CASES},
           "minutes": old["minutes"] + (time.time() - t0) / 60}
    for kind in ("walk", "push"):
        res[kind] = [r for r in old[kind] if (r["case"], r["policy"]) not in done] + [r for r in rows if r["kind"] == kind]
    JSON.write_text(json.dumps(res))
    write(res)
    print(f"{len(js)} jobs in {(time.time() - t0) / 60:.1f} min")


# ---------------- tables ----------------
def runs(res, cid, pol, cmd=None):
    return [r for r in res["walk"] if r["case"] == cid and r["policy"] == pol and cmd in (None, r["cmd"])]


def mean_ok(rs, k):
    v = [r[k] for r in rs if r["fall"] is None and r[k] is not None]
    return float(np.mean(v)) if v else None


def fmt(x, p=2):
    return "-" if x is None else f"{x:.{p}f}"


def falls(res, cid, pol, cmds=None):
    """(falls, runs) of a case and policy, for all commands or only cmds."""
    rs = [r for r in runs(res, cid, pol) if cmds is None or r["cmd"] in cmds]
    return sum(r["fall"] is not None for r in rs), len(rs)


def falls_txt(res, cid, pol, cmds=None):
    f, n = falls(res, cid, pol, cmds)
    return f"{f} of {n}" if n else "-"


def push_min(res, cid, pol):
    """The smallest standing push survived (N s), over all directions and push times, and its direction."""
    ps = [p for p in res["push"] if p["case"] == cid and p["policy"] == pol]
    p = min(ps, key=lambda p: p["impulse"], default=None)
    return (p["impulse"], p["dir"]) if p else (None, "")


def paired(res, cid, ref="F3"):
    """Change in falls against ref over all policies, and the exact McNemar p of the paired runs."""
    fell = lambda c: {(r["policy"], r["cmd"], r["seed"]): r["fall"] is not None for r in res["walk"] if r["case"] == c}
    a, b = fell(ref), fell(cid)
    keys = a.keys() & b.keys()
    up, down = sum(b[k] and not a[k] for k in keys), sum(a[k] and not b[k] for k in keys)
    n = up + down
    p = 1.0 if n == 0 else min(1.0, sum(math.comb(n, i) for i in range(min(up, down) + 1)) / 2 ** (n - 1))
    return up - down, p


def per_pol(res, f):
    return " / ".join(f(p) for p in res["policies"])


def pfmt(p):
    return "< 0.01" if p < 0.01 else f"{p:.2f}"


def case_rows(res, ids, ref="F3", cmds=None):
    """A case table. Values per policy, in the order of the Falls header. cmds: only the runs of these commands."""
    pols = list(res["policies"])
    cols = [("vx at cmd 0.3 (m/s)", lambda c, q: fmt(mean_ok(runs(res, c, q, "vx 0.3"), "vx")))]
    if cmds:
        both = lambda c, q, k: fmt(mean_ok(runs(res, c, q, "vx 0.2 + yaw 0.5"), k))
        cols.append(("vx, yaw rate at cmd 0.2 + 0.5", lambda c, q: f"{both(c, q, 'vx')}, {both(c, q, 'wz')}"))
    else:
        cols += [("Yaw rate, turn on the spot 0.5 (rad/s)", lambda c, q: fmt(mean_ok(runs(res, c, q, "yaw 0.5"), "wz"))),
                 ("Smallest standing push (N s)", lambda c, q: fmt(push_min(res, c, q)[0], 1))]
    L = [f"| Case | Falls ({' / '.join(pols)}) | Change in falls, all policies | p | " + " | ".join(h for h, _ in cols) + " |",
         "|---|---|---|---|" + "---|" * len(cols)]
    for c in ids:
        ch, p = paired(res, c, ref)
        L.append(f"| {c}: {CASES[c][0]} | {per_pol(res, lambda q: falls_txt(res, c, q, cmds))} "
                 f"| {'-' if c == ref else f'{ch:+d}'} | {'-' if c == ref else pfmt(p)} | "
                 + " | ".join(per_pol(res, lambda q: f(c, q)) for _, f in cols) + " |")
    return L


def tables(res):
    pols = list(res["policies"])
    info = res["model_info"]
    L = ["### Policies", "", "| Name | File |", "|---|---|"] + [f"| {p} | `{f}` |" for p, f in res["policies"].items()]
    L += ["", "### 1. Payload comparison (nominal drive: hw_risks case 1)", "",
          f"Falls in {4 * SEEDS} runs. Speeds: mean of the runs without a fall, from 5 s to 20 s. "
          "Push: the smallest standing push survived, over 4 directions and 2 push times, and its direction.", "",
          "| Payload | Mass (kg) | CoM height (m) | Policy | Falls | vx at cmd 0.3 (m/s) | Yaw rate, turn on the spot 0.5 "
          "(rad/s) | vx, yaw rate at cmd 0.2 + 0.5 | Smallest standing push (N s) |", "|---|---|---|---|---|---|---|---|---|"]
    for c in MODEL_IDS:
        for q in pols:
            pv, pd = push_min(res, c, q)
            both = runs(res, c, q, "vx 0.2 + yaw 0.5")
            L.append(f"| {c}: {CASES[c][0]} | {info[c][0]:.2f} | {info[c][1]:.3f} | {q} | {falls_txt(res, c, q)} "
                     f"| {fmt(mean_ok(runs(res, c, q, 'vx 0.3'), 'vx'))} | {fmt(mean_ok(runs(res, c, q, 'yaw 0.5'), 'wz'))} "
                     f"| {fmt(mean_ok(both, 'vx'))}, {fmt(mean_ok(both, 'wz'))} | {fmt(pv, 1)} {pd} |")
    L += ["", "### 2. Joint torque while walking (cmd vx 0.3, nominal drive)", "",
          "Share of the time at the torque limit of the joint (18.1 Nm hip and knee, 22.6 Nm ankle), mean of both legs and all 10 runs, "
          "from 1 s to the end or the fall. Peak: the largest torque in the 10 runs.", "",
          "| Payload | Policy | " + " | ".join(f"{j} (%)" for j in JOINT_TYPES) + " | Peak hip pitch (Nm) | Peak knee (Nm) |",
          "|---|---|" + "---|" * (len(JOINT_TYPES) + 2)]
    for c in MODEL_IDS:
        for q in pols:
            rs = runs(res, c, q, "vx 0.3")
            if not rs:
                continue
            s = np.mean([r["sat_j"] for r in rs], axis=0)
            pk = np.max([r["peak_j"] for r in rs], axis=0)
            L.append(f"| {c} | {q} | " + " | ".join(f"{(s[i] + s[i + 6]) / 2:.1f}" for i in range(6))
                     + f" | {max(pk[2], pk[8]):.1f} | {max(pk[3], pk[9]):.1f} |")
    L += ["", "### 3. One change at a time on F3 (nominal drive)", "",
          "Change in falls: falls of the case minus falls of F3 nominal, summed over the policies. "
          "p: exact McNemar test of the paired runs (same policy, command and seed). "
          "A p below 0.05 means that the change is probably not noise.", ""]
    L += case_rows(res, ["F3"] + OAT_IDS)
    L += ["", "### 4. Factors sorted by the effect of one change on F3", "",
          "Sorted by the largest change in falls of any tested value, over all policies. The main ranking at the top also uses R.", "",
          "| Rank | Factor | Values tested: change in falls (p) |", "|---|---|---|"]
    by = {}
    for c in OAT_IDS:
        by.setdefault(CASES[c][1], []).append((c, *paired(res, c)))
    rank = sorted(by.items(), key=lambda kv: (-max(x[1] for x in kv[1]), min(x[2] for x in kv[1])))
    L += [f"| {i} | {f} | " + ", ".join(f"{c}: {ch:+d} (p {pfmt(p)})" for c, ch, p in v) + " |"
          for i, (f, v) in enumerate(rank, 1)]
    L += ["", "### 5. Combined cases on F3", ""] + case_rows(res, ["F3"] + COMBINED_IDS)
    L += ["", "### 6. The realistic case R with one value moved to its pessimistic value", "",
          f"These cases run only the two walking commands ({' and '.join(WALK_CMDS)}), 10 seeds each, without the push test. "
          "The row of R counts the same 20 runs per policy. Change and p are against R. Sorted by the change in falls.", ""]
    L += case_rows(res, ["R"] + sorted(R_PLUS_IDS, key=lambda c: -paired(res, c, "R")[0]), "R", WALK_CMDS)
    L += ["", "### 7. Falls per command (stand / vx 0.3 / yaw 0.5 / vx 0.2 + yaw 0.5)", "",
          "| Case | " + " | ".join(pols) + " |", "|---|" + "---|" * len(pols)]
    for c in CASES:
        L.append(f"| {c} | " + " | ".join(" / ".join(fmt(sum(r["fall"] is not None for r in runs(res, c, q, k)), 0)
                                                    if runs(res, c, q, k) else "-" for k in s2s.COMMANDS) for q in pols) + " |")
    return "\n".join(L) + "\n"


def plot(res, out):
    """Falls and the smallest standing push per case and policy. F3 changes sorted by the change in falls."""
    pols = list(res["policies"])
    blocks = [MODEL_IDS, sorted(OAT_IDS, key=lambda c: -paired(res, c)[0]), COMBINED_IDS,
              sorted(R_PLUS_IDS, key=lambda c: -paired(res, c, "R")[0])]
    ids = [c for b in blocks for c in b]
    y = -np.arange(len(ids), dtype=float) - np.repeat(np.arange(len(blocks)), [len(b) for b in blocks]) * 0.8  # gaps
    cols = [report.S1, report.S2, report.S3, report.INK2]
    w = 0.8 / len(pols)
    rate = lambda c, q: 100 * falls(res, c, q)[0] / falls(res, c, q)[1] if falls(res, c, q)[1] else np.nan
    pushed = lambda c, q: np.nan if push_min(res, c, q)[0] is None else push_min(res, c, q)[0]
    fig, axs = plt.subplots(1, 2, figsize=(13, 0.25 * len(ids) + 1.8), sharey=True, layout="constrained")
    axs[0].set_ylim(y.min() - 0.7, y.max() + 0.7)
    for ax, title, f in ((axs[0], "Runs that fall (%)", rate), (axs[1], "Smallest standing push survived (N s)", pushed)):
        for i, q in enumerate(pols):
            ax.barh(y + 0.4 - w * (i + 0.5), [f(c, q) for c in ids], height=w * 0.9, color=cols[i % 4], label=q)
        for k in np.cumsum([len(b) for b in blocks])[:-1]:
            ax.axhline((y[k - 1] + y[k]) / 2, color=report.INK2, lw=0.6)
        ax.set_title(title, fontsize=10, loc="left")
        ax.grid(axis="x", color=report.GRID)
        ax.set_axisbelow(True)
    walk = len(ids) - len(blocks[-1])  # first row of the R+ block
    for i, q in enumerate(pols):
        if not np.isnan(pushed("F3", q)):
            axs[1].axvline(pushed("F3", q), color=cols[i % 4], lw=1, ls="--")  # F3 nominal
        f, n = falls(res, "R", q, WALK_CMDS)
        if n and blocks[-1]:  # R on the same walking runs as the R+ cases
            axs[0].vlines(100 * f / n, y[-1] - 0.5, y[walk] + 0.5, color=cols[i % 4], lw=1, ls="--")
    axs[0].set_yticks(y, [f"{c}: {CASES[c][0]}" for c in ids], fontsize=8)
    axs[0].legend(frameon=False, fontsize=9, loc="lower right")
    fig.suptitle("Gap study of F3. Blocks from the top: the models with the nominal drive, one change at a time on F3, "
                 "the combined cases,\nR with one value moved to its pessimistic value (walking commands only, no push test). "
                 "Dashed lines: F3 nominal (push), R on the same walking runs (bottom block).", fontsize=10, x=0.01, ha="left")
    fig.savefig(out, dpi=130, bbox_inches="tight")
    plt.close(fig)


def write(res, md=s2s.OUT / "gap_study.md", png=s2s.OUT / "gap_study.png"):
    md.write_text(REPORT_HEAD + tables(res) + REPORT_TAIL.format(minutes=res["minutes"], workers=WORKERS))
    plot(res, png)
    print(f"wrote {md} and {png.name}")


REPORT_HEAD = """# Sim-to-real gap study of the trolley version F3

**Status:** tested in simulation (MuJoCo 3.15.0, CPU only). Nothing here is measured on board.

F3 is the trolley version of the robot (team decision of 2026-10-29).
The water tank and the pump hang high on a trolley that a teammate pushes behind the robot. Only the hose comes to the robot.
This study asks two questions:

1. How much does F3 help, compared with F2 and the bare biped?
2. Which differences between the simulation and the real robot most likely stop the real robot from walking?

The answers set the order of the hardware measurements and the randomization ranges of the next training.

## Answers

Status: tested in simulation, with the shipped BHL policy, f1_v1 and f3_v1. Details in "Findings" at the end.

1. F3 helps, mostly with torque margin. With the nominal drive, F2 and F3 fall at most once in 40 runs.
   F3 walks faster (0.25 against 0.22 m/s, shipped policy) and spends less time at the torque limit.
   F3 also tolerates drive errors better. 40 ms of command latency gives 2 falls in 40 runs on F3.
   On F2, it gives 13 (shipped policy, `hw_risks.py` case 5c).
2. The two lighter options (one 6S pack, motor power by cable) help a little more. They do not change the falls.
3. Alone, only three changes cause clear falls on F3: a constant 3 N hose pull, a 4° IMU tilt and 15 % less torque per amp.
4. Small errors add up. With the realistic case R, the shipped policy falls in 5 of 40 runs. f1_v1 does not fall.
   In R, five values cause many falls when they move to their pessimistic value.
   These are: torque per amp × 0.85, CoM 2 cm back, 3 N constant hose pull, 40 ms command latency, 4° IMU tilt.
5. The mechanical details matter less than expected: armature, joint friction, foot friction, leg mass and gear play.
6. With everything at the pessimistic value (case P), all 80 runs fall within 1 s. With a 9 Nm limit, 77 of 80 runs still fall.
7. Our policy f3_v1 trained with the recommended ranges below. It does not fall in R or in the 25 cases with one change.
   It falls once in the 240 runs of R with one value at its pessimistic value (CoM 2 cm back).
   It still falls in all 40 runs of P, and in 26 of 40 runs of P with a 9 Nm limit.
   These cases test the same kinds of errors as its training. So they do not prove that it walks on the robot.

## Ranking of the differences

Ranked by the extra falls in R (the realistic robot), then by the effect alone on F3 nominal.
The shipped policy and f1_v1 together. f3_v1 is not in the ranking, because it trained with these errors (answer 7).
"In R": 20 walking runs per policy. "Alone": 40 runs per policy. p is given when it is below 0.05.

| Rank | Difference | Extra falls alone (of 80) | Extra falls in R (of 40) | How to measure it |
|---|---|---|---|---|
| 1 | Torque per amp 15 % low (K, D and limit) | +8 (p < 0.01) | +22 (p < 0.01) | lever with a known mass, drive in torque mode |
| 2 | Torso CoM 2 cm back | +1 | +21 (p < 0.01) | weighed parts, then a balance test |
| 3 | Hose pull 3 N, constant | +12 (p < 0.01) | +20 (p < 0.01) | spring scale at the nozzle mount, trolley moving |
| 4 | IMU tilt 4° | +11 (p < 0.01) | +17 (p < 0.01) | gravity vector with the torso on a level table |
| 5 | Command latency 40 ms | +2 | +17 (p < 0.01) | time stamps in the policy loop |
| 6 | Observation latency 20 ms | +1 | +8 (p < 0.01) | time stamps in the policy loop |
| 7 | Joint zero error ±2° | +1 | +4 | zero jig |
| 8 | Armature 0.0224 / 0.0074 kg m² | 0 | +3 | free swing or step test of one actuator |
| 9 | Foot friction 0.4 | 0 | +2 | incline test on the demo floor |
| 10 | Joint friction 0.5 Nm | 0 | +1 | the same lever test as rank 1 |
| 10 | Leg mass +10 % | +1 | +1 | scale |
| 12 | Gear play 1° | +1 | 0 | lock the joint, rock the motor, read the encoder |

## Recommended order of hardware measurements

Status: planned. Measure first the differences with the largest effect. Each step gives the limit from this study.

1. **Torque per amp** of one hip or knee drive and one ankle drive. Put the drive in torque mode and hang a known mass on a lever.
   Find the current that just lifts the mass and the current that just lets it drop. Their mean gives the torque per amp.
   Half their difference gives the joint friction. Then set `current_lim` so that the joint really gets 6 Nm, or 9 Nm.
2. **Torso CoM in x.** Compute it from the weighed parts (planned), then check it with a balance test of the built torso.
   2 cm back already costs many falls in R. Move parts forward if the CoM is behind the model value.
3. **Hose pull** with the trolley. Put a spring scale between the hose and the nozzle mount while a teammate walks the trolley.
   Keep the mean pull below 1 N with a slack hose. A hose clamp lower on the torso would also tilt the robot less (not tested).
4. **IMU tilt.** Put the torso on a level table and read the gravity vector. This takes 5 minutes.
   Correct the offset in software if it is above 1°. 2° changed nothing, 4° caused falls.
5. **Loop latency** on the ROCK 3C. Log the times of the encoder sample, the policy step and the CAN send.
   Keep the total from the sensor sample to the new drive target at 20 ms or less, as `hw_risks.py` recommends.
6. **Joint zero error.** Check each joint with a jig after the zero procedure. Aim for ±1°, as `hw_risks.py` recommends.
7. **Last:** armature, foot friction, leg masses and gear play. They changed the falls by 3 or less in this study.

## Recommended randomization ranges for the next training

Status: planned. The column "f1_v1" gives the ranges of its last training stage (`policies/f1_v1/train_ranges.json`, `archive/rl_f1/f1_env.py`).
The recommended ranges cover the tested values with some margin.
f3_v1 (stage E) uses them, with a constant hose pull of 0 to 2.5 N plus a changing pull of 0 to 1.5 N ([archive/rl_f1](../../../archive/rl_f1/README.md)).

| Term | f1_v1 | Recommended | Reason |
|---|---|---|---|
| Torque per amp: one scale on K, D and the torque limit together | none (K and D × 0.8 to 1.2 for each joint, limit 6 to 9 Nm) | × 0.8 to 1.2, on top of the K and D ranges | rank 1. A torque constant error moves K, D and the limit together |
| Torso CoM, torso and payload together | ±3 cm in x, y and z | keep ±3 cm | rank 2 |
| Hose pull at the nozzle (0.03, 0, 0.944) m, horizontal, backward in the heading frame | none (recoil 0 to 1.5 N along the nozzle, constant force on the torso ±1 N) | 0 to 4 N: a constant part per episode plus a slow change (period 2 to 8 s) | rank 3. The constant part hurts most |
| IMU tilt: one fixed rotation of the gravity and gyro inputs, random axis | none (only the noise of ±0.05) | 0 to 5° | rank 4. The noise does not teach a constant offset |
| Command latency | 0 to 20 ms | 0 to 40 ms | rank 5. f1_v1, trained to 20 ms, falls much less than the shipped policy at 40 ms |
| Observation latency | 0 to 20 ms | keep 0 to 20 ms | rank 6 |
| Joint zero error | ±0.05 rad (±2.9°) | keep | rank 7 |
| Armature, hip and knee / ankle | 0.004 to 0.008 kg m², all joints | 0.007 to 0.025 / 0.002 to 0.008 kg m² | small effect, but the true value is unknown |
| Joint friction | 0.05 to 0.15 Nm | 0.05 to 0.5 Nm | no extra falls, but it slows the turn on the spot |
| Leg link mass | none | × 0.9 to 1.1 | small effect |
| Foot friction | 0.4 to 1.2 | keep | 0.4 changed nothing on a flat floor |
| Gear play | none | not needed now | no effect with this simple model |

**Assumed:** the realistic case R is a guess of the real robot. Measure the ranked values, then run the study again with them.
**Not verified:** the hose model is a horizontal force at the nozzle. A real hose also pulls down and sideways, and it moves with the pan-tilt.
**Not verified:** the motor speed limit is not in the model. The fastest joint in the F3 nominal runs reached 13.3 rad/s.
At 22.2 V, the no-load speed at the joint is about 23 rad/s (150KV, hip and knee) and 17 rad/s (110KV, ankle).
**Open:** the BHL training config sets a velocity limit of 10 rad/s. We do not know if Isaac Lab applied it to the joints.
**Open:** every run starts with the BHL 4 cm drop. A start from a gantry is gentler. P falls during this phase.

## Setup

- Loop: `hw_risks.simulate()`. This is the `run.py` policy loop with the drive model of `hw_risks.py`.
  Policy at 25 Hz, physics at 2 kHz, the training observation noise, and the BHL start with a 4 cm drop.
- Nominal drive (`hw_risks.py` case 1): K = 20 Nm/rad, D = 2 Nm s/rad, 6 Nm torque limit, PASSTHROUGH input mode.
  No zero error, no latency.
- Policies: the shipped BHL policy and our policy f1_v1. f1_v1 was trained on F1 (15.24 kg). Both have the same 45 inputs and 12 outputs.
- Commands: stand, vx 0.3 m/s, turn on the spot at 0.5 rad/s, and vx 0.2 m/s + yaw 0.5 rad/s. 10 seeds per command, 20 s per run.
  So each case has 40 runs per policy.
- Seeds: the seed sets the sensor noise, the zero errors, the IMU tilt axis and the phase of the varying hose pull.
  All cases use the same seeds. So each run of a case pairs with one run of F3 nominal (same policy, command and seed).
- Push test: the robot stands with a zero command. A 0.1 s push hits the torso forward, back, left or right, at 3.0 s or 3.3 s.
  A bisection finds the largest push with no fall in the next 3 s (resolution 0.375 N s).
  "Smallest standing push" is the smallest of these 8 values. 0 means that the robot fell without a push.
- Fall: a torso tilt above 0.78 rad or a large height drop, as in `run.py`.
- Knee: F3 has no side tanks. So this study does not clip the knee target at 1.4 rad, unlike `archive/rl_f1/f1_eval.py`.

### The changes

| Change | Values tested | Nominal | Source of the values |
|---|---|---|---|
| Joint armature, hip and knee / ankle (kg m²) | 0.007 / 0.002, 0.014 / 0.0047, 0.0224 / 0.0074 | 0.005 / 0.005 | BHL Isaac Lab training config, the middle, BHL motor page |
| Joint friction (`frictionloss`, Nm) | 0.3, 0.5 | 0.1 | BHL model value and two higher guesses |
| Torque per amp: K, D and the torque limit together | × 0.85, × 1.15 | × 1 | BHL and the course config use different torque constants |
| Leg link mass and inertia, 12 bodies | +10 % (+0.82 kg) | CAD values | printed parts, screws and cables |
| Torso CoM in x: the torso and all payload parts | +2 cm, −2 cm | model | |
| Foot friction (all geoms) | 0.6, 0.4 | 1.0 | BHL training range 0.4 to 1.2 |
| Observation latency | 10 ms, 20 ms | 0 | `hw_risks.py` model |
| Command latency | 20 ms, 40 ms | 0 | `hw_risks.py` model |
| IMU mounting tilt | 2°, 4° | 0 | |
| Gear play (backlash) at the joint | 0.5°, 1° | 0 | model below |
| Hose pull at the nozzle | 1 N, 3 N, constant or from 0 to the value and back in 4 s | 0 | |
| Joint zero error | ±2° | 0 | `hw_risks.py` case 4b |

- Armature: the BHL motor page gives a reflected rotor inertia of 0.0224 kg m² (M6C12, hip and knee) and 0.00743 kg m² (5010, ankle).
  The page uses a thin-shell rotor, which probably gives too much. So the true value is probably between the training value and the page value.
- Torque per amp: the drive turns the gains into a current with its configured torque constant.
  A wrong torque constant scales K, D and the torque limit together. BHL uses 0.0919 and 0.1176 Nm/A, the course config 0.067 and 0.0836 Nm/A.
- IMU tilt: the IMU frame turns about a horizontal axis. The seed sets the direction of the axis.
  The policy sees the turned gravity and gyro vectors. The fall check uses the true torso.
- Hose pull: a horizontal force, backward in the heading frame, at the nozzle point (0.03, 0, 0.944) m of the torso frame.
  The seed sets the phase of the varying pull.
- Joint zero error: we added it because `hw_risks.py` found that ±2° matters on F2.

**Assumed:** the gear play model. It is simple, so read its results as a first estimate:

- The drive measures the rotor (the motor encoder) and turns the rotor. The rotor has no mass.
- The rotor pushes the joint only when it touches a wall of the play. Then the joint gets the drive torque, and the rotor turns with the joint.
- Between the walls, the joint gets no torque. The rotor turns at the speed where the drive torque is zero: K × (target − rotor) / D.
- The policy sees the rotor position and speed, as on the real robot.
- So the position loop has a dead band of ± half the play, and the measured joint position is off by up to ± half the play.
- Not in the model: the rotor inertia while it crosses the gap, the impact at the wall, and the gear stiffness.

### Combined cases

| Case | Changes |
|---|---|
| R: realistic | armature 0.014 / 0.0047, joint friction 0.3 Nm, legs +5 %, foot friction 0.6, observation latency 10 ms, command latency 20 ms, IMU tilt 2°, gear play 0.5°, hose 0 to 1 N (4 s), zero error ±1°. Torque per amp × 1 and torso CoM 0 (the middle of their ranges) |
| P: pessimistic | armature 0.0224 / 0.0074, joint friction 0.5 Nm, torque per amp × 0.85, legs +10 %, torso CoM 2 cm back, foot friction 0.4, observation latency 20 ms, command latency 40 ms, IMU tilt 4°, gear play 1°, hose 3 N constant, zero error ±2° |
| R9, P9 | R and P with a 9 Nm drive limit. In P9, the torque per amp error makes it 7.65 Nm at the joint |
| R+armature to R+zero | R with one value moved to its value in P. These 12 cases run only the two walking commands (20 runs per policy) and no push test, to save compute time |

- P takes the worse direction of the two-sided changes: the torso CoM 2 cm back and the constant hose pull.
  The one-change results confirmed both choices.

Metrics:

- Falls: runs that fall in 20 s. Speeds: the mean from 5 s to 20 s of the runs without a fall.
- Change in falls: the falls of a case minus the falls of F3 nominal, summed over the policies.
- p: the exact McNemar test of the paired runs. It counts the pairs where only one of the two runs falls.
  A p below 0.05 means that the change is probably not noise. A change of 5 falls or less never reaches p < 0.05.
- Torque at the limit: the share of the physics steps where a joint asks for more than its limit.

![Falls and the smallest standing push per case and policy](gap_study.png)

## Results

"""
REPORT_TAIL = """
## Findings

Status: tested in simulation, with the shipped policy, f1_v1 and f3_v1 (runs of 2026-10-30). Most important first.

1. **Small errors add up, and the margin is thin.** The realistic case R makes the shipped policy fall in 5 of 40 runs (p = 0.06).
   f1_v1 does not fall in R. In R, one value at its pessimistic value gives these extra falls in 40 paired walking runs (p < 0.01):
   - torque per amp × 0.85: +22
   - torso CoM 2 cm back: +21
   - constant 3 N hose pull: +20
   - IMU tilt 4°: +17
   - command latency 40 ms: +17
   - observation latency 20 ms: +8

   The other values give +4 or less.
2. **The CoM shift and the command latency hurt only together with other errors.** Alone on F3, they give 1 and 2 falls in 80 runs.
   In R, they give +21 and +17. So a test of one error at a time on an ideal robot misses them.
   Alone, the CoM 2 cm back already slows walking by 0.05 to 0.06 m/s.
3. **Knee torque is the common thread.** The three worst values all make the knee hit its 6 Nm limit more often.
   At vx 0.3 on F3, the knee is at the limit 21 % (shipped) and 16 % (f1_v1) of the time.
   With the CoM 2 cm back or a constant 3 N pull, it is 28 to 38 %. In R with one of the three worst values, it is 40 to 57 %.
   A 9 Nm limit cuts the falls of the shipped policy in R from 5 to 1 in 40.
4. **A constant backward hose pull is the strongest single change on F3.** 3 N gives +12 falls in 80 runs, and the walking speed drops by about half.
   f1_v1 even falls in 2 of 10 standing runs with this pull. A pull that swings from 0 to 3 N gives only +2. The mean pull matters most.
5. **A 4° IMU tilt causes falls, 2° does not.** 4° gives +11 falls in 80 runs. The policies then see a constant gravity error of 0.07.
   This is larger than the training noise of ±0.05.
6. **15 % less torque per amp causes falls alone too.** × 0.85 gives +8 falls in 80 runs and slows walking from 0.25 to 0.17 m/s (shipped).
   × 1.15 gives no falls and faster walking.
7. **F3 tolerates drive errors better than F2.** With the shipped policy and the same seeds, 40 ms of command latency gives 2 falls in 40 runs on F3.
   `hw_risks.py` case 5c gives 13 on F2. A zero error of ±2° gives 1 fall on F3 and 4 on F2 (case 4b).
8. **F3 gives more torque margin than F2, but the same falls with the nominal drive.** Both fall at most once in 40 runs.
   F3 walks 0.03 m/s faster with the shipped policy and 0.02 m/s faster with f1_v1.
   At vx 0.3, the hip yaw is at the limit 22.6 % of the time on F3 and 29.2 % on F2. The knee: 20.7 % and 26.5 % (shipped).
9. **The lighter options help a little.** One 6S pack or motor power by cable cut the time at the limit by 1 to 7 more percentage points.
   The smallest standing push of f1_v1 drops from 3.8 to 3.0 N s with the cable option. A lighter robot gains more speed from the same push.
   **Not verified:** the cable option needs a power cable from the trolley. This study does not model the pull of that cable.
10. **The pessimistic case P falls at once.** All 80 runs fall within 0.9 s, during or right after the start drop.
    With 9 Nm (7.65 Nm at the joint after the torque per amp error), 77 of 80 runs still fall.
    A one-off check (not in the script, shipped policy, 6 runs per variant) removed one change at a time from an earlier P
    with a hose pull from 0 to 3 N. No single removal stopped the falls.
    In the same check, the P model changes alone (armature, frictions, masses, CoM) gave no falls. The P drive and sensor changes alone made all 6 runs fall.
11. **The mechanical details did not change the falls.** Alone on F3, the armature (all three values), joint friction up to 0.5 Nm,
    foot friction down to 0.4, gear play up to 1° and legs +10 % give 1 fall or less in 80 runs each.
    In R, their pessimistic values give +3 or less. Joint friction changes the gait:
    0.5 Nm slows the turn on the spot of f1_v1 from 0.44 to 0.32 rad/s.
    The armature result surprised us. The BHL motor page value is about 4.5 times the model value at the hip and knee.
    It changed neither the falls nor the pushes.
12. **The two policies fail in different ways.** f1_v1 handles latency and IMU tilt much better. In R with 40 ms of command latency,
    f1_v1 falls in 3 of 20 walking runs and the shipped policy in 19. f1_v1 saw latency and zero errors in training.
    f1_v1 is weaker against a backward pull. In R with a constant 3 N pull, it falls in 15 of 20 runs, and the shipped policy in 10.
    f1_v1 also walks slower: 0.17 against 0.25 m/s on F3 at a 0.3 m/s command.
13. **Noise.** A change of 5 falls or less is not significant (p ≥ 0.06), even when all the changed runs go one way.
    The push values have a resolution of 0.375 N s and often move by one step between similar cases.
14. **f3_v1 passes R and the R+ cases.** f3_v1 trained with the recommended ranges (runs of 2026-10-30).
    It falls once in 240 R+ runs (CoM 2 cm back) and never in the 1000 runs with one change.
    In R, it walks at 0.18 m/s at a 0.3 m/s command. The shipped policy walks at 0.22 m/s and f1_v1 at 0.15 m/s.
    P still makes all its 40 runs fall. P has every value at its worst at the same time, and gear play, which is not in the training.
    **Not verified:** these cases use the same kinds of errors as the training of f3_v1, so they do not prove that it walks on the robot.

Run time of all results in `gap_study.json`: {minutes:.0f} min with {workers} worker processes, shared with a GPU training job.
"""


# ---------------- self-check ----------------
class Recorder:
    """Stands in for the ONNX session: zero actions. It keeps each observation and a copy of state (if given)."""

    def __init__(self, state=None):
        self.obs, self.state = [], state

    def run(self, _, feed):
        self.obs.append((feed["obs"][0].copy(), dict(self.state or {})))
        return [np.zeros((1, 12), np.float32)]


def clean(rec, seed):
    """The observations without the training noise: hw_risks.simulate draws it from default_rng(seed)."""
    rng = np.random.default_rng(seed)
    return [(o - (rng.uniform(-1, 1, 45) * s2s.NOISE).astype(np.float32), s) for o, s in rec.obs]


def quick():
    t0 = time.time()
    shipped = str(POLICIES["shipped"])
    # 1. The F3 nominal case gives the same run as hw_risks.py with MODEL = bhl_F3.
    hw.MODEL = F3
    a = hw.simulate(hw.BASE, (0.3, 0.0, 0.0), 0, 3.0)
    b = hw.simulate(drive("F3"), (0.3, 0.0, 0.0), 0, 3.0, env=(model("F3"), session(shipped)))
    assert a == b, (a, b)
    # 2. Model changes.
    nom = model("F3")
    mass = lambda m, f: sum(m.body_mass[i] for i in range(m.nbody) if f(m.body(i).name))
    legs = mass(nom, lambda b: b.startswith("leg_"))
    upper = mass(nom, lambda b: b == "base" or b.startswith("payload_"))
    assert abs(model("leg_mass").body_subtreemass[1] - nom.body_subtreemass[1] - 0.1 * legs) < 1e-9
    assert np.allclose(model("arm_page").dof_armature[6:18], np.where(hw.IS_6512, 0.0224, 0.00743))
    assert np.all(model("foot_0.4").geom_friction[:, 0] == 0.4) and np.all(model("fric_0.5").dof_frictionloss[6:] == 0.5)
    com = []
    for c in ("F3", "com_fwd"):
        d = mujoco.MjData(model(c))
        mujoco.mj_forward(model(c), d)
        com.append(d.subtree_com[1][0])
    assert abs(com[1] - com[0] - 0.02 * upper / nom.body_subtreemass[1]) < 1e-9, "torso CoM shift"
    # 3. IMU tilt: the policy sees gravity turned by the tilt, and a gyro vector of the same length.
    r0, r4 = Recorder(), Recorder()
    hw.simulate(drive("F3"), (0.0, 0.0, 0.0), 0, 0.4, env=(nom, r0))  # zero actions: same motion in both runs
    hw.simulate({**drive("F3"), "imu_deg": 4.0}, (0.0, 0.0, 0.0), 0, 0.4, env=(nom, r4))
    for (x, _), (z, _) in zip(clean(r0, 0), clean(r4, 0)):
        ang = math.degrees(math.acos(np.clip(np.dot(x[6:9], z[6:9]) / np.linalg.norm(x[6:9]) / np.linalg.norm(z[6:9]), -1, 1)))
        assert abs(ang - 4.0) < 0.05 and abs(np.linalg.norm(x[3:6]) - np.linalg.norm(z[3:6])) < 1e-5, ang
    # 4. Gear play: the encoder (rotor) stays within half the play of the joint, and a joint gets torque only when
    # the rotor sits at the wall on the side of that torque.
    state = {}

    def frames(m, d):  # every physics step: the joint position after the step, and the torque of the step
        state["q"], state["tau"] = d.qpos[7:19].copy(), d.ctrl.copy()
    frames.n = 0
    rec, h = Recorder(state), math.radians(1.0) / 2
    hw.simulate({**drive("F3"), "backlash_deg": 1.0}, (0.0, 0.0, 0.0), 0, 2.0, frames=frames, env=(nom, rec))
    pushing = []
    for o, s in clean(rec, 0)[1:]:
        gap = o[9:21] + s2s.DEFAULT - s["q"]
        on = np.abs(s["tau"]) > 1e-6
        assert np.all(np.abs(gap) <= h + 1e-5) and np.allclose(gap[on], h * np.sign(s["tau"][on]), atol=1e-5)
        pushing.append(on.mean())
    assert 0 < np.mean(pushing) < 1, "the play test needs joints at a wall and joints in the gap"
    # 5. Hose pull: during the start drop (no contact), the robot gains F t of backward momentum and the torque of F
    # at the nozzle about the CoM.
    fall = []

    def frames2(m, d):
        if d.time <= 0.05:
            mujoco.mj_subtreeVel(m, d)
            fall.append((d.time, d.ncon, d.subtree_linvel[1].copy(), d.subtree_angmom[1].copy(),
                         d.xpos[1][2] + hw.NOZZLE[2] - d.subtree_com[1][2]))
    frames2.n = 0
    hw.simulate({**drive("F3"), "hose_n": 3.0}, (0.0, 0.0, 0.0), 0, 0.08, frames=frames2, env=(nom, Recorder()))
    t, ncon, v, L, arm = fall[-1]
    t, M = t - s2s.PHYSICS_DT, nom.body_subtreemass[1]  # mj_subtreeVel uses the body speeds of the start of the step
    assert ncon == 0 and abs(v[2] + 9.81 * t) < 1e-3, "free fall"
    assert abs(v[0] + 3.0 * t / M) < 1e-6 and abs(L[1] + 3.0 * arm * t) < 0.02 * 3.0 * arm * t, (v, L)
    # 6. Every case runs, both kinds of job work, and the report builds.
    js = [("walk", c, "shipped", shipped, "vx 0.2 + yaw 0.5", 0, 6.0) for c in CASES] + [("push", "F3", "shipped", shipped, "back", 0)]
    with Pool(WORKERS) as pool:
        rows = pool.starmap(job, js, chunksize=1)
    res = {"policies": {"shipped": rel(shipped)}, "model_info": {c: model_info(c) for c in CASES}, "minutes": 0,
           "walk": [r for r in rows if r["kind"] == "walk"], "push": [r for r in rows if r["kind"] == "push"]}
    with tempfile.TemporaryDirectory() as tmp:
        write(res, Path(tmp) / "gap_study.md", Path(tmp) / "gap_study.png")
    print(f"check passed in {time.time() - t0:.0f} s: F3 nominal matches hw_risks.py, model changes, IMU tilt, "
          "gear play, hose pull, all cases run, report builds")


def arg(name):
    return sys.argv[sys.argv.index(name) + 1] if name in sys.argv else None


if __name__ == "__main__":
    if "--quick" in sys.argv:
        quick()
    elif "--report" in sys.argv:
        write(json.loads(JSON.read_text()))
    else:
        pols = {Path(arg("--policy")).stem: Path(arg("--policy")).resolve()} if arg("--policy") else POLICIES
        ids = arg("--cases").split(",") if arg("--cases") else list(CASES)
        assert all(c in CASES for c in ids), f"unknown case in {ids}"
        main(pols, ids, merge=bool(arg("--policy") or arg("--cases")))
