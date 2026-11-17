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
    old = json.loads(JSON.read_text(encoding="utf-8")) if merge and JSON.exists() else {"policies": {}, "walk": [], "push": [], "minutes": 0}
    done = {(c, p) for c in cases for p in policies}
    res = {"policies": {**old["policies"], **{p: rel(f) for p, f in policies.items()}},
           "cases": {c: {"label": v[0], "factor": v[1], "model": v[2], "changes": v[3], "drive": v[4]} for c, v in CASES.items()},
           "model_info": {c: model_info(c) for c in CASES},
           "minutes": old["minutes"] + (time.time() - t0) / 60}
    for kind in ("walk", "push"):
        res[kind] = [r for r in old[kind] if (r["case"], r["policy"]) not in done] + [r for r in rows if r["kind"] == kind]
    JSON.write_text(json.dumps(res), encoding="utf-8")
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
    fig.suptitle("Gap study of the F3 payload. Blocks from the top: the payloads with the nominal drive, one change at a time on F3, "
                 "the combined cases,\nR with one value moved to its pessimistic value (walking commands only, no push test). "
                 "Dashed lines: F3 nominal (push), R on the same walking runs (bottom block).", fontsize=10, x=0.01, ha="left")
    fig.savefig(out, dpi=130, bbox_inches="tight")
    plt.close(fig)


def write(res, md=s2s.OUT / "gap_study.md", png=s2s.OUT / "gap_study.png"):
    md.write_text(REPORT_HEAD + tables(res) + REPORT_TAIL.format(minutes=res["minutes"], workers=WORKERS),
                  encoding="utf-8")
    plot(res, png)
    print(f"wrote {md} and {png.name}")


REPORT_HEAD = """# Sim-to-real gap study of the F3 payload on the ASMR robot

**Status:** tested in simulation (MuJoCo, CPU only). Nothing here is measured on board.

The robot is the ASMR 2026 course robot (`source/robot_models`), walking with a policy of `policies/` (baseline unless
`--policy NAME` adds another). F3 is the payload of 47 parts, 4.14 kg (`source/robot_models/robots/asmr2026_payload.py`).
This study asks two questions:

1. How much does the payload matter (bare robot, the 1 kg box of the training, F3)?
2. Which differences between the simulation and the real robot most likely stop the real robot from walking?

The answers set the order of the hardware measurements and the randomization ranges of the next training.

**Findings:** see "Findings" at the end. The older study on the BHL biped (shipped BHL policy, f1_v1, f3_v1) is in `archive/sim2sim_bhl/`.

## Setup

- Loop: `hw_risks.simulate()`. This is the `run.py` policy loop with the drive model of `hw_risks.py`.
  Policy at 25 Hz, physics at 2 kHz, the training observation noise, the standing start of `run.py`.
- Nominal drive (`hw_risks.py` case 1): the gains and torque limits of the ASMR MJCF (K 90 / 22 Nm/rad, D 5.7 / 0.49 Nm s/rad,
  18.1 / 22.6 Nm for hip and knee / ankle), PASSTHROUGH input mode. No zero error, no latency.
- Commands: stand, vx 0.3 m/s, turn on the spot at 0.5 rad/s, and vx 0.2 m/s + yaw 0.5 rad/s. 10 seeds per command, 20 s per run.
  So each case has 40 runs per policy.
- Seeds: the seed sets the sensor noise, the zero errors, the IMU tilt axis and the phase of the varying hose pull.
  All cases use the same seeds. So each run of a case pairs with one run of F3 nominal (same policy, command and seed).
- Push test: the robot stands with a zero command. A 0.1 s push hits the torso forward, back, left or right, at 3.0 s or 3.3 s.
  A bisection finds the largest push with no fall in the next 3 s (resolution 0.375 N s).
  "Smallest standing push" is the smallest of these values. 0 means that the robot fell without a push.
- Fall: a torso tilt above 0.78 rad or a large height drop, as in `run.py`.

### The changes

The nominal values are those of the ASMR MJCF: armature 0.0228 (hip, knee) / 0.0056 (ankle) kg m², joint friction 0.38 / 0.32 Nm,
measured leg masses, foot friction 1.0 (floor).

| Change | Values tested | Source of the values |
|---|---|---|
| Joint armature, hip and knee / ankle (kg m²) | 0.007 / 0.002 (BHL training value), 0.014 / 0.0047, 0.034 / 0.0085 (1.5 × nominal) | guesses around the nominal |
| Joint friction (`frictionloss`, Nm) | 0.3, 0.5 | guesses around the nominal |
| Torque per amp: K, D and the torque limit together | × 0.85, × 1.15 | the drive turns the gains into a current with its configured torque constant |
| Leg link mass and inertia, 12 bodies | +10 % | printed parts, screws and cables |
| Torso CoM in x: the torso and all payload parts | +2 cm, −2 cm | |
| Foot friction (all geoms) | 0.6, 0.4 | |
| Observation latency | 10 ms, 20 ms | `hw_risks.py` model |
| Command latency | 20 ms, 40 ms | `hw_risks.py` model |
| IMU mounting tilt | 2°, 4° | |
| Gear play (backlash) at the joint | 0.5°, 1° | model below |
| Hose pull at the nozzle (F3 layout: -0.02, 0, 0.93 m) | 1 N, 3 N, constant or from 0 to the value and back in 4 s | |
| Joint zero error | ±2° | `hw_risks.py` case 4b |

- IMU tilt: the IMU frame turns about a horizontal axis. The seed sets the direction of the axis.
  The policy sees the turned gravity and gyro vectors. The fall check uses the true torso.
- Hose pull: a horizontal force, backward in the heading frame, at the nozzle point. The seed sets the phase of the varying pull.

**Assumed:** the gear play model. It is simple, so read its results as a first estimate:

- The drive measures the rotor (the motor encoder) and turns the rotor. The rotor has no mass.
- The rotor pushes the joint only when it touches a wall of the play. Then the joint gets the drive torque, and the rotor turns with the joint.
- Between the walls, the joint gets no torque. The rotor turns at the speed where the drive torque is zero: K × (target − rotor) / D.
- The policy sees the rotor position and speed, as on the real robot.
- Not in the model: the rotor inertia while it crosses the gap, the impact at the wall, and the gear stiffness.

### Combined cases

| Case | Changes |
|---|---|
| R: realistic | foot friction 0.6, observation latency 10 ms, command latency 20 ms, IMU tilt 2°, gear play 0.5°, hose 0 to 1 N (4 s), zero error ±1° |
| P: pessimistic | armature 0.034 / 0.0085, joint friction 0.5 Nm, torque per amp × 0.85, legs +10 %, torso CoM 2 cm back, foot friction 0.4, observation latency 20 ms, command latency 40 ms, IMU tilt 4°, gear play 1°, hose 3 N constant, zero error ±2° |
| R+armature to R+zero | R with one value moved to its value in P. These 12 cases run only the two walking commands (20 runs per policy) and no push test, to save compute time |

R is a guess of the real robot. Measure the ranked values, then run the study again with them.

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

Not written yet. Read the tables above: section 3 ranks the single changes, section 6 shows which value hurts most in R.
After the run, write here the ranking, the recommended order of hardware measurements, and the randomization ranges for the next training.

Run time of all results in `gap_study.json`: {minutes:.0f} min with {workers} worker processes.
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
    shipped = str(POLICIES[policy_select.DEFAULT])
    # 1. The F3 nominal case gives the same run as hw_risks.py with MODEL = f3.
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
    assert np.allclose(model("arm_high").dof_armature[6:18], np.where(hw.IS_6512, 0.034, 0.0085))
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
    # 5. Hose pull: the force and torque that the drive loop sets on the base are those of a force F at the nozzle point
    # (mj_applyFT of F at the point gives the same generalized force).
    seen, prev = [], []  # the loop sets the force from the body positions of the previous step, so compare with those

    def frames2(m, d):
        base = m.body("base").id
        if prev:
            xpos, xmat, xipos = prev[-1]
            R = xmat.reshape(3, 3)
            ang = math.atan2(R[1, 0], R[0, 0])
            F = -3.0 * np.array([math.cos(ang), math.sin(ang), 0.0])
            want, got = np.zeros(m.nv), np.zeros(m.nv)
            mujoco.mj_applyFT(m, d, F, np.zeros(3), xpos + R @ hw.NOZZLE, base, want)
            mujoco.mj_applyFT(m, d, d.xfrc_applied[base, :3], d.xfrc_applied[base, 3:], xipos, base, got)
            seen.append(np.abs(want - got).max())
        prev.append((d.xpos[base].copy(), d.xmat[base].copy(), d.xipos[base].copy()))
    frames2.n = 0
    hw.simulate({**drive("F3"), "hose_n": 3.0}, (0.0, 0.0, 0.0), 0, 0.5, frames=frames2, env=(nom, Recorder()))
    assert seen and max(seen) < 1e-6, max(seen)
    # 6. Every case runs, both kinds of job work, and the report builds.
    js = [("walk", c, policy_select.DEFAULT, shipped, "vx 0.2 + yaw 0.5", 0, 6.0) for c in CASES] + [("push", "F3", policy_select.DEFAULT, shipped, "back", 0)]
    with Pool(WORKERS) as pool:
        rows = pool.starmap(job, js, chunksize=1)
    res = {"policies": {policy_select.DEFAULT: rel(shipped)}, "model_info": {c: model_info(c) for c in CASES}, "minutes": 0,
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
        write(json.loads(JSON.read_text(encoding="utf-8")))
    else:
        pols = {ASKED_POLICY: policy_select.onnx(ASKED_POLICY, robot="asmr2026")} if ASKED_POLICY else POLICIES
        ids = arg("--cases").split(",") if arg("--cases") else list(CASES)
        assert all(c in CASES for c in ids), f"unknown case in {ids}"
        main(pols, ids, merge=bool(ASKED_POLICY or arg("--cases")))
