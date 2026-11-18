"""Payload check of the ASMR robot with the F3 parts: statics, walking and push test.

    uv run python sim/payload/final_check.py                    # about 10-20 min on 8 cores -> results/final_f3.md, final_f3.json
    uv run python sim/payload/final_check.py --policy NAME      # another policy folder of policies/ (default baseline)
    uv run python sim/payload/final_check.py check              # self-check: statics, F3 stands 3 s (about 10 s)
    MUJOCO_GL=egl uv run python sim/payload/final_check.py video   # results/final_f3_walk.mp4, final_f3_push.mp4 (ffmpeg)

The robot is the MJCF of source/robot_models (run.py) with a payload option: none, box_1kg (the training default) or
f3 (the 47 parts of source/robot_models/data/f3_layout.json). The three are compared in each table. The drive model (gains,
latency, current limit) is in sim/sim2sim/hw_risks.py, the motor battery time in docs/hardware/power_calc.py.
"""
import json
import os
import sys
from multiprocessing import Pool
from pathlib import Path

for _i, _a in enumerate(sys.argv):  # --policy NAME is read before run.py loads the policy
    if _a == "--policy" and _i + 1 < len(sys.argv):
        os.environ["SIM2SIM_POLICY"] = sys.argv[_i + 1]
        del sys.argv[_i:_i + 2]
        break

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent / "sim2sim")]
import stability as tk  # noqa: E402

OUT = HERE / "results"
LIMITS = (None, 12.0)  # Nm cap on every joint: None = the effort limits of the MJCF (18 / 22.6 Nm), 12 Nm = 67 % of 18 Nm


def label(limit):
    return "MJCF" if limit is None else f"{limit:.0f} Nm"


def tables(res):
    s2s = tk.s2s
    L = ["# ASMR robot with the F3 parts: payload check (tested in simulation)", "",
         f"Robot: `{s2s.MODEL}` (MJCF of `source/robot_models`) with the payload options of "
         "`source/robot_models/robots/asmr2026_payload.py`. F3: the 47 parts of `source/robot_models/data/f3_layout.json` (4.14 kg). "
         f"Policy: `{s2s.POLICY.parent.name}` (`{s2s.POLICY.name}`), the PD gains of the MJCF.", "",
         "## Statics", "", "Joint torques that hold the pose, feet flat. The single-leg pose sways the pelvis over the stance "
         "foot as far as the hip roll range allows (the CoM must stay on the sole: `ok`).", "",
         "| Model | Mass (kg) | Added (kg) | CoM x / y (mm) | CoM height (m) | Peak torque, stand (Nm) | "
         "Peak torque, crouch (Nm) | Peak pitch torque, one leg (Nm) | Peak torque, one leg, any joint (Nm) | One leg ok |",
         "|---|---|---|---|---|---|---|---|---|---|"]
    for v, r in res["static"].items():
        L.append(f"| {r['label']} | {r['mass']:.2f} | {r['add']:+.2f} | {r['com_dx'] * 1000:+.0f} / {r['com_dy'] * 1000:+.0f} | "
                 f"{r['com_h']:.3f} | {r['stand']:.1f} | {r['crouch']:.1f} | {r['single_pitch']:.1f} | {r['single_all']:.1f} | "
                 f"{'yes' if r['single_ok'] else 'no'} |")
    L += ["", f"Effort limits of the MJCF: {s2s.EFFORT_LIMITS.min():.1f} Nm (hip, knee), {s2s.EFFORT_LIMITS.max():.1f} Nm (ankle).", "",
          "## Walking, 20 s with observation noise", "",
          "| Model | Torque cap | stand | vx 0.3 | vx 0.2 + yaw 0.5 |", "|---|---|---|---|---|"]
    for lab, v in tk.ROWS:
        for lim in LIMITS:
            cells = []
            for c in tk.COMMANDS:
                w = next(w for w in res["walk"] if w["model"] == v and w["cmd"] == c and w["limit"] == lim)
                cells.append(f"falls at {w['fall']:.1f} s" if w["fall"] else f"vx {w['vx']:+.2f}, wz {w['wz']:+.2f}")
            L.append(f"| {lab} | {label(lim)} | " + " | ".join(cells) + " |")
    L += ["", "## Push test: largest push with no fall (N s, worst of 3 push times)", "",
          "| Model | " + " | ".join(f"{c}, {d}" for c in tk.PUSH_CMDS for d in tk.PUSH_DIRS) + " |",
          "|---|" + "---|" * (len(tk.PUSH_CMDS) * len(tk.PUSH_DIRS))]
    for lab, v in tk.ROWS:
        L.append(f"| {lab} | " + " | ".join(f"{tk.push_worst(res['push'], v, c, d):.1f}" for c in tk.PUSH_CMDS
                                            for d in tk.PUSH_DIRS) + " |")
    L += ["", "## Findings and recommendation", "", "Not written yet: this check has not been run on the ASMR robot. "
          "The F2 check of the Berkeley model (4 models, shipped Berkeley policy) is in `archive/payload_f2/final_f2.md`.", ""]
    return "\n".join(L)


def main():
    res = {"static": tk.static_rows()}
    print("statics done", flush=True)
    models_all = [v for _, v in tk.ROWS]
    with Pool(min(8, os.cpu_count() or 1)) as pool:
        res["walk"] = pool.starmap(tk.walk_job, [(v, c, lim) for v in models_all for c in tk.COMMANDS for lim in LIMITS])
        print("walking done", flush=True)
        res["push"] = pool.starmap(tk.push_job, [(v, c, d, t0) for v in models_all for c in tk.PUSH_CMDS
                                                 for d in tk.PUSH_DIRS for t0 in tk.PUSH_T], chunksize=1)
        print("push done", flush=True)
    md = tables(res)
    OUT.mkdir(exist_ok=True)  # results/ is not in git while it is empty
    (OUT / "final_f3.json").write_text(json.dumps(res), encoding="utf-8")
    (OUT / "final_f3.md").write_text(md, encoding="utf-8")
    print(md)


def videos():
    tk.video("f3", tk.COMMANDS["vx 0.2 + yaw 0.5"], "final_f3_walk.mp4", xray=True)
    tk.video("f3", tk.PUSH_CMDS["walking 0.3 m/s"], "final_f3_push.mp4", seconds=8.0, push=(3.0, 6.0, tk.PUSH_DIRS["left"]))


def check():
    assert tk.policy_run("f3", tk.COMMANDS["stand"], seconds=3.0) is None, "the robot with the F3 parts falls while it stands"
    r = tk.static_rows()
    assert abs(r["none"]["add"]) < 1e-9 and abs(r["f3"]["add"] - 4.14) < 0.01, (r["none"]["add"], r["f3"]["add"])
    assert r["none"]["stand"] < tk.s2s.EFFORT_LIMITS.min(), r["none"]["stand"]
    tables({"static": r, "walk": [{"model": v, "cmd": c, "limit": lim, "fall": None, "vx": 0.0, "wz": 0.0}
                                  for _, v in tk.ROWS for c in tk.COMMANDS for lim in LIMITS],
            "push": [{"model": v, "cmd": c, "dir": d, "t0": t, "impulse": 1.0}
                     for _, v in tk.ROWS for c in tk.PUSH_CMDS for d in tk.PUSH_DIRS for t in tk.PUSH_T]})  # the report builds
    print("final_check.py check: OK (F3 stands 3 s, statics, report tables)")


if __name__ == "__main__":
    {"video": videos, "check": check}.get(sys.argv[1] if len(sys.argv) > 1 else "", main)()
