"""Walking, push and statics helpers of the payload check. final_check.py uses them.

They run the ASMR robot of sim/sim2sim/run.py with the policy chosen there (--policy NAME in final_check.py, default
baseline) and a payload option (none, box_1kg, f3; see source/robot_models/robots/asmr2026_payload.py).
policy_run() is run.simulate() with a push: a horizontal force on the base for 0.1 s. max_push() finds the largest push
without a fall by bisection. video() films one run (needs ffmpeg).
"""
import math
import subprocess
import sys
from pathlib import Path

import mujoco

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent / "sim2sim")]
import analyze  # noqa: E402
import report  # noqa: E402  palette and plot style
import run as s2s  # noqa: E402  ASMR runner, policy and payload options

OUT = HERE / "results"
ROWS = [("bare robot", "none"), ("robot with a 1 kg box (training default)", "box_1kg"), ("robot with the F3 parts", "f3")]
# (table label, payload option)
COMMANDS = {"stand": (0.0, 0.0, 0.0), "vx 0.3": (0.3, 0.0, 0.0), "vx 0.2 + yaw 0.5": (0.2, 0.0, 0.5)}
PUSH_DIRS = {"forward": 0.0, "back": math.pi, "left": math.pi / 2, "right": -math.pi / 2}
PUSH_CMDS = {"standing": (0.0, 0.0, 0.0), "walking 0.3 m/s": (0.3, 0.0, 0.0)}
PUSH_T = (3.0, 3.2, 3.4)  # s, three push times = three gait phases; the table keeps the worst one
PUSH_HI, PUSH_ITERS = 24.0, 6  # N s upper bound, bisection steps (resolution 0.375 N s)


def policy_run(payload, cmd, seconds=10.0, push=None, frames=None, noise=True):
    """One run of the policy of run.py. push = (t0 s, impulse N s, direction rad in the heading frame, 0 = forward).
    Returns the fall time (s) or None."""
    return s2s.simulate(s2s.MODEL, cmd, None, seconds, frames=frames, noise=noise, push=push, payload=payload)["fall"]


def max_push(payload, cmd, angle, t0):
    """Largest push (N s) with no fall in the 3 s after it, by bisection. 0 when the robot falls without a push."""
    if policy_run(payload, cmd, seconds=t0 + 3.0) is not None:
        return 0.0
    lo, hi = 0.0, PUSH_HI
    for _ in range(PUSH_ITERS):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if policy_run(payload, cmd, seconds=t0 + 3.0, push=(t0, mid, angle)) is None else (lo, mid)
    return lo


def walk_job(payload, cmd_name, limit):
    """limit: torque cap in Nm on every joint (None = the effort limits of the MJCF)."""
    r = s2s.simulate(s2s.MODEL, COMMANDS[cmd_name], limit, 20.0, noise=True, payload=payload)
    return {"model": payload, "cmd": cmd_name, "limit": limit, "fall": r["fall"], "vx": r["vx"], "wz": r["wz"]}


def push_job(payload, cmd_name, dir_name, t0):
    return {"model": payload, "cmd": cmd_name, "dir": dir_name, "t0": t0,
            "impulse": max_push(payload, PUSH_CMDS[cmd_name], PUSH_DIRS[dir_name], t0)}


def static_rows():
    res = {}
    bare = analyze.static_report(s2s.load_model(payload="none"))["mass"]
    for label, v in ROWS:
        r = analyze.static_report(s2s.load_model(payload=v))
        pitch = max(max(abs(r[f"single_{s}"]["tau"][j]) for j in analyze.LEG if any(p in j for p in report.PITCH))
                    for s in ("left", "right"))
        res[v] = {"label": label, "mass": r["mass"], "add": r["mass"] - bare,
                  "com_dx": r["com_dx"], "com_dy": r["com_dy"], "com_h": r["com_h"], "stand": r["stand"]["peak"],
                  "crouch": r["crouch"]["peak"], "single_pitch": pitch, "single_all": r["single"]["peak"],
                  "single_ok": bool(r["single_left"]["cop_ok"] and r["single_right"]["cop_ok"])}
    return res


def push_worst(push, v, c, d):
    return min(p["impulse"] for p in push if p["model"] == v and p["cmd"] == c and p["dir"] == d)


def video(payload, cmd, out, seconds=10.0, push=None, dist=2.2, azim=135.0, elev=-15.0, xray=False, look_dz=0.0):
    """30 fps, 640 x 480, the camera follows the torso (as sim2sim/run.py video()). xray: the base and the payload parts
    are see-through."""
    w, h = 640, 480
    OUT.mkdir(exist_ok=True)
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}",
                           "-r", "30", "-i", "-", "-pix_fmt", "yuv420p", str(OUT / out)], stdin=subprocess.PIPE)
    st = {}

    def grab(m, d):
        if not st:
            st["r"] = mujoco.Renderer(m, h, w)
            cam = mujoco.MjvCamera()
            cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            cam.distance, cam.elevation, cam.azimuth = dist, elev, azim
            st["cam"], st["base"] = cam, m.body("base").id
            for g in range(m.ngeom) if xray else []:
                b = m.body(m.geom_bodyid[g]).name
                if b == "base" or b.startswith("payload_"):
                    m.geom_matid[g] = -1
                    m.geom_rgba[g] = [0.45, 0.45, 0.47, 0.18] if b == "base" else [0.85, 0.2, 0.1, 0.25]
        st["cam"].lookat[:] = d.subtree_com[st["base"]] + [0, 0, look_dz]  # whole-robot CoM, as the tracking camera
        st["r"].update_scene(d, st["cam"])
        ff.stdin.write(st["r"].render().tobytes())
        grab.n += 1
    grab.n = 0
    fall = policy_run(payload, cmd, seconds=seconds, push=push, frames=grab)
    ff.stdin.close()
    ff.wait()
    st["r"].close()
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-ss", "2", "-i", str(OUT / out), "-frames:v", "1",
                    str(OUT / out.replace(".mp4", "_poster.jpg"))], check=True)
    print(out, "fall" if fall else "no fall", fall)
