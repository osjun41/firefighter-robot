"""Joint identification: move one joint, log it, and find the rotor inertia, the friction and the torque scale that
make the simulation move the same way. The walking simulation needs these numbers (docs/plan/walking.md, cause C2).

    .venv/bin/python legs/sysid.py record left_knee_pitch           swing test. The robot hangs from the gantry.
    .venv/bin/python legs/sysid.py record left_knee_pitch --step    step test
    .venv/bin/python legs/sysid.py fit legs/logs/sysid_left_knee_pitch_swing.csv
    .venv/bin/python legs/sysid.py fit legs/logs/sysid_left_knee_pitch_step.csv --armature 0.012 --friction 0.2
    .venv/bin/python legs/sysid.py --check                          fake drives: record, then fit (about 1 min)

Both tests: the 6 joints of the leg move to a rest pose in 2 s and hold it with the gains of legs/robot.py. For a knee test, the thigh
leans 0.6 rad forward, so the hanging shin swings clear of the knee stop. The joint then moves 0.3 rad away from
its nearest limit. The program logs the joint angle at up to 200 Hz.
- Swing test: after 1 s, the drive of the joint turns off, and the link swings like a pendulum. The swing gives the
  rotor inertia (armature) and the joint friction (frictionloss). Use the knee (6512 motor) and the ankle pitch (5010).
- Step test: the joint target steps away and back, 1.5 s each. With the armature and the friction of the swing test,
  the step gives the torque scale: the real torque per amp over the value in legs/robot.py. It changes K, D and the
  torque limit together, as in the drive. The step test also checks the whole control loop (test stage T1).
fit: the same test in MuJoCo (the ASMR model of sim/sim2sim, torso held still) for a grid of values. It prints the best match and the
match with the training values, and draws a figure next to the log.
"""
import argparse
import csv
import json
import os
import time
import multiprocessing as mp
from pathlib import Path

import mujoco
import numpy as np

import odrive_can as oc
import robot as rb
from sim.sim2sim import run as sim2sim  # robot.py puts the repo root on sys.path

LOG_HZ = 200
SIZE, HOLD = 0.3, 1.5                               # rad, s
ARMATURE = np.geomspace(0.002, 0.05, 12)            # kg m^2 at the joint
FRICTION = np.array([0.0, 0.05, 0.1, 0.2, 0.3, 0.45, 0.6])   # N m
SCALE = np.array([0.85, 0.92, 1.0, 1.08, 1.15])
_M = sim2sim.load_model()
_KNEE, _ANKLE = rb.NAMES.index("left_knee_pitch"), rb.NAMES.index("left_ankle_pitch")
# armature and friction of the ASMR model, per motor: the values of the training and of the fake drives
TRAINED = {"6512": (float(_M.dof_armature[6 + _KNEE]), float(_M.dof_frictionloss[6 + _KNEE])),
           "5010": (float(_M.dof_armature[6 + _ANKLE]), float(_M.dof_frictionloss[6 + _ANKLE]))}
SIGN = rb.BHL_TO_ASMR                               # joint angle in the ASMR model = SIGN * angle of legs/robot.py


def leg(i):
    return list(range(6)) if i < 6 else list(range(6, 12))


def rest_pose(i):
    """Pose of the leg during the test. For a knee test the thigh leans forward, so the shin hangs at knee 0.6 rad."""
    q = rb.ZERO_POSE.copy()
    if rb.NAMES[i].endswith("knee_pitch"):
        q[i - 1], q[i] = -0.6, 0.6                  # hip pitch, knee pitch (the same signs on both legs)
    return q


def record(joint, out, fake=False, mode="swing", size=SIZE):
    """Run the test on one joint and write the log. Returns the path of the log."""
    i = rb.NAMES.index(joint)
    if fake:
        from fake_bus import FakeRobot
        sim = FakeRobot(hoist_z=0.3)                         # high on the hoist: the swinging foot stays off the floor
        buses = sim.buses()
    else:
        buses, sim = rb.open_buses(), None
    robot = rb.Robot(buses, calib=None if sim else rb.CALIB)
    rows, pose, release = [], None, None
    try:
        if sim:
            robot.calibrate(robot.turns(), rb.ZERO_POSE)    # the fake starts exactly in the zero pose
        robot.zero()
        idx = leg(i)
        if missing := [rb.NAMES[k] for k in idx if k not in robot.idx]:
            raise SystemExit("No reply from " + ", ".join(missing))
        robot.enable(idx)
        rest = rest_pose(i)                                  # the hold targets of the test
        if f := robot.move_to(rest, 2.0):
            raise SystemExit("Stop: " + "; ".join(f))
        time.sleep(0.5)                                      # the leg settles under the hold
        pose, _ = robot.read()
        away = 1.0 if robot.hi[i] - pose[i] >= pose[i] - robot.lo[i] else -1.0
        clock = (lambda: sim.truth()[2]) if sim else time.monotonic   # the fake physics can lag the wall clock
        target, b, node, t0 = rest.copy(), robot.bus[i], robot.node[i], clock()
        held, end = idx, (HOLD + 3.0 if mode == "swing" else 3 * HOLD)
        for k in rb.ticks(LOG_HZ):
            t = k / LOG_HZ
            if mode == "step":
                target[i] = rest[i] + (away * size if HOLD <= t < 2 * HOLD else 0.0)
            else:
                target[i] = rest[i] + away * size * min(1.0, t / 0.5)   # 0.5 s ramp, then hold until the release
                if release is None and t >= HOLD:
                    robot._send([i], lambda n, _: [oc.set_axis_state(n, oc.IDLE)])
                    robot.enabled[i], held, release = False, [x for x in idx if x != i], clock() - t0
            robot.command(target, held if k % 10 == 0 else [x for x in held if x == i])   # watchdogs: 20 times a second
            for g, _ in robot.groups:
                g.poll()                                     # heartbeats of both buses, for robot.faults()
            got = b.ask([node], oc.GET_ENCODER_ESTIMATES, timeout=0.004)
            if node in got:
                p, v = got[node]
                rows.append((clock() - t0, np.nan if release is not None else target[i],
                             robot.sign[i] * (p - robot.offset[i]) / rb.TURNS_PER_RAD, robot.sign[i] * v / rb.TURNS_PER_RAD))
            if f := robot.faults():
                print("Stop:", "; ".join(f))
                break
            if t >= end:
                break
    finally:
        robot.relax(1.0)
        for b in buses:
            b.close()
        if sim:
            sim.close()
    if pose is None:
        raise SystemExit("No log: the test did not start")
    meta = {"joint": joint, "motor": rb.JOINTS[i][2], "mode": mode, "K": np.asarray(rb.KP).tolist(), "D": np.asarray(rb.KD).tolist(),
            "torque_limit": rb.TORQUE_LIMIT, "targets": [float(v) for v in rest], "pose": [round(float(v), 5) for v in pose],
            "release_t": None if release is None else round(release, 5),   # as the logged times: a NaN row is never before the release
            "rate_hz": LOG_HZ, "fake": fake}
    with open(out, "w", newline="") as f:
        f.write("# " + json.dumps(meta) + "\n")
        w = csv.writer(f)
        w.writerow(["t", "target", "q", "dq"])
        w.writerows([[f"{v:.5f}" for v in r] for r in rows])
    print(f"wrote {out}: {len(rows)} samples in {rows[-1][0]:.2f} s" if rows else f"wrote {out}: no samples")
    return out


def load(path):
    with open(path) as f:
        meta = json.loads(f.readline()[1:])
        data = np.array([[float(v) for v in r.values()] for r in csv.DictReader(f)])
    return meta, data


def simulate(meta, data, armature, friction, scale):
    """Joint angle of the simulated test at the logged times (NaN before the release of a swing test).
    The torso is held still. The run starts from the logged pose, and the other 5 joints of the leg hold the test
    targets with the drive PD, so they sag as on the robot. At the release of a swing test, the tested joint takes
    its logged angle and speed, and its drive turns off."""
    spec = mujoco.MjSpec.from_file(str(sim2sim.MODEL_PATH))
    sim2sim.add_payload(spec)                                # the same model as the fake drives (fake_bus.py)
    hold = spec.add_equality()
    hold.type, hold.objtype, hold.name1 = mujoco.mjtEq.mjEQ_WELD, mujoco.mjtObj.mjOBJ_BODY, "base"
    hold.data = [0, 0, 0, 0, 0, -0.5, 1, 0, 0, 0, 1]        # torso 0.5 m up: the feet hang free
    m = spec.compile()
    m.opt.timestep = sim2sim.PHYSICS_DT
    sim2sim.configure_torque_actuators(m, 1e3)              # ctrl = joint torque, clipped below
    i = rb.NAMES.index(meta["joint"])
    dofs = [6 + k for k in leg(i)]
    m.dof_armature[dofs], m.dof_frictionloss[dofs] = armature, friction
    d = mujoco.MjData(m)
    d.qpos[2], d.qpos[7:19] = 0.5, SIGN * np.array(meta["pose"])
    mujoco.mj_forward(m, d)
    t_log, target = data[:, 0], data[:, 1]
    goal, out, n = np.array(meta["targets"]), np.full(len(t_log), np.nan), 0
    k_rel = int(np.searchsorted(t_log, meta["release_t"])) if meta["mode"] == "swing" else None
    released = False
    K, D, lim = scale * np.array(meta["K"]), scale * np.array(meta["D"]), scale * meta["torque_limit"]
    while n < len(t_log):
        if k_rel is not None and not released and d.time >= t_log[k_rel]:
            d.qpos[7 + i], d.qvel[6 + i], released = SIGN[i] * data[k_rel, 2], SIGN[i] * data[k_rel, 3], True
        if not released:
            goal[i] = target[max(0, np.searchsorted(t_log, d.time, side="right") - 1)]
            assert np.isfinite(goal[i]), "the log has no target before the release"
        d.ctrl[:] = SIGN * np.clip(K * (goal - SIGN * d.qpos[7:19]) - D * SIGN * d.qvel[6:18], -lim, lim)
        if released:
            d.ctrl[i] = 0.0                                  # drive off: the link swings freely
        mujoco.mj_step(m, d)
        while n < len(t_log) and d.time >= t_log[n]:
            out[n], n = (SIGN[i] * d.qpos[7 + i] if k_rel is None or n >= k_rel else np.nan), n + 1
    return out


def _rms(args):
    meta, data, a, f, s = args
    return a, f, s, float(np.sqrt(np.nanmean((simulate(meta, data, a, f, s) - data[:, 2]) ** 2)))


def fit(path, armature=ARMATURE, friction=FRICTION, scale=SCALE, plot=True):
    """Grid search. Returns the best (armature, friction, scale, rms) and the same for the training values."""
    meta, data = load(path)
    if meta["mode"] == "swing":
        scale = np.array([1.0])                              # no motor torque after the release
    grid = [(meta, data, a, f, s) for a in armature for f in friction for s in scale]
    with mp.get_context("spawn").Pool(min(6, os.cpu_count() or 1)) as p:   # no fork: the fake drives use threads
        res = p.map(_rms, grid)
    best = min(res, key=lambda r: r[3])
    nominal = _rms((meta, data, *TRAINED[meta["motor"]], 1.0))
    print(f"{meta['joint']} ({meta['mode']} test): best armature {best[0]:.4f} kg m^2, friction {best[1]:.2f} N m, "
          f"torque scale {best[2]:.2f}, rms {np.degrees(best[3]):.2f} deg")
    print(f"  training values (armature {nominal[0]:.4f}, friction {nominal[1]:.3f}, scale 1.0): rms {np.degrees(nominal[3]):.2f} deg")
    edge = [n for n, v, g in (("armature", best[0], armature), ("friction", best[1], friction), ("scale", best[2], scale))
            if len(g) > 1 and v in (g[0], g[-1])]
    if edge:
        print("  The best value is at the edge of the grid for: " + ", ".join(edge) + ". Check the log.")
    if plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 4))
        t = data[:, 0]
        ax.plot(t, np.degrees(data[:, 1]), color="#8a8a8a", lw=1, label="target")
        ax.plot(t, np.degrees(data[:, 2]), color="#1d1d1f", lw=1.8, label="robot")
        ax.plot(t, np.degrees(simulate(meta, data, *best[:3])), color="#2f6690", lw=1.4, ls="--",
                label=f"simulation, best: armature {best[0]:.4f}, friction {best[1]:.2f}, scale {best[2]:.2f}")
        ax.plot(t, np.degrees(simulate(meta, data, *nominal[:3])), color="#e8711a", lw=1.2, ls=":",
                label="simulation, training values")
        ax.set_xlabel("time (s)")
        ax.set_ylabel(f"{meta['joint']} (deg)")
        ax.legend(frameon=False, fontsize=8)
        fig.tight_layout()
        png = str(Path(path).with_suffix(".png"))
        fig.savefig(png, dpi=150)
        print(f"  wrote {png}")
    return best, nominal


def self_check():
    """Record on the fake drives, which use the ASMR model values (TRAINED, torque scale 1). The fit must find them again."""
    import tempfile
    tmp = tempfile.mkdtemp()
    for mode in ("swing", "step"):
        out = record("left_knee_pitch", os.path.join(tmp, f"sysid_{mode}.csv"), fake=True, mode=mode)
        meta, data = load(out)
        assert len(data) > 0.4 * (4.5 if mode == "swing" else 3 * HOLD) * LOG_HZ, f"{len(data)} samples: the loop is too slow"
        a0, f0 = TRAINED[meta["motor"]]
        best, _ = fit(out, armature=np.array([a0 / 2.5, a0, a0 * 2.5]), friction=np.array([0.0, f0, 2 * f0]),
                      scale=np.array([0.85, 1.0, 1.15]), plot=False)
        assert best[0] == a0 and best[1] == f0 and best[2] == 1.0, (mode, best)
    print("self-check sysid: passed")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", nargs="?", choices=["record", "fit"])
    ap.add_argument("target", nargs="?", help="joint name (record) or log file (fit)")
    ap.add_argument("--step", action="store_true", help="record a step test instead of a swing test")
    ap.add_argument("--out", help="log file of record, default legs/logs/sysid_<joint>_<test>.csv")
    ap.add_argument("--size", type=float, default=SIZE, help="joint move in rad (default 0.3)")
    ap.add_argument("--armature", type=float, help="fit: use this armature (from a swing test)")
    ap.add_argument("--friction", type=float, help="fit: use this friction (from a swing test)")
    ap.add_argument("--fake", action="store_true", help="fake drives on a MuJoCo biped")
    ap.add_argument("--yes", action="store_true", help="do not ask before the joint moves")
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    if a.check:
        return self_check()
    if a.action == "record":
        if a.target not in rb.NAMES:
            raise SystemExit("joint names: " + ", ".join(rb.NAMES))
        mode = "step" if a.step else "swing"
        print(f"CAUTION: the 6 joints of this leg move to the test pose in 2 s and hold it. Then {a.target} moves "
              f"{np.degrees(a.size):.0f} deg" + (" and swings freely." if mode == "swing" else " and back.") +
              " The robot must hang from the gantry. Keep hands off the leg.")
        if not (a.yes or a.fake) and input("Type yes to move: ").strip() != "yes":
            return
        out = a.out or str(Path(__file__).with_name("logs") / f"sysid_{a.target}_{mode}.csv")
        Path(out).parent.mkdir(exist_ok=True)
        record(a.target, out, a.fake, mode, a.size)
    elif a.action == "fit":
        fit(a.target, armature=ARMATURE if a.armature is None else np.array([a.armature]),
            friction=FRICTION if a.friction is None else np.array([a.friction]))
    else:
        ap.print_help()


if __name__ == "__main__":
    main()
