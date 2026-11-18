"""Sim-to-real risks of the course motor drives, for a walking policy of policies/ (default baseline) on the ASMR robot with the F3 parts.

The robot uses XDrive MINI drives (ODrive v3.6 clone, firmware v0.5.1). The drive does the PD at 8 kHz, in motor turns:
vel_sp = pos_gain (pos_set - pos) + vel_ff, torque = vel_gain (vel_sp - vel). With the gear ratio N = 15 this gives
joint K = N^2/(2 pi) pos_gain vel_gain and joint D = N^2/(2 pi) vel_gain. This script runs the run.py policy loop and
adds: per-joint K, D and torque limit, a joint zero error, command and observation latency, and the TRAP_TRAJ input mode.

    uv run python sim/sim2sim/hw_risks.py            all cases -> results/hw_risks.md, hw_risks.json, hw_risks*.png
    uv run python sim/sim2sim/hw_risks.py --report   tables and plots again from results/hw_risks.json
    uv run python sim/sim2sim/hw_risks.py --regen    regen analysis only, added to results/hw_risks.json, about 1 min
    uv run python sim/sim2sim/hw_risks.py --check    self-check, about 20 s
    MUJOCO_GL=egl uv run python sim/sim2sim/hw_risks.py --video   the two videos (needs ffmpeg and hw_risks.json)

--policy NAME (a folder of policies/, default baseline) and --payload NAME (none, box_1kg, f3; default f3) go before the
other options. The robot, its gains and limits, the standing pose and the IMU mount come from the ASMR model
(sim/sim2sim/run.py, source/robot_models), so "as trained" means the gains and torque limits of its MJCF.
"""
import json
import math
import os
import sys
import time
from collections import deque
from functools import lru_cache
from multiprocessing import Pool
from pathlib import Path

import mujoco
import numpy as np
import onnxruntime as ort

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent / "payload")]
for _option, _variable in (("--policy", "SIM2SIM_POLICY"), ("--payload", "SIM2SIM_PAYLOAD")):  # before run.py reads them
    if _option in sys.argv:
        os.environ[_variable] = sys.argv[sys.argv.index(_option) + 1]
        del sys.argv[sys.argv.index(_option):sys.argv.index(_option) + 2]
import run as s2s  # noqa: E402  the ASMR runner: policy, contract, model, observation
import report  # noqa: E402  sim/payload/report.py: palette and plot style

plt = report.plt
OUT = s2s.OUT
MODEL = os.environ.get("SIM2SIM_PAYLOAD") or "f3"  # payload on the torso (source/robot_models/robots/asmr2026_payload.py)
N = 15  # actuator gear ratio of the course leg drives
# Leg joint order of run.py JOINTS: hip roll, hip yaw, hip pitch, knee (6512), ankle pitch, ankle roll (5010).
# Source: the course drive config, torque constants 0.067 (6512) and 0.0836 (5010) Nm/A (BHL file: 0.0919 and 0.1176).
IS_6512 = np.tile([True] * 4 + [False] * 2, 2)


def joint_gains(pos_gain, vel_gain):
    """ODrive pos_gain ((turn/s)/turn) and vel_gain (Nm/(turn/s)), motor side -> joint K (Nm/rad), D (Nm s/rad)."""
    return N * N / (2 * math.pi) * pos_gain * vel_gain, N * N / (2 * math.pi) * vel_gain


# Course config: omletkang/xdrive-mini-fw-instant-start xdrive_config.txt (read 2026-10-15). 6512 / 5010 values.
COURSE_K = np.where(IS_6512, joint_gains(12.6, 0.223)[0], joint_gains(20.0, 0.1)[0])
COURSE_D = np.where(IS_6512, joint_gains(12.6, 0.223)[1], joint_gains(20.0, 0.1)[1])
COURSE_LIM = 18.0 * N * np.where(IS_6512, 0.067, 0.0836)  # current_lim 18 A x torque_constant x N
TRAP_V, TRAP_A = 10 * 2 * math.pi / N, 5 * 2 * math.pi / N  # trap_traj vel_limit 10 turn/s, accel 5 turn/s^2 -> joint
OVERSPEED = 1.2 * 50 * 2 * math.pi / N  # rad/s at the joint: vel_limit 50 turn/s x vel_limit_tolerance 1.2 (default)
REGEN_W, BUS_W = 3.0 * 24.0, 12.0 * 24.0  # W: dc_max_negative_current 3 A, dc_max_positive_current 12 A, at 24 V

BASE = dict(K=s2s.KP, D=s2s.KD, lim=s2s.EFFORT_LIMITS, zero_deg=0.0, cmd_ms=0, obs_ms=0, trap=False)  # the MJCF of the ASMR robot
COURSE = dict(K=COURSE_K, D=COURSE_D)
CASES = {  # id: (label, settings). The first character of the id is the case group (GROUPS).
    "1": ("baseline: MJCF gains K 90 / 22 Nm/rad, D 5.7 / 0.49, 18 / 23 Nm (as trained)", BASE),
    "2": ("course gains, 18 A, PASSTHROUGH (course file run to the end)", {**BASE, **COURSE, "lim": COURSE_LIM}),
    "2b": ("course gains, 18 A, TRAP_TRAJ (course file section 4)", {**BASE, **COURSE, "lim": COURSE_LIM, "trap": True}),
    "3": ("course gains, 67 % of the 18 A current limit", {**BASE, **COURSE, "lim": 0.67 * COURSE_LIM}),
    "4a": ("zero error ±1°", {**BASE, "zero_deg": 1.0}),
    "4b": ("zero error ±2°", {**BASE, "zero_deg": 2.0}),
    "4c": ("zero error ±4°", {**BASE, "zero_deg": 4.0}),
    "5a": ("command latency 10 ms", {**BASE, "cmd_ms": 10}),
    "5b": ("command latency 20 ms", {**BASE, "cmd_ms": 20}),
    "5c": ("command latency 40 ms", {**BASE, "cmd_ms": 40}),
    "5d": ("command latency 60 ms (edge test)", {**BASE, "cmd_ms": 60}),
    "5e": ("observation latency 10 ms", {**BASE, "obs_ms": 10}),
    "6a": ("torque limit 67 % (12 / 15 Nm)", {**BASE, "lim": 0.67 * s2s.EFFORT_LIMITS}),
    "6b": ("torque limit 50 % (9 / 11 Nm)", {**BASE, "lim": 0.5 * s2s.EFFORT_LIMITS}),
    "7": ("realistic: MJCF gains and limits, ±2°, 10 ms command latency", {**BASE, "zero_deg": 2.0, "cmd_ms": 10}),
    "7b": ("realistic + 10 ms observation latency", {**BASE, "zero_deg": 2.0, "cmd_ms": 10, "obs_ms": 10}),
    "7c": ("course file (case 2) + ±2°, 10 ms command latency",
           {**BASE, **COURSE, "lim": COURSE_LIM, "zero_deg": 2.0, "cmd_ms": 10}),
    "7d": ("7b with a 67 % torque limit", {**BASE, "lim": 0.67 * s2s.EFFORT_LIMITS, "zero_deg": 2.0, "cmd_ms": 10, "obs_ms": 10}),
}
SEEDS, PUSH_SEEDS = 10, 5  # noise sequence and zero errors per seed; the push time also changes with the seed
PUSH_DIRS = {"back": math.pi, "left": math.pi / 2}  # walking at 0.3 m/s. Sideways pushes are the weakest (payload push test)
PUSH_HI, PUSH_ITERS = 12.0, 5  # N s, bisection resolution 0.375 N s
VIDEO_WORST, VIDEO_REC = "7c", "7b"
COURSE_IDS = ("2", "2b", "3", "7c")


def load_model(name):
    """The ASMR robot with the payload `name` at the run.py physics rate. Its position servos become torque motors
    (the drive model below does the PD and the clipping)."""
    m = s2s.load_model(payload=name)
    s2s.configure_torque_actuators(m, 30.0)  # the model stops at 30 Nm; the drive limit below does the clipping
    return m


def session(path):
    so = ort.SessionOptions()
    so.intra_op_num_threads = 1  # one thread per worker process
    return ort.InferenceSession(str(path), so, providers=["CPUExecutionProvider"])


@lru_cache(maxsize=1)
def load():
    """Model and policy, once per worker process."""
    return load_model(MODEL), session(s2s.POLICY)


def trap_step(p, v, goal, dt):
    """One step of a trapezoidal profile toward goal: brake so that the setpoint stops at goal, |v| <= TRAP_V.
    Limit: replans every step from (p, v). The ODrive replans at each new input_pos; the shapes are close."""
    e = goal - p
    v_des = np.sign(e) * np.minimum(np.sqrt(2 * TRAP_A * np.abs(e)), TRAP_V)
    v = v + np.clip(v_des - v, -TRAP_A * dt, TRAP_A * dt)
    return p + v * dt, v


NOZZLE = np.array([-0.0201, 0.0, 0.9323])  # m, base frame: the brass nozzle of the F3 layout (0.8925 + 0.0398), where the hose pulls
HOSE_PERIOD = 4.0  # s, period of the slowly varying hose pull


def simulate(cfg, command, seed, seconds=20.0, push=None, frames=None, log=None, env=None):
    """One run of the policy (default baseline) on the ASMR robot with the payload MODEL and the drive model cfg, with the training observation noise.
    push = (t0 s, impulse N s, direction rad in the heading frame): a horizontal force on the torso for 0.1 s.
    Returns the fall time and the metrics. The torque metrics start at 1 s.
    log: optional list that gets [12 joint torques, 12 joint speeds] per physics step from 1 s.
    env: optional (MjModel, ONNX session) instead of load(), for other models and policies (gap_study.py).
    Optional cfg keys of gap_study.py, off when missing: imu_deg (IMU mounting tilt about a horizontal axis that the
    seed sets), backlash_deg (gear play at the joint), hose_n (backward pull at NOZZLE, N), hose_var (the pull goes
    from 0 to hose_n and back in HOSE_PERIOD s)."""
    m, policy = env or load()
    d = mujoco.MjData(m)
    d.qpos[:3] = s2s.DEFAULT_BASE_POSITION  # the standing pose, as run.py
    d.qpos[7:19] = s2s.DEFAULT
    mujoco.mj_forward(m, d)
    imu_in_base, base_in_imu = s2s._imu_frame_transform(m, d)  # the IMU of the ASMR robot is turned about z
    rng = np.random.default_rng(seed)
    off = np.deg2rad(np.random.default_rng([seed, 1]).uniform(-1, 1, 12) * cfg["zero_deg"])  # measured - true
    K, D, lim = (np.broadcast_to(np.asarray(cfg[k], dtype=float), 12) for k in ("K", "D", "lim"))
    dt, sub = s2s.PHYSICS_DT, round(s2s.POLICY_DT / s2s.PHYSICS_DT)
    cmd_steps, obs_steps = round(cfg["cmd_ms"] / 1000 / dt), round(cfg["obs_ms"] / 1000 / dt)
    base, z0 = m.body("base").id, d.site("imu").xpos[2]
    c = np.array(command, dtype=np.float32)
    mount, az = math.radians(cfg.get("imu_deg", 0.0)), np.random.default_rng([seed, 2]).uniform(0, 2 * math.pi)
    q_imu = np.array([math.cos(mount / 2), math.sin(mount / 2) * math.cos(az), math.sin(mount / 2) * math.sin(az), 0.0])
    h = math.radians(cfg.get("backlash_deg", 0.0)) / 2  # half of the gear play, at the joint
    qr, vr = d.qpos[7:19].copy(), np.zeros(12)  # with play: rotor position and speed, joint units (the encoder)
    hose, phase = cfg.get("hose_n", 0.0), np.random.default_rng([seed, 3]).uniform(0, 2 * math.pi)
    assert not (h and cfg["trap"]), "the gear play model needs PASSTHROUGH"

    def reading():
        quat, gyro = d.sensor("imu_quat").data.copy(), d.sensor("imu_gyro").data.copy()
        if mount:  # the IMU frame is turned by q_imu (mounting error)
            mujoco.mju_mulQuat(quat, d.sensor("imu_quat").data, q_imu)
            gyro = s2s.quat_rotate_inverse(q_imu, gyro)
        quat, gyro = s2s.quat_mul(quat, base_in_imu), imu_in_base @ gyro  # to the base frame, as run.py
        return (quat, gyro) + ((qr + off, vr.copy()) if h else (d.qpos[7:19] + off, d.qvel[6:18].copy()))
    hist = deque([reading()], maxlen=obs_steps + 1)  # hist[0] is the reading of obs_steps physics steps ago
    queue = deque()  # (physics step when the drive gets it, joint target)
    goal = s2s.DEFAULT.astype(float)  # the drive holds the stand pose until the first command arrives
    sp_p, sp_v = s2s.DEFAULT + off, np.zeros(12)  # TRAP_TRAJ setpoint, measured coordinates
    action = np.zeros(12, dtype=np.float32)
    n, fall, vx, wz = 0, None, [], []
    tau_sum, tau_n, peak, sat, qd_peak, p_min, p_max, rate_sq, rate_n = 0.0, 0, 0.0, 0, 0.0, 0.0, 0.0, 0.0, 0
    sat_j, peak_j = np.zeros(12), np.zeros(12)  # per joint: steps at the torque limit, peak torque
    for _ in range(round(seconds / s2s.POLICY_DT)):
        obs = s2s.observation(c, *hist[0], action)
        obs += (rng.uniform(-1, 1, 45) * s2s.NOISE).astype(np.float32)
        prev, action = action, policy.run(None, {"obs": obs})[0][0]
        if d.time >= 1.0:
            rate_sq, rate_n = rate_sq + float(np.sum((action - prev) ** 2)), rate_n + 12
        queue.append((n + cmd_steps, np.clip(s2s.DEFAULT + s2s.ACTION_SCALE * action, s2s.JOINT_LIMITS[:, 0], s2s.JOINT_LIMITS[:, 1])))
        for _ in range(sub):
            while queue and queue[0][0] <= n:
                goal = queue.popleft()[1]
            qm, qd = d.qpos[7:19] + off, d.qvel[6:18]
            if cfg["trap"]:
                sp_p, sp_v = trap_step(sp_p, sp_v, goal, dt)
                tau = K * (sp_p - qm) + D * (sp_v - qd)
            elif h:  # gear play: the drive turns the rotor, and the rotor pushes the joint only at a wall of the play
                gap = qr - d.qpos[7:19]
                tau = K * (goal - qr - off) - D * qd  # drive torque when the rotor turns with the joint
                touch = ((gap >= h - 1e-9) & (tau > 0)) | ((gap <= 1e-9 - h) & (tau < 0))
                tau = np.where(touch, tau, 0.0)
            else:
                tau = K * (goal - qm) - D * qd
            d.ctrl[:] = np.clip(tau, -lim, lim)
            if d.time >= 1.0:
                a = np.abs(d.ctrl)
                tau_sum, tau_n, peak = tau_sum + a.sum(), tau_n + 12, max(peak, float(a.max()))
                clipped = np.abs(tau) > lim
                sat += bool(np.any(clipped))
                sat_j, peak_j = sat_j + clipped, np.maximum(peak_j, a)
                pw = d.ctrl * qd  # mechanical power at each joint, W
                qd_peak, p_min, p_max = max(qd_peak, float(np.abs(qd).max())), min(p_min, pw.min()), max(p_max, pw.max())
                if log is not None:
                    log.append(np.r_[d.ctrl, qd])
            d.xfrc_applied[base, :3] = 0
            if push and push[0] <= d.time < push[0] + 0.1:
                R = d.xmat[base].reshape(3, 3)
                ang = math.atan2(R[1, 0], R[0, 0]) + push[2]
                d.xfrc_applied[base, :2] = push[1] / 0.1 * np.array([math.cos(ang), math.sin(ang)])
            if hose:  # horizontal, backward in the heading frame, at the nozzle point
                R = d.xmat[base].reshape(3, 3)
                ang = math.atan2(R[1, 0], R[0, 0])
                f = hose * ((1 - math.cos(2 * math.pi * d.time / HOSE_PERIOD + phase)) / 2 if cfg.get("hose_var") else 1)
                F = -f * np.array([math.cos(ang), math.sin(ang), 0.0])
                d.xfrc_applied[base, :3] += F
                d.xfrc_applied[base, 3:] = np.cross(d.xpos[base] + R @ NOZZLE - d.xipos[base], F)  # about the CoM
            mujoco.mj_step(m, d)
            if h:  # a rotor at a wall turns with the joint. A free rotor turns at the speed where the drive torque is 0
                vr = np.where(touch, d.qvel[6:18], K / D * (goal - qr - off))
                qr = np.clip(np.where(touch, d.qpos[7:19] + gap, qr + vr * dt), d.qpos[7:19] - h, d.qpos[7:19] + h)
            n += 1
            hist.append(reading())
            if frames and d.time * 30 >= frames.n:
                frames(m, d)
        t = d.time
        base_quat = s2s.quat_mul(d.sensor("imu_quat").data, base_in_imu)
        tilt = np.arccos(np.clip(-s2s.quat_rotate_inverse(base_quat, np.array([0, 0, -1.0]))[2], -1, 1))
        if fall is None and (tilt > s2s.TILT_LIMIT or d.site("imu").xpos[2] < 0.6 * z0):
            fall = t
        if fall is not None and (not frames or t > fall + 2.0):  # a video shows 2 s after the fall
            break
        if t >= 5.0 and fall is None:  # as run.py: speeds from 5 s, in the heading frame
            R = d.xmat[base].reshape(3, 3)
            yaw = np.arctan2(R[1, 0], R[0, 0])
            v = d.sensor("frame_vel").data
            vx.append(np.cos(yaw) * v[0] + np.sin(yaw) * v[1])
            wz.append((R @ d.qvel[3:6])[2])
    ok = vx and fall is None
    return dict(model=MODEL, command=list(command), seed=seed, limit=float(lim.max()), fall=fall,
                vx=float(np.mean(vx)) if ok else None, wz=float(np.mean(wz)) if ok else None,
                tau_mean=tau_sum / max(tau_n, 1), peak=peak, sat=100.0 * sat / max(tau_n // 12, 1),
                rate=s2s.ACTION_SCALE * math.sqrt(rate_sq / max(rate_n, 1)), qd_peak=qd_peak,
                p_regen=float(-p_min), p_drive=float(p_max),
                sat_j=(100.0 * sat_j / max(tau_n // 12, 1)).tolist(), peak_j=peak_j.tolist())


def walk_job(cid, cmd_name, seed):
    return {"case": cid, "cmd": cmd_name, **simulate(CASES[cid][1], s2s.COMMANDS[cmd_name], seed)}


def push_job(cid, dir_name, seed):
    """Largest push (N s) with no fall in the 3 s after it, by bisection, walking at 0.3 m/s. 0 if it falls anyway."""
    cfg, cmd, t0 = CASES[cid][1], s2s.COMMANDS["vx 0.3"], 3.0 + 0.1 * seed  # 5 seeds span one step (0.5 s)
    alive = lambda j: simulate(cfg, cmd, seed, t0 + 3.0, push=(t0, j, PUSH_DIRS[dir_name]))["fall"] is None
    lo, hi = 0.0, PUSH_HI
    if not alive(0.0):
        return {"case": cid, "dir": dir_name, "seed": seed, "impulse": 0.0}
    for _ in range(PUSH_ITERS):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if alive(mid) else (lo, mid)
    return {"case": cid, "dir": dir_name, "seed": seed, "impulse": lo}


# Regen with the bus diode: the course BOM has a Schottky diode (MBRT400100) in the battery line, so regen current
# cannot go back into the batteries. All 12 drives share one bus. Their capacitors take the regen energy.
R_PHASE = 0.1  # ohm, GUESS for both motors: no datasheet value found for the MAD M6C12 150KV or the MAD 5010 110KV
KT = np.where(IS_6512, 0.067, 0.0836)  # Nm/A, course config torque_constant
C_BUS = 12 * 5 * 100e-6  # F: 5 x 100 uF per XDrive MINI (from photos), 12 drives
V_FULL, V_TRIP = 25.2, 26.0  # V: full 6S battery (diode drop ignored), course dc_bus_overvoltage_trip_level
REGEN_CASES = ("1", "7")


def bus_metrics(tau, qd, dt):
    """tau, qd: (steps, 12) joint torque and speed. Drive input power = mechanical + copper loss (1.5 R I^2, I = Iq)."""
    p = tau * qd + 1.5 * R_PHASE * (tau / N / KT) ** 2
    bus = p.sum(axis=1)
    edges = np.flatnonzero(np.diff(np.r_[0, (bus < 0).astype(int), 0]))  # start, end, start, end ... of negative stretches
    e = e_max = 0.0
    for b in bus:  # energy above the battery level in the capacitors: the diode blocks the way back
        e = max(0.0, e - b * dt)
        e_max = max(e_max, e)
    return dict(p_drive=float(p.min()), p_bus=float(bus.min()), e=e_max,
                t_neg=float((edges[1::2] - edges[::2]).max(initial=0) * dt))


def regen_job(cid, cmd_name, seed):
    log = []
    r = simulate(CASES[cid][1], s2s.COMMANDS[cmd_name], seed, log=log)
    a = np.array(log)
    return {"case": cid, "cmd": cmd_name, "seed": seed, "fall": r["fall"], **bus_metrics(a[:, :12], a[:, 12:], s2s.PHYSICS_DT)}


def add_regen(res):
    with Pool(os.cpu_count()) as pool:
        res["regen"] = pool.starmap(regen_job, [(c, k, s) for c in REGEN_CASES for k in s2s.COMMANDS for s in range(SEEDS)],
                                    chunksize=1)


def regen_table(res):
    L = ["| Case | Lowest drive input power (W) | Drive current at 24 V (A) | Runs with a drive below −3 A | Lowest bus power, "
         "12 drives (W) | Longest negative bus power (ms) | Largest energy into the capacitors (J) | Peak bus voltage from "
         f"{V_FULL} V (V) |", "|---|---|---|---|---|---|---|---|"]
    for cid in REGEN_CASES:
        rs = [r for r in res["regen"] if r["case"] == cid]
        pd, e = min(r["p_drive"] for r in rs), max(r["e"] for r in rs)
        L.append(f"| {cid} | {pd:.0f} | {pd / 24:.1f} | {sum(r['p_drive'] < -REGEN_W for r in rs)} of {len(rs)} "
                 f"| {min(r['p_bus'] for r in rs):.0f} | {1000 * max(r['t_neg'] for r in rs):.0f} | {e:.2f} "
                 f"| {math.sqrt(V_FULL ** 2 + 2 * e / C_BUS):.2f} |")
    return "\n".join(L) + "\n"


def summary(res, cid):
    w = [r for r in res["walk"] if r["case"] == cid]
    falls = [r["fall"] for r in w if r["fall"] is not None]
    mean = lambda rs, k: float(np.mean([r[k] for r in rs if r[k] is not None])) if any(r[k] is not None for r in rs) else None
    by = lambda c: [r for r in w if r["cmd"] == c]
    push = {k: [p["impulse"] for p in res["push"] if p["case"] == cid and p["dir"] == k] for k in PUSH_DIRS}
    return dict(runs=len(w), falls=len(falls), t_fall=float(np.mean(falls)) if falls else None,
                vx03=mean(by("vx 0.3"), "vx"), vx02=mean(by("vx 0.2 + yaw 0.5"), "vx"), wz02=mean(by("vx 0.2 + yaw 0.5"), "wz"),
                tau_mean=mean(w, "tau_mean"), peak=max(r["peak"] for r in w), sat=mean(w, "sat"), rate=mean(w, "rate"),
                qd_peak=max(r["qd_peak"] for r in w), p_regen=max(r["p_regen"] for r in w), p_drive=max(r["p_drive"] for r in w),
                push={k: (min(v), float(np.mean(v))) for k, v in push.items()})


HEAD = ("| Case | Falls | Mean time to fall (s) | vx at cmd 0.3 (m/s) | vx, yaw at cmd 0.2 + 0.5 | Mean abs torque (Nm) "
        "| Peak torque (Nm) | Clipped steps (%) | Action rate (rad) | Push back, worst / mean (N s) | Push left, worst / mean (N s) |")


def table_row(cid, s):
    f = lambda x, p=2: "-" if x is None else f"{x:.{p}f}"
    pb, pl = s["push"]["back"], s["push"]["left"]
    return (f"| {cid}: {CASES[cid][0]} | {s['falls']} of {s['runs']} | {f(s['t_fall'], 1)} | {f(s['vx03'])} "
            f"| {f(s['vx02'])}, {f(s['wz02'])} | {f(s['tau_mean'])} | {f(s['peak'], 1)} | {f(s['sat'], 0)} | {f(s['rate'], 3)} "
            f"| {pb[0]:.1f} / {pb[1]:.1f} | {pl[0]:.1f} / {pl[1]:.1f} |")


GROUPS = {"1": "Case 1: baseline", "2": "Case 2: course default gains, course current limit",
          "3": "Case 3: course default gains, 67 % of the current limit", "4": "Case 4: joint zero error (baseline gains)",
          "5": "Case 5: latency (baseline gains)", "6": "Case 6: torque limit (baseline gains)",
          "7": "Case 7: combined realistic cases"}


def tables(res):
    S = {cid: summary(res, cid) for cid in CASES}
    L = []
    for g, title in GROUPS.items():
        L += [f"### {title}", "", HEAD, "|" + "---|" * 11]
        L += [table_row(cid, S[cid]) for cid in CASES if cid == "1" or (cid[0] == g and g != "1")]
        L.append("")
    L += ["### Drive limits that the simulation does not enforce", "",
          f"Above these values the drive stops with an error, and the joint goes limp. Overspeed: {OVERSPEED:.0f} rad/s at the "
          f"joint. Regen: about {REGEN_W:.0f} W per drive. Bus current: about {BUS_W:.0f} W per drive. "
          "Power is the mechanical power at one joint, without motor losses. The peaks include the runs that fall, up to the fall.",
          "",
          "| Case | Peak joint speed (rad/s) | Peak power into one drive, regen (W) | Peak power from one drive (W) |",
          "|---|---|---|---|"]
    L += [f"| {c} | {S[c]['qd_peak']:.1f} | {S[c]['p_regen']:.0f} | {S[c]['p_drive']:.0f} |" for c in CASES]
    L += ["", "### All runs per command (falls, mean vx, mean yaw rate of the runs without a fall)", "",
          "| Case | " + " | ".join(s2s.COMMANDS) + " |", "|---|" + "---|" * len(s2s.COMMANDS)]
    for cid in CASES:
        cells = []
        for c in s2s.COMMANDS:
            rs = [r for r in res["walk"] if r["case"] == cid and r["cmd"] == c]
            ok = [r for r in rs if r["fall"] is None]
            fl = len(rs) - len(ok)
            cells.append(f"{fl} falls" + (f", vx {np.mean([r['vx'] for r in ok]):+.2f}, yaw {np.mean([r['wz'] for r in ok]):+.2f}"
                                          if ok else ""))
        L.append(f"| {cid} | " + " | ".join(cells) + " |")
    return "\n".join(L) + "\n", S


def plots(S):
    ids = list(CASES)
    y = np.arange(len(ids))[::-1]
    fig, axs = plt.subplots(1, 4, figsize=(15, 5.5), sharey=True)
    runs = S["1"]["runs"]
    panels = [("Falls (of %d runs)" % runs, [S[c]["falls"] for c in ids]),
              ("Mean vx at a 0.3 m/s command (m/s)", [S[c]["vx03"] or 0 for c in ids]),
              ("Peak joint torque (Nm)", [S[c]["peak"] for c in ids]),
              ("Largest backward push, walking, mean of 5 (N s)", [S[c]["push"]["back"][1] for c in ids])]
    for ax, (title, v) in zip(axs, panels):
        ax.barh(y, v, color=[report.S2 if c in COURSE_IDS else report.S1 for c in ids], height=0.65)
        ax.axvline(v[0], color=report.INK2, lw=1, ls="--")  # baseline
        ax.set_title(title, fontsize=10, loc="left")
        ax.grid(axis="x", color=report.GRID)
        ax.set_axisbelow(True)
    axs[1].axvline(0.3, color=report.INK, lw=1)
    axs[0].set_yticks(y, [f"{c}: {CASES[c][0]}" for c in ids], fontsize=8)
    fig.suptitle(f"Drive risks, ASMR robot, payload {MODEL}, {s2s.POLICY.parent.name}, {SEEDS} seeds x 4 commands. Dashed line: baseline. "
                 "Orange: course drive gains.", fontsize=10, x=0.01, ha="left")
    fig.savefig(OUT / "hw_risks.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


REPORT_HEAD = """# Sim-to-real risks of the course motor drives

**Status:** tested in simulation (MuJoCo, CPU only). Nothing here is measured on board.

**Not verified:** The MuJoCo actuator model has no motor dynamics, no CAN jitter and no gearbox backlash.
The joint friction and armature are those of the ASMR MJCF. We did not check the drive trip limits (last table) on an XDrive MINI.
They come from the course config and the ODrive v0.5.1 defaults.

**Assumed:** Hip roll, hip yaw, hip pitch and knee use the 6512 motor. Ankle pitch and ankle roll use the 5010 motor.
Torque constants 0.067 and 0.0836 Nm/A from the course drive config.

## Setup

- Robot: the ASMR 2026 robot with the payload given by `--payload` (default `f3`: 47 parts, 4.14 kg, 16.35 kg in all).
- Policy: `policies/<name>` (`--policy`, default baseline) at 25 Hz, physics at 2 kHz, the training observation noise, and the standing start of `run.py`.
- `hw_risks.py` imports the `run.py` functions. With the baseline settings, it gives the same result as `run.py` `simulate()` (self-check).
- Commands: the 4 commands of `run.py`: stand, vx 0.3, yaw 0.5, and vx 0.2 + yaw 0.5. Each run lasts 20 s.
- Seeds: {seeds} seeds per command, so each case has {runs} runs. The seed sets the sensor noise and the zero errors.
- Push test: the robot walks at 0.3 m/s. A 0.1 s push hits the torso, backward or to the left.
  A bisection finds the largest push without a fall in the next 3 s (resolution 0.375 N s).
- The {push_seeds} push seeds put the push at 3.0 to 3.4 s, so it hits 5 phases of one step. "Worst" is the smallest of the 5 values.
  A worst value of 0.0 means that the robot fell without a push before 6 s.
- Drive model: the drive does the PD. Torque = K × (target − measured position) − D × velocity, clipped to the joint limit.
  Case 1 uses the gains and limits of the MJCF (K 90 / 22 Nm/rad, D 5.7 / 0.49 Nm s/rad, 18.1 / 22.6 Nm, hip and knee / ankle), the values the policy trained with.
- Course gains: pos_gain 12.6 and vel_gain 0.223 (6512) give K = 100.6 Nm/rad and D = 8.0 Nm s/rad.
  pos_gain 20 and vel_gain 0.1 (5010) give K = 71.6 Nm/rad and D = 3.6 Nm s/rad.
  current_lim 18 A gives 18.1 Nm (6512) and 22.6 Nm (5010) at the joint.
- TRAP_TRAJ: the drive moves its setpoint to each new target at 4.2 rad/s and 2.1 rad/s² at most (joint side).
  These are trap_traj vel_limit 10 turn/s and accel 5 turn/s², divided by the gear ratio 15.
- Zero error: each joint gets a random offset, uniform in ±1°, ±2° or ±4°. The policy and the drive both see the offset position.
- Command latency: the drive gets the new target L ms after the policy computes it.
  Observation latency: the policy sees the sensor values of 10 ms before.
- Extra cases: 2b tests TRAP_TRAJ, because section 4 of the course config sets it. 5d finds the latency limit.
  7b to 7d combine the errors.
- Run time: {minutes:.1f} min on this machine.

Metrics:

- Falls: runs with a torso tilt above 0.78 rad or a large height drop, as in `run.py`. Mean time to fall uses only the runs that fall.
- vx: mean forward speed from 5 s to 20 s. Only runs without a fall count.
- Mean abs torque and peak torque: all 12 joints, from 1 s (after the start drop) to the end or the fall.
- Clipped steps: the share of physics steps where at least one joint asks for more than its limit.
- Action rate: the RMS change of the joint target between two policy steps, in rad. A higher value means a less smooth gait.

![Falls, speed, torque and push per case](hw_risks.png)

## Results

"""

REPORT_TAIL = """
## Findings and recommendation

Not written yet. Read the tables above and write here: which cases fall, how the course gains and TRAP_TRAJ compare with the MJCF gains,
the zero and latency limits, the drive settings to put on the 12 leg drives, and the bench checks before the first walk.
The drive settings of the older BHL study (K 20, D 2, 6 Nm) do not apply to this robot. That study is in `archive/sim2sim_bhl/hw_risks.md`.
"""


REGEN_TEXT = """
## Regen and the bus diode

**Status:** tested in simulation.
**Not verified:** the diode layout, the bus capacitance and the motor phase resistance.
**Assumed:** the phase resistance is 0.1 Ω for both motors. We found no datasheet value for the MAD M6C12 150KV or the MAD 5010 110KV.

- The course BOM has a Schottky diode module (MBRT400100, dual, common cathode) in the battery line. It probably joins the two 6S batteries.
- So regen current cannot flow back into the batteries. All 12 drives share one 24 V bus, and only their capacitors can take the energy.
- Capacitance: 5 × 100 µF per XDrive MINI (from photos), so 6 mF for 12 drives.
- Input power of one drive = τ × ω + 1.5 × R × I², with I = τ / (15 × Kt). Kt is 0.067 Nm/A (6512) and 0.0836 Nm/A (5010).
- The bus power is the sum over the 12 drives. When it is negative, the capacitors charge above the battery voltage. The script tracks this energy at each step.
- Start voltage: 25.2 V, a full 6S battery. We ignore the diode drop. The course config trips at 26 V (overvoltage).
- Runs: cases 1 and 7, 4 commands × 10 seeds, 20 s each. Runs that fall count up to the fall.

{table}
"""


def write(res):
    text, S = tables(res)
    (OUT / "hw_risks.md").write_text(REPORT_HEAD.format(minutes=res["minutes"], seeds=SEEDS, runs=4 * SEEDS, push_seeds=PUSH_SEEDS)
                                     + text + REPORT_TAIL
                                     + (REGEN_TEXT.format(table=regen_table(res)) if "regen" in res else ""),
                                     encoding="utf-8")
    plots(S)
    print(text)


def main():
    t0 = time.time()
    walk = [(cid, c, s) for cid in CASES for c in s2s.COMMANDS for s in range(SEEDS)]
    push = [(cid, k, s) for cid in CASES for k in PUSH_DIRS for s in range(PUSH_SEEDS)]
    with Pool(os.cpu_count()) as pool:
        res = {"push": pool.starmap(push_job, push, chunksize=1), "walk": pool.starmap(walk_job, walk, chunksize=1)}
    res["minutes"] = (time.time() - t0) / 60
    res["cases"] = {cid: {"label": lab, **{k: np.asarray(v).tolist() if isinstance(v, np.ndarray) else v
                                            for k, v in cfg.items()}} for cid, (lab, cfg) in CASES.items()}
    add_regen(res)
    (OUT / "hw_risks.json").write_text(json.dumps(res), encoding="utf-8")
    write(res)
    print(f"run time {res['minutes']:.1f} min, without the regen runs")


def video(cid, cmd_name, seed, out, seconds):
    """run.py video() with this drive model: run.py looks up simulate() at call time, so point it here."""
    s2s.simulate = lambda _m, command, _l, secs, frames=None, noise=False: simulate(CASES[cid][1], command, seed, secs, frames=frames)
    s2s.video(cmd_name, str(OUT / out), seconds)


def videos():
    res = json.loads((OUT / "hw_risks.json").read_text(encoding="utf-8"))
    w = sorted((r for r in res["walk"] if r["case"] == VIDEO_WORST and r["fall"] is not None), key=lambda r: r["fall"])
    r = w[len(w) // 2]  # the median fall, so the video shows a typical run
    video(VIDEO_WORST, r["cmd"], r["seed"], "hw_risks_worst.mp4", r["fall"] + 2.0)
    r = next(r for r in res["walk"] if r["case"] == VIDEO_REC and r["cmd"] == "vx 0.2 + yaw 0.5" and r["fall"] is None)
    video(VIDEO_REC, r["cmd"], r["seed"], "hw_risks_recommended.mp4", 10.0)


def check():
    K, D = joint_gains(12.6, 0.223)
    assert abs(K - 100.6) < 0.5 and abs(D - 7.99) < 0.05, (K, D)  # course config gains at the joint
    assert abs(COURSE_LIM[0] - 18.1) < 0.1 and abs(COURSE_LIM[4] - 22.6) < 0.1
    p, v = np.zeros(1), np.zeros(1)
    for _ in range(20000):  # 10 s: a 1 rad step takes 2 sqrt(1 / a) at a = 2.09 rad/s^2, below the speed limit
        p, v = trap_step(p, v, np.ones(1), s2s.PHYSICS_DT)
    assert abs(p[0] - 1) < 1e-3 and abs(v[0]) < 1e-2, (p, v)
    a = simulate(BASE, (0.3, 0.0, 0.0), 0, 6.0)
    s2s.PAYLOAD = MODEL  # run.simulate() loads the payload from this
    b = s2s.simulate(s2s.MODEL, (0.3, 0.0, 0.0), None, 6.0, noise=True)  # same noise seed, same loop -> same result
    assert a["fall"] is None and b["fall"] is None and abs(a["vx"] - b["vx"]) < 1e-6, (a, b)
    lat = simulate({**BASE, "cmd_ms": 40, "obs_ms": 10}, (0.0, 0.0, 0.0), 0, 2.0)
    assert lat["fall"] is None, lat
    print("check passed: gains, current limit, trap profile, baseline matches run.py, latency runs")


if __name__ == "__main__":
    if "--check" in sys.argv:
        check()
    elif "--video" in sys.argv:
        videos()
    elif "--regen" in sys.argv:
        res = json.loads((OUT / "hw_risks.json").read_text(encoding="utf-8"))
        add_regen(res)
        (OUT / "hw_risks.json").write_text(json.dumps(res), encoding="utf-8")
        print(regen_table(res))
    elif "--report" in sys.argv:
        write(json.loads((OUT / "hw_risks.json").read_text(encoding="utf-8")))
    else:
        main()
