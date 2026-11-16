"""Goal-reaching quality of walking policies on the nominal dynamics: straight walks, turns in place and standing.

Water is sprayed while standing, so walking only has to bring the robot to the target. Each environment runs one
trial: stand SETTLE_S, one command for its duration, then stand STOP_S. Errors are accumulated over the trial
(where the robot ends up), not instantaneous, because the step-by-step oscillation cancels out:

* walk (forward / backward): displacement along the starting heading vs. the commanded distance (%), sideways
  drift (cm) and heading change (deg), measured after the stop;
* turn in place: turned angle vs. the commanded angle (deg) and how far the base moved (cm), after the stop;
* stand: torso sway (RMS of the roll, pitch and yaw rates, rad/s), base drift and largest foot displacement (cm).

Dynamics, actuation and observation noise are as in the gap evaluation (eval_dynamics_gap.py) with nominal
dynamics; trials differ only in the observation noise.

Evaluate ONE policy per process (repeat the command per policy): on the course robot, a policy evaluated after another
one in the same process falls at the first step on high foot friction (mu >= 1.2), as if contact state survived the
reset; the first policy of a process is unaffected (Berkeley Humanoid Lite: negligible).

Usage (from the repository root):
    uv run --project scripts/isaaclab scripts/isaaclab/s2r/eval_goal_reaching.py --headless --out_dir logs/s2r_study/eval_goal \
        --policy DR000=logs/rsl_rl/s2r_dr000/<run>/model_11998.pt [--policy ...]
"""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Goal-reaching quality of walking policies on the nominal dynamics.")
parser.add_argument("--policy", action="append", required=True, metavar="NAME=CHECKPOINT", help="RSL-RL checkpoint.")
parser.add_argument("--repeats", type=int, default=32, help="Runs per trial type.")
parser.add_argument("--seed", type=int, default=0, help="Seed (observation noise).")
parser.add_argument("--out_dir", type=str, default="logs/s2r_study/eval_goal", help="Output directory.")
parser.add_argument(
    "--task", type=str, default="Velocity-Berkeley-Humanoid-Lite-Biped-S2R-DR000-v0", help="Task to run."
)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import csv
import gymnasium as gym
import math
import os
import sys
import torch

import isaaclab.utils.math as math_utils
from isaaclab_tasks.utils import parse_env_cfg

import berkeley_humanoid_lite.tasks  # noqa: F401

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from video_common import FALL_TILT_RAD, build_actor  # noqa: E402

SETTLE_S = 1.5
STOP_S = 2.0
# (name, kind, (vx, vy, wz), duration s)
TRIALS = [
    ("forward 0.20 m/s", "walk", (0.20, 0.0, 0.0), 4.0),
    ("forward 0.35 m/s", "walk", (0.35, 0.0, 0.0), 4.0),
    ("forward 0.50 m/s", "walk", (0.50, 0.0, 0.0), 4.0),
    ("backward 0.20 m/s", "walk", (-0.20, 0.0, 0.0), 4.0),
    ("backward 0.35 m/s", "walk", (-0.35, 0.0, 0.0), 4.0),
    ("turn left 0.5 rad/s", "turn", (0.0, 0.0, 0.5), 3.0),
    ("turn right 0.5 rad/s", "turn", (0.0, 0.0, -0.5), 3.0),
    ("turn left 1.0 rad/s", "turn", (0.0, 0.0, 1.0), 3.0),
    ("turn right 1.0 rad/s", "turn", (0.0, 0.0, -1.0), 3.0),
    ("stand", "stand", (0.0, 0.0, 0.0), 5.0),
]


def make_env(num_envs):
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=num_envs)
    env_cfg.seed = args_cli.seed
    env_cfg.episode_length_s = SETTLE_S + max(t[3] for t in TRIALS) + STOP_S + 2.0
    for term in ["physics_material", "add_base_mass", "scale_leg_mass", "shift_torso_com", "base_external_force_torque",
                 "add_all_joint_default_pos", "scale_all_actuator_torque_constant"]:
        if hasattr(env_cfg.events, term):
            setattr(env_cfg.events, term, None)
    env_cfg.events.reset_base.params["pose_range"] = {}
    env_cfg.events.reset_base.params["velocity_range"] = {}
    env_cfg.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
    # a fall is recorded, the run is not restarted
    env_cfg.terminations.base_orientation = None
    command_cfg = env_cfg.commands.base_velocity
    command_cfg.heading_command = False
    command_cfg.rel_standing_envs = 0.0
    command_cfg.resampling_time_range = (1.0e6, 1.0e6)
    command_cfg.debug_vis = False
    return gym.make(args_cli.task, cfg=env_cfg)


def main():
    n = len(TRIALS) * args_cli.repeats
    env = make_env(n)
    u = env.unwrapped
    device = u.device
    robot = u.scene["robot"]
    command_term = u.command_manager.get_term("base_velocity")
    feet = robot.find_bodies(".*_ankle_roll")[0]
    dt = u.step_dt

    trial_of_env = [i for i in range(len(TRIALS)) for _ in range(args_cli.repeats)]
    cmd = torch.tensor([TRIALS[i][2] for i in trial_of_env], device=device)
    dur = torch.tensor([TRIALS[i][3] for i in trial_of_env], device=device)
    # timing in policy steps: settle, command, stop
    k0 = round(SETTLE_S / dt)
    n_cmd = torch.round(dur / dt).long()
    n_stop = round(STOP_S / dt)
    k_end = k0 + n_cmd + n_stop  # per environment
    steps = int(k_end.max())
    os.makedirs(args_cli.out_dir, exist_ok=True)

    for spec in args_cli.policy:
        name, path = spec.split("=", 1)
        actor = build_actor(torch.load(path, map_location="cpu", weights_only=False)["model_state_dict"], device)
        fell = torch.zeros(n, dtype=torch.bool, device=device)
        p0 = torch.zeros(n, 2, device=device)
        yaw0 = torch.zeros(n, device=device)
        p_final = torch.zeros(n, 2, device=device)
        turned = torch.zeros(n, device=device)  # integrated yaw from t0 to the end of the trial
        feet0 = torch.zeros(n, len(feet), 2, device=device)
        foot_drift = torch.zeros(n, device=device)
        sway_sq = torch.zeros(n, device=device)
        sway_cnt = torch.zeros(n, device=device)
        with torch.inference_mode():
            obs = env.reset()[0]["policy"]
            for k in range(steps):
                if k == k0:
                    p0[:] = robot.data.root_pos_w[:, :2]
                    yaw0[:] = math_utils.euler_xyz_from_quat(robot.data.root_quat_w)[2]
                    feet0[:] = robot.data.body_pos_w[:, feet, :2]
                active = (k >= k0) & (k < k0 + n_cmd)
                command = cmd * active[:, None].float()
                command_term.vel_command_b[:] = command
                obs[:, 0:3] = command
                obs = env.step(actor(obs))[0]["policy"]
                in_trial = (k + 1 > k0) & (k + 1 <= k_end)
                turned += robot.data.root_ang_vel_w[:, 2] * dt * in_trial.float()
                tilt = torch.acos(torch.clamp(-robot.data.projected_gravity_b[:, 2], -1.0, 1.0))
                fell |= (tilt > FALL_TILT_RAD) & in_trial
                # final position at the end of each trial
                ending = k + 1 == k_end
                p_final[ending] = robot.data.root_pos_w[ending, :2]
                # standing: sway and foot drift while the command is zero for the whole window
                standing = in_trial & (k + 1 <= k0 + n_cmd)
                ang = math_utils.quat_rotate_inverse(math_utils.yaw_quat(robot.data.root_quat_w), robot.data.root_ang_vel_w)
                sway_sq += (ang ** 2).sum(-1) * standing.float()
                sway_cnt += standing.float()
                fd = torch.linalg.norm(robot.data.body_pos_w[:, feet, :2] - feet0, dim=-1).max(-1).values
                foot_drift = torch.where(in_trial, torch.maximum(foot_drift, fd), foot_drift)

        # walk: displacement in the starting heading frame
        d = p_final - p0
        c, s = torch.cos(yaw0), torch.sin(yaw0)
        along = d[:, 0] * c + d[:, 1] * s
        lateral = -d[:, 0] * s + d[:, 1] * c
        expected = cmd[:, 0] * dur
        rows = []
        for i in range(n):
            trial, kind, (vx, vy, wz), T = TRIALS[trial_of_env[i]]
            row = {"policy": name, "trial": trial, "kind": kind, "vx": vx, "wz": wz, "duration_s": T, "fell": int(fell[i])}
            if kind == "walk":
                row["distance_m"] = round(float(along[i]), 4)
                # positive: went farther than commanded, negative: fell short (forward and backward alike)
                row["distance_err_pct"] = round(float(100.0 * math.copysign(1.0, vx) * (along[i] - expected[i])
                                                      / expected[i].abs()), 2)
                row["lateral_cm"] = round(float(100.0 * lateral[i]), 2)
                row["heading_deg"] = round(float(math.degrees(turned[i])), 2)
            elif kind == "turn":
                row["turned_deg"] = round(float(math.degrees(turned[i])), 2)
                # positive: turned more than commanded, negative: less (left and right alike)
                row["turn_err_deg"] = round(math.copysign(1.0, wz) * (math.degrees(float(turned[i])) - math.degrees(wz * T)), 2)
                row["base_drift_cm"] = round(float(100.0 * torch.linalg.norm(d[i])), 2)
            else:
                row["sway_rms"] = round(float(torch.sqrt(sway_sq[i] / sway_cnt[i].clamp(min=1))), 4)
                row["base_drift_cm"] = round(float(100.0 * torch.linalg.norm(d[i])), 2)
                row["foot_drift_cm"] = round(float(100.0 * foot_drift[i]), 2)
            rows.append(row)
        keys = ["policy", "trial", "kind", "vx", "wz", "duration_s", "fell", "distance_m", "distance_err_pct",
                "lateral_cm", "heading_deg", "turned_deg", "turn_err_deg", "base_drift_cm", "sway_rms", "foot_drift_cm"]
        out = os.path.join(args_cli.out_dir, f"{name}.csv")
        with open(out, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(rows)
        # short summary
        def mean(kind, key, absolute=True):
            vals = [abs(r[key]) if absolute else r[key] for r in rows if r["kind"] == kind and not r["fell"]]
            return sum(vals) / max(1, len(vals))
        print(f"[RESULT] {name}: falls {int(fell.sum())}/{n} | walk |dist err| {mean('walk', 'distance_err_pct'):.1f}% "
              f"(signed {mean('walk', 'distance_err_pct', False):+.1f}%), |lateral| {mean('walk', 'lateral_cm'):.1f} cm, "
              f"|heading| {mean('walk', 'heading_deg'):.1f} deg | turn |angle err| {mean('turn', 'turn_err_deg'):.1f} deg "
              f"(signed {mean('turn', 'turn_err_deg', False):+.1f}), drift {mean('turn', 'base_drift_cm'):.1f} cm | "
              f"stand sway {mean('stand', 'sway_rms'):.3f} rad/s, base drift {mean('stand', 'base_drift_cm'):.1f} cm, "
              f"foot drift {mean('stand', 'foot_drift_cm'):.1f} cm -> {out}", flush=True)
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
