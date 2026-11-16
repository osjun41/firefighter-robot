"""Fall rate of policies on the video scenario (video_common.SCENARIO) for several torso masses.

Each (torso mass, repeat) is one environment; repeats differ only in the observation noise, as in training.
Used to choose and caption the dynamics-gap videos: the recorded run is one sample, these numbers show how
typical it is.

Usage (from the repository root):
    uv run --project scripts/isaaclab scripts/isaaclab/s2r/scenario_fall_rate.py --headless --torso_mass_add -2,0,1,2,3 --repeats 32 \
        --policy DR000=logs/rsl_rl/s2r_dr000/<run>/model_11998.pt [--policy ...] [--out logs/s2r_study/x.csv]
"""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Fall rate of policies on the video scenario.")
parser.add_argument("--policy", action="append", required=True, metavar="NAME=CHECKPOINT", help="RSL-RL checkpoint.")
parser.add_argument("--torso_mass_add", type=str, required=True, help="Comma-separated torso mass changes (kg).")
parser.add_argument("--repeats", type=int, default=32, help="Runs per torso mass.")
parser.add_argument("--seed", type=int, default=0, help="Seed.")
parser.add_argument("--out", type=str, default=None, help="Optional CSV with one row per policy and mass.")
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
import os
import sys
import torch

from isaaclab_tasks.utils import parse_env_cfg

import berkeley_humanoid_lite.tasks  # noqa: F401

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from video_common import (  # noqa: E402
    DURATION_S,
    FALL_TILT_RAD,
    build_actor,
    command_at,
    measured_velocity,
    set_torso_mass,
)


def make_env(num_envs):
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=num_envs)
    env_cfg.seed = args_cli.seed
    env_cfg.episode_length_s = DURATION_S + 2.0
    # dynamics are set below; no randomization or reset perturbation; observation noise as in training
    for term in ["physics_material", "add_base_mass", "scale_leg_mass", "shift_torso_com", "base_external_force_torque",
                 "add_all_joint_default_pos", "scale_all_actuator_torque_constant"]:
        if hasattr(env_cfg.events, term):
            setattr(env_cfg.events, term, None)
    env_cfg.events.reset_base.params["pose_range"] = {}
    env_cfg.events.reset_base.params["velocity_range"] = {}
    env_cfg.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
    command_cfg = env_cfg.commands.base_velocity
    command_cfg.heading_command = False
    command_cfg.rel_standing_envs = 0.0
    command_cfg.resampling_time_range = (1.0e6, 1.0e6)
    command_cfg.debug_vis = False
    return gym.make(args_cli.task, cfg=env_cfg)


def main():
    masses = [float(m) for m in args_cli.torso_mass_add.split(",")]
    n = len(masses) * args_cli.repeats
    env = make_env(n)
    unwrapped = env.unwrapped
    device = unwrapped.device
    robot = unwrapped.scene["robot"]
    command_term = unwrapped.command_manager.get_term("base_velocity")
    base_id = robot.find_bodies("base")[0][0]
    mass_of_env = torch.tensor([m for m in masses for _ in range(args_cli.repeats)])
    set_torso_mass(robot, base_id, mass_of_env)
    dt = unwrapped.step_dt
    steps = int(round(DURATION_S / dt))
    rows = []

    for spec in args_cli.policy:
        name, path = spec.split("=", 1)
        actor = build_actor(torch.load(path, map_location="cpu", weights_only=False)["model_state_dict"], device)
        fell = torch.zeros(n, dtype=torch.bool, device=device)
        fall_time = torch.full((n,), float("nan"), device=device)
        lin_err = torch.zeros(n, device=device)
        yaw_err = torch.zeros(n, device=device)
        upright = torch.zeros(n, device=device)
        with torch.inference_mode():
            obs = env.reset()[0]["policy"]
            for k in range(steps):
                _, cmd = command_at(k * dt)
                command = torch.tensor(cmd, device=device).expand(n, 3)
                command_term.vel_command_b[:] = command
                obs[:, 0:3] = command
                obs, _, terminated, _, _ = env.step(actor(obs))
                obs = obs["policy"]
                vx, vy, wz, tilt = measured_velocity(robot)
                fallen = (terminated | (tilt > FALL_TILT_RAD)) & ~fell
                fall_time[fallen] = (k + 1) * dt
                fell |= fallen
                alive = (~fell).float()
                lin_err += torch.sqrt((vx - cmd[0]) ** 2 + (vy - cmd[1]) ** 2) * alive
                yaw_err += torch.abs(wz - cmd[2]) * alive
                upright += alive
        for m in masses:
            sel = (mass_of_env == m).to(device)
            falls = int(fell[sel].sum())
            times = fall_time[sel & fell]
            row = {"policy": name, "torso_mass_add": m, "runs": int(sel.sum()), "falls": falls,
                   "fall_rate": falls / int(sel.sum()),
                   "mean_fall_time_s": float(times.mean()) if len(times) else float("nan"),
                   "lin_err": float((lin_err[sel] / upright[sel].clamp(min=1)).mean()),
                   "yaw_err": float((yaw_err[sel] / upright[sel].clamp(min=1)).mean())}
            rows.append(row)
            print(f"[RESULT] {name:8s} torso {m:+5.2f} kg: falls {falls:2d}/{row['runs']} "
                  f"(mean fall time {row['mean_fall_time_s']:.1f} s), lin err {row['lin_err']:.3f}, "
                  f"yaw err {row['yaw_err']:.3f}", flush=True)
    if args_cli.out:
        os.makedirs(os.path.dirname(os.path.abspath(args_cli.out)), exist_ok=True)
        with open(args_cli.out, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
