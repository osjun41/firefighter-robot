"""Standing push test: how hard can the robot be pushed while it is told to stand still?

Every environment runs one trial (push type, direction, force). The robot stands for SETTLE_S, then a
horizontal force is applied at the torso (base) CoM, as a short impulse (IMPULSE_S) or sustained for the
whole window. A trial fails if the robot falls (base tilt over 0.78 rad, the training termination) within
WINDOW_S of the push onset. Conditions are nominal: no randomization, observation noise or reset perturbation.

Writes one CSV row per trial to <out_dir>/<name>.csv. Plot the result with plot_push_radar.py.

Usage (from the repository root):
    uv run --project scripts/isaaclab scripts/isaaclab/push_test/run_push_test.py --config configs/policy_latest.yaml --name ours --headless \
        [--impulse_range 0 80 1] [--sustained_range 0 25 0.5]
"""

"""Launch Isaac Sim Simulator first."""

import argparse
import csv
import os
import sys

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Standing push test in Isaac Lab.")
parser.add_argument("--config", type=str, required=True, help="Path to the deployment config of the policy.")
parser.add_argument("--name", type=str, required=True, help="Name of the result file and of the policy in it.")
parser.add_argument("--out_dir", type=str, default="logs/push_test", help="Output directory.")
parser.add_argument(
    "--impulse_range", type=float, nargs=3, default=[0, 200, 10], metavar=("START", "STOP", "STEP"),
    help="Impulse forces to test in N, STOP included.",
)
parser.add_argument(
    "--sustained_range", type=float, nargs=3, default=[0, 80, 5], metavar=("START", "STOP", "STEP"),
    help="Sustained forces to test in N, STOP included.",
)
parser.add_argument(
    "--task", type=str, default="Velocity-Berkeley-Humanoid-Lite-Biped-v0", help="Name of the task."
)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import torch

from omegaconf import OmegaConf

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from push_common import (  # noqa: E402
    DIRECTIONS_DEG,
    IMPULSE_S,
    SETTLE_S,
    STEP_AIR_TIME_S,
    WINDOW_S,
    OnnxPolicy,
    horizontal_force_w,
    make_standing_env_cfg,
    tilt_deg,
    world_to_body,
)


def force_range(start, stop, step):
    return [round(start + i * step, 3) for i in range(int(round((stop - start) / step)) + 1)]


def main():
    """Run all trials in parallel and write the results."""
    deploy_cfg = OmegaConf.load(args_cli.config)
    trials = [("impulse", d, f) for d in DIRECTIONS_DEG for f in force_range(*args_cli.impulse_range)]
    trials += [("sustained", d, f) for d in DIRECTIONS_DEG for f in force_range(*args_cli.sustained_range)]
    num_trials = len(trials)
    print(f"[INFO] Running {num_trials} trials")

    env_cfg = make_standing_env_cfg(args_cli.task, args_cli.device, num_trials, deploy_cfg.policy_dt)
    env = gym.make(args_cli.task, cfg=env_cfg)
    unwrapped = env.unwrapped
    device = unwrapped.device
    robot = unwrapped.scene["robot"]
    contact = unwrapped.scene["contact_forces"]
    base_id = robot.find_bodies("base")[0][0]
    feet_ids = contact.find_bodies(".*_ankle_roll")[0]
    command_term = unwrapped.command_manager.get_term("base_velocity")
    policy = OnnxPolicy(deploy_cfg.policy_checkpoint_path)

    directions = torch.tensor([float(d) for _, d, _ in trials], device=device)
    magnitudes = torch.tensor([float(f) for _, _, f in trials], device=device)
    durations = torch.tensor([IMPULSE_S if t == "impulse" else WINDOW_S for t, _, _ in trials], device=device)
    force_w = horizontal_force_w(directions, magnitudes)

    fell = torch.zeros(num_trials, dtype=torch.bool, device=device)
    fell_during_settle = torch.zeros_like(fell)
    fall_time = torch.full((num_trials,), float("nan"), device=device)
    max_tilt = torch.zeros(num_trials, device=device)
    max_air_time = torch.zeros(num_trials, device=device)
    displacement = torch.zeros(num_trials, device=device)
    start_pos = None

    obs, _ = env.reset()
    obs = obs["policy"]
    dt = unwrapped.step_dt
    for k in range(int(round((SETTLE_S + WINDOW_S) / dt))):
        t_push = k * dt - SETTLE_S
        # push at the torso CoM; stop pushing a robot once it has fallen (it is reset by the termination)
        active = (t_push >= 0) & (t_push < durations) & ~fell
        force_b = world_to_body(robot.data.root_quat_w, force_w * active.unsqueeze(-1))
        robot.set_external_force_and_torque(
            force_b.unsqueeze(1), torch.zeros_like(force_b).unsqueeze(1), body_ids=[base_id]
        )
        if start_pos is None and t_push >= 0:
            start_pos = robot.data.root_pos_w[:, :2].clone()

        with torch.inference_mode():
            # stand still
            command_term.vel_command_b[:] = 0.0
            obs[:, 0:3] = 0.0
            obs, _, terminated, _, _ = env.step(policy(obs))
            obs = obs["policy"]

        if t_push < 0:
            fell_during_settle |= terminated
            continue
        fall_time = torch.where(terminated & ~fell, torch.full_like(fall_time, t_push + dt), fall_time)
        fell |= terminated
        alive = ~fell
        max_tilt = torch.where(alive, torch.maximum(max_tilt, tilt_deg(robot.data.projected_gravity_b)), max_tilt)
        air_time = contact.data.current_air_time[:, feet_ids].max(dim=1).values
        max_air_time = torch.where(alive, torch.maximum(max_air_time, air_time), max_air_time)
        moved = torch.linalg.norm(robot.data.root_pos_w[:, :2] - start_pos, dim=-1)
        displacement = torch.where(alive, moved, displacement)

    os.makedirs(args_cli.out_dir, exist_ok=True)
    path = os.path.join(args_cli.out_dir, f"{args_cli.name}.csv")
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "policy", "type", "direction_deg", "force_N", "impulse_Ns", "fell", "fall_time_s", "max_tilt_deg",
            "displacement_m", "stepped", "fell_during_settle",
        ])
        for i, (push_type, direction, force) in enumerate(trials):
            impulse = force * (IMPULSE_S if push_type == "impulse" else WINDOW_S)
            writer.writerow([
                args_cli.name, push_type, direction, force, f"{impulse:.1f}", int(fell[i]),
                f"{fall_time[i].item():.2f}",
                f"{max_tilt[i].item():.1f}", f"{displacement[i].item():.3f}",
                int(max_air_time[i].item() > STEP_AIR_TIME_S), int(fell_during_settle[i]),
            ])
    print(f"[INFO] Wrote {path}: {int(fell.sum())} of {num_trials} trials fell "
          f"({int(fell_during_settle.sum())} during the settling phase)")

    # close the simulator
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
