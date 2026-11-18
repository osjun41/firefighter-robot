"""Sustained push evaluation: how much steady force can a standing controller hold, and how well?

The robot stands for SETTLE_S, then a horizontal force of fixed magnitude pushes it for --window seconds,
along a direction fixed in its heading frame (default 180 deg: backward, like a water jet's reaction when
spraying forward). The force acts at a point --app_offset metres above the torso CoM (body frame), so it
also tips the robot like a nozzle mounted above the torso. Every environment runs one (force, trial).

Controllers, all driven with a zero velocity command unless stated otherwise:
  * the policy alone (--policy)
  * the policy with a feedforward forward-velocity command during the push (--ff_gain, m/s per N), and
    optionally position feedback on the drift (--pos_gain, 1/s)
  * a policy that also observes the push signal (planned force / PUSH_SIGNAL_SCALE, appended to the
    observation), detected from the ONNX input size; the signal can lead the force by --lead seconds
  * switching to a second policy (--brace_config) while the push signal is on

After --window the force is released and the robot is watched for --post more seconds, since a policy
that leans into the push can fall once it stops. Displacement and falls count over the whole time.

Modes: "nominal" (no randomization or noise, one trial per force) or "robust" (training randomization and
observation noise, --trials per force). A trial "holds" if the robot neither falls nor moves more than
HOLD_DISPLACEMENT_M from where the push started.

With --video, one nominal trial (a single force) is recorded instead, following the robot with captions
and a red arrow for the push; the fall termination is then off so the fall stays on screen.

Usage (from the repository root):
    uv run --project sim/locomotion sim/locomotion/scripts/push_test/eval_sustained_push.py --policy baseline --name baseline \
        --mode nominal --forces 0 50 1 --headless
"""

"""Launch Isaac Sim Simulator first."""

import argparse
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))  # repository root: policies/policy_select.py
from policies import policy_select  # noqa: E402
import onnxruntime  # noqa: F401,E402  Windows: load its DLLs before Isaac Sim starts, or the import fails afterwards
import csv
import os
import sys

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Sustained push evaluation in Isaac Lab.")
policy_select.add_argument(parser)
parser.add_argument("--name", type=str, required=True, help="Name of the result file.")
parser.add_argument("--out_dir", type=str, default="logs/sustained_push", help="Output directory.")
parser.add_argument("--mode", choices=["nominal", "robust"], default="nominal", help="Test conditions.")
parser.add_argument("--trials", type=int, default=20, help="Trials per force in robust mode.")
parser.add_argument("--forces", type=float, nargs=3, default=[0, 50, 1], metavar=("START", "STOP", "STEP"))
parser.add_argument("--direction", type=float, default=180.0, help="Push direction in the heading frame (deg).")
parser.add_argument("--app_offset", type=float, default=0.20, help="Push point above the torso CoM (m).")
parser.add_argument("--window", type=float, default=10.0, help="Push duration (s).")
parser.add_argument("--post", type=float, default=3.0, help="Time after the push is released (s).")
parser.add_argument("--ramp", type=float, default=0.1, help="Time for the force to build up or fall off (s).")
parser.add_argument("--lead", type=float, default=0.0, help="Push signal lead before the force onset (s).")
parser.add_argument("--brace_hold", type=float, default=1.0, help="Keep the brace policy this long after release (s).")
parser.add_argument("--ff_gain", type=float, default=0.0, help="Forward velocity command per newton (m/s/N).")
parser.add_argument("--pos_gain", type=float, default=0.0, help="Velocity command per metre of drift (1/s).")
parser.add_argument("--max_cmd", type=float, default=0.5, help="Limit of the forward velocity command (m/s).")
parser.add_argument("--brace_config", type=str, default=None, help="Policy to switch to while the signal is on.")
parser.add_argument("--seed", type=int, default=0, help="Seed of the randomization.")
parser.add_argument("--video", type=str, default=None, help="Record one nominal trial to this video (.mp4).")
parser.add_argument("--video_nocaption", type=str, default=None, help="Also write the video without captions.")
parser.add_argument("--label", type=str, default=None, help="Controller name in the video captions.")
parser.add_argument(
    "--task", type=str, default="Velocity-ASMR-Biped-v0", help="Name of the task."
)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
# rendering needs cameras
if args_cli.video:
    args_cli.enable_cameras = True

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import math
import torch

from omegaconf import OmegaConf

import isaaclab.utils.math as math_utils
from isaaclab_tasks.utils import parse_env_cfg

from berkeley_humanoid_lite.tasks.locomotion.velocity.mdp.push import PUSH_SIGNAL_SCALE

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from push_common import (  # noqa: E402
    FALL_TILT_DEG,
    SETTLE_S,
    STEP_AIR_TIME_S,
    OnnxPolicy,
    PushArrow,
    draw_caption,
    make_standing_env_cfg,
    tilt_deg,
)

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scenario"))
from scenario_common import CAM_EYE, CAM_LOOKAT  # noqa: E402

HOLD_DISPLACEMENT_M = 0.10
# keep recording this long after a fall
FALL_TAIL_S = 1.5


def force_range(start, stop, step):
    return [round(start + i * step, 3) for i in range(int(round((stop - start) / step)) + 1)]


def describe_controller(sizes, num_obs):
    parts = []
    if args_cli.brace_config:
        parts.append("switch to brace policy while spraying")
    if any(size == num_obs + 1 for size in sizes.values()):
        parts.append(f"push signal observed (lead {args_cli.lead:.1f} s)")
    if args_cli.ff_gain > 0.0:
        parts.append(f"feed-forward walk command {args_cli.ff_gain * 1000:.0f} mm/s per N")
    if args_cli.pos_gain > 0.0:
        parts.append(f"drift feedback {args_cli.pos_gain:.1f} 1/s")
    return ", ".join(parts) if parts else "zero velocity command"


def record_frame(env, writers, arrow, robot, base_id, force_w, offset_b, t_push, label, controller, force_mag, fell,
                 tilt, start_pos, contact, feet_ids):
    """Draw the push arrow and the captions, and append the frame to the videos."""
    com_w = robot.data.body_com_pos_w[:, base_id]
    push_point_w = com_w + math_utils.quat_apply(robot.data.root_quat_w, offset_b)
    arrow.update(push_point_w, force_w)
    frame = env.render()
    if len(writers) > 1:
        writers[1].append_data(frame)

    moved = 0.0 if start_pos is None else torch.linalg.norm(robot.data.root_pos_w[0, :2] - start_pos[0]).item()
    stepping = contact.data.current_air_time[0, feet_ids].max().item() > STEP_AIR_TIME_S
    if t_push < 0.0:
        phase = "Standing, no push yet"
    elif t_push < args_cli.window:
        phase = f"Pushing  {t_push:4.1f} / {args_cli.window:.0f} s"
    else:
        phase = f"Push released, watching recovery  {t_push - args_cli.window:3.1f} / {args_cli.post:.0f} s"
    if bool(fell[0]):
        status, kind = f"FELL (tilt > {FALL_TILT_DEG:.0f} deg)", "bad"
    elif moved > HOLD_DISPLACEMENT_M:
        status, kind = f"Pushed off its spot ({moved:.2f} m)", "warn"
    elif stepping:
        status, kind = "Stepping to hold", "warn"
    elif t_push < 0.0:
        status, kind = "Standing", "neutral"
    else:
        status, kind = "Holding in place", "good"
    lines = [
        f"Sustained backward push (water-jet recoil model)  |  {label}",
        f"{force_mag[0].item():.0f} N backward at the nozzle point (torso CoM + {args_cli.app_offset:.2f} m)",
        f"{phase}   |   {controller}",
        f"Tilt {tilt[0].item():4.1f} deg     Moved {moved:4.2f} m",
    ]
    writers[0].append_data(draw_caption(frame, lines, status, kind))


def main():
    """Run the evaluation and write per-trial results."""
    deploy_cfg = OmegaConf.load(policy_select.config(args_cli.policy))
    forces = force_range(*args_cli.forces)
    trials = 1 if args_cli.mode == "nominal" else args_cli.trials
    num_envs = len(forces) * trials

    video = args_cli.video is not None
    if video and (num_envs != 1 or args_cli.mode != "nominal"):
        raise ValueError("--video records one nominal trial: give a single force (START == STOP)")

    env_cfg = make_standing_env_cfg(
        args_cli.task, args_cli.device, num_envs, deploy_cfg.policy_dt, terminate_on_fall=not video
    )
    env_cfg.seed = args_cli.seed
    if video:
        # follow the robot, same view as the scenario videos
        env_cfg.viewer.origin_type = "asset_root"
        env_cfg.viewer.asset_name = "robot"
        env_cfg.viewer.env_index = 0
        env_cfg.viewer.eye = CAM_EYE
        env_cfg.viewer.lookat = CAM_LOOKAT
    if args_cli.mode == "robust":
        # training randomization and observation noise; the reset stays nominal and the push replaces
        # the small random reset wrench
        train_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=num_envs)
        for term in ["physics_material", "add_base_mass", "add_all_joint_default_pos",
                     "scale_all_actuator_torque_constant"]:
            setattr(env_cfg.events, term, getattr(train_cfg.events, term))
        env_cfg.observations.policy.enable_corruption = True

    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if video else None)
    unwrapped = env.unwrapped
    device = unwrapped.device
    robot = unwrapped.scene["robot"]
    contact = unwrapped.scene["contact_forces"]
    base_id = robot.find_bodies("base")[0][0]
    feet_ids = contact.find_bodies(".*_ankle_roll")[0]
    command_term = unwrapped.command_manager.get_term("base_velocity")

    policy = OnnxPolicy(deploy_cfg.policy_checkpoint_path,
                        (deploy_cfg.action_limit_lower, deploy_cfg.action_limit_upper))
    brace = None
    if args_cli.brace_config:
        brace_cfg = OmegaConf.load(args_cli.brace_config)
        brace = OnnxPolicy(brace_cfg.policy_checkpoint_path, (brace_cfg.action_limit_lower, brace_cfg.action_limit_upper))
    num_obs = env.observation_space["policy"].shape[-1]
    sizes = {"policy": policy.session.get_inputs()[0].shape[-1]}
    if brace is not None:
        sizes["brace"] = brace.session.get_inputs()[0].shape[-1]
    for key, size in sizes.items():
        if size not in (num_obs, num_obs + 1):
            raise ValueError(f"{key} expects {size} observations; the task gives {num_obs} (+1 with the signal)")

    force_mag = torch.tensor([f for f in forces for _ in range(trials)], device=device)
    direction = math.radians(args_cli.direction)
    dir_h = torch.tensor([math.cos(direction), math.sin(direction), 0.0], device=device)
    offset_b = torch.tensor([0.0, 0.0, args_cli.app_offset], device=device).expand(num_envs, 3)

    dt = unwrapped.step_dt
    steps = int(round((SETTLE_S + args_cli.window + args_cli.post) / dt))
    if video:
        import imageio

        arrow = PushArrow()
        writers = []
        for path in [args_cli.video, args_cli.video_nocaption]:
            if path:
                os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
                writers.append(imageio.get_writer(path, fps=int(round(1.0 / dt)), codec="libx264", quality=8,
                                                  macro_block_size=1))
        label = args_cli.label or args_cli.policy
        controller = describe_controller(sizes, num_obs)
    fell = torch.zeros(num_envs, dtype=torch.bool, device=device)
    fall_time = torch.full((num_envs,), float("nan"), device=device)
    max_tilt = torch.zeros(num_envs, device=device)
    max_disp = torch.zeros(num_envs, device=device)
    air_steps = torch.zeros(num_envs, device=device)
    push_steps = 0
    start_pos = None

    obs, _ = env.reset()
    obs = obs["policy"]
    for k in range(steps):
        t_push = k * dt - SETTLE_S
        pushing = 0.0 <= t_push < args_cli.window
        signal_on = -args_cli.lead <= t_push < args_cli.window
        # force ramps in and out like a valve
        ramp_in = min(max(t_push / max(args_cli.ramp, dt), 0.0), 1.0)
        ramp_out = min(max((args_cli.window - t_push) / max(args_cli.ramp, dt), 0.0), 1.0)
        active = (~fell).float() * ramp_in * ramp_out

        # push along the heading, at a point above the torso CoM
        yaw_quat = math_utils.yaw_quat(robot.data.root_quat_w)
        force_w = math_utils.quat_apply(yaw_quat, dir_h.expand(num_envs, 3)) * (force_mag * active).unsqueeze(-1)
        force_b = math_utils.quat_rotate_inverse(robot.data.root_quat_w, force_w)
        torque_b = torch.cross(offset_b, force_b, dim=-1)
        robot.set_external_force_and_torque(force_b.unsqueeze(1), torque_b.unsqueeze(1), body_ids=[base_id])
        if t_push >= 0.0 and start_pos is None:
            start_pos = robot.data.root_pos_w[:, :2].clone()
            start_yaw = math_utils.yaw_quat(robot.data.root_quat_w)

        # forward velocity command: feedforward on the planned force, feedback on the drift
        cmd_vx = torch.zeros(num_envs, device=device)
        if signal_on:
            cmd_vx += args_cli.ff_gain * force_mag
        if t_push >= 0.0 and args_cli.pos_gain > 0.0:
            drift_w = torch.zeros(num_envs, 3, device=device)
            drift_w[:, :2] = robot.data.root_pos_w[:, :2] - start_pos
            drift_h = math_utils.quat_rotate_inverse(start_yaw, drift_w)
            cmd_vx -= args_cli.pos_gain * drift_h[:, 0]
        cmd_vx = torch.clamp(cmd_vx, -args_cli.max_cmd, args_cli.max_cmd)
        signal = (force_mag * float(signal_on) / PUSH_SIGNAL_SCALE).unsqueeze(-1)

        with torch.inference_mode():
            command_term.vel_command_b[:] = 0.0
            command_term.vel_command_b[:, 0] = cmd_vx
            obs[:, 0:3] = command_term.vel_command_b
            obs_signal = torch.cat([obs, signal], dim=-1)
            use_brace = brace is not None and -args_cli.lead <= t_push < args_cli.window + args_cli.brace_hold
            net = brace if use_brace else policy
            net_obs = obs_signal if sizes["brace" if use_brace else "policy"] == num_obs + 1 else obs
            obs, _, terminated, _, _ = env.step(net(net_obs))
            obs = obs["policy"]

        tilt = tilt_deg(robot.data.projected_gravity_b)
        if video:
            record_frame(env, writers, arrow, robot, base_id, force_w, offset_b, t_push, label, controller,
                         force_mag, fell, tilt, start_pos, contact, feet_ids)
            if bool(fell[0]) and t_push - fall_time[0].item() >= FALL_TAIL_S:
                break
        if t_push < 0.0:
            continue
        push_steps += int(pushing)
        # a fall ends an episode by termination; with the termination off (video) the tilt shows it
        fallen = terminated | (tilt > FALL_TILT_DEG)
        fall_time = torch.where(fallen & ~fell, torch.full_like(fall_time, t_push + dt), fall_time)
        fell |= fallen
        alive = ~fell
        max_tilt = torch.where(alive, torch.maximum(max_tilt, tilt), max_tilt)
        disp = torch.linalg.norm(robot.data.root_pos_w[:, :2] - start_pos, dim=-1)
        max_disp = torch.where(alive, torch.maximum(max_disp, disp), max_disp)
        in_air = contact.data.current_air_time[:, feet_ids].max(dim=1).values > STEP_AIR_TIME_S
        air_steps += (in_air & alive).float() * float(pushing)

    if video:
        for writer in writers:
            writer.close()
        state = "fell" if bool(fell[0]) else "held" if max_disp[0].item() <= HOLD_DISPLACEMENT_M else "drifted"
        print(f"[INFO] Video: {state} (max displacement {max_disp[0].item():.2f} m) -> {args_cli.video}")
        env.close()
        return

    holds = ~fell & (max_disp <= HOLD_DISPLACEMENT_M)
    os.makedirs(args_cli.out_dir, exist_ok=True)
    path = os.path.join(args_cli.out_dir, f"{args_cli.name}.csv")
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["name", "mode", "force_N", "trial", "fell", "fall_time_s", "held", "max_displacement_m",
                         "max_tilt_deg", "stepping_fraction"])
        for i in range(num_envs):
            writer.writerow([
                args_cli.name, args_cli.mode, force_mag[i].item(), i % trials, int(fell[i]),
                f"{fall_time[i].item():.2f}", int(holds[i]), f"{max_disp[i].item():.3f}",
                f"{max_tilt[i].item():.1f}", f"{(air_steps[i] / max(push_steps, 1)).item():.2f}",
            ])

    # summary: success rate per force
    survive = (~fell).float().view(len(forces), trials).mean(dim=1)
    hold = holds.float().view(len(forces), trials).mean(dim=1)
    print(f"[INFO] Wrote {path}")
    print("[RESULT] force_N  survive  hold")
    for f, s, h in zip(forces, survive.tolist(), hold.tolist()):
        print(f"[RESULT] {f:7.1f}  {s:7.2f}  {h:5.2f}")

    # close the simulator
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
