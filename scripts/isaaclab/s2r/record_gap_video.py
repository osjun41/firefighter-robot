"""Record one policy walking the video scenario (video_common.SCENARIO) with a changed torso mass, in Isaac Lab.

The robot is followed by the same camera as the earlier scenario and push videos. Dynamics are as in the gap
evaluation (actuation exact, observation noise as in training, no randomization or push); only the torso mass
is changed. The fall termination is disabled so a fall stays on screen; the video ends FALL_TAIL_S after a fall.

One run writes up to three files: <name>.mp4 (English captions), <name>_graph.mp4 (captions plus strip charts
of commanded vs actual speed and yaw rate, with --graph) and <name>_nocaption.mp4 (no overlay).

Usage (from the repository root):
    uv run --project scripts/isaaclab scripts/isaaclab/s2r/record_gap_video.py --headless --policy logs/rsl_rl/s2r_dr000/<run>/model_11998.pt \
        --title "..." --torso_mass_add 2.0 --stat "line 1|line 2" --out_dir videos/s2r --name x [--graph]
"""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Record a policy on the video scenario with a changed torso mass.")
parser.add_argument("--policy", type=str, required=True, help="RSL-RL checkpoint.")
parser.add_argument("--title", type=str, required=True, help="Caption title (policy).")
parser.add_argument("--torso_mass_add", type=float, default=0.0, help="Torso mass change (kg).")
parser.add_argument("--stat", type=str, default="", help="Statistics box lines, separated by '|'.")
parser.add_argument("--out_dir", type=str, default="videos/s2r", help="Output folder.")
parser.add_argument("--name", type=str, required=True, help="Base file name.")
parser.add_argument("--graph", action="store_true", default=False, help="Also write the version with strip charts.")
parser.add_argument("--seed", type=int, default=0, help="Seed (observation noise).")
parser.add_argument(
    "--task", type=str, default="Velocity-Berkeley-Humanoid-Lite-Biped-S2R-DR000-v0", help="Task to run."
)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
# rendering needs cameras
args_cli.enable_cameras = True

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import imageio
import os
import sys
import torch

from isaaclab.envs.mdp.commands.velocity_command import UniformVelocityCommand
from isaaclab_tasks.utils import parse_env_cfg

import berkeley_humanoid_lite.tasks  # noqa: F401

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from video_common import (  # noqa: E402
    ARROW_HEIGHT,
    CAM_EYE,
    CAM_LOOKAT,
    DURATION_S,
    FALL_TILT_RAD,
    TORSO_MASS_KG,
    build_actor,
    caption_overlay,
    command_at,
    graph_overlay,
    measured_velocity,
    set_torso_mass,
)

FALL_TAIL_S = 2.5


def _debug_vis_above_head(self, event):
    """Same as UniformVelocityCommand._debug_vis_callback, but drawn above the head instead of at 0.5 m."""
    if not self.robot.is_initialized:
        return
    base_pos_w = self.robot.data.root_pos_w.clone()
    base_pos_w[:, 2] += ARROW_HEIGHT
    vel_des_arrow_scale, vel_des_arrow_quat = self._resolve_xy_velocity_to_arrow(self.command[:, :2])
    vel_arrow_scale, vel_arrow_quat = self._resolve_xy_velocity_to_arrow(self.robot.data.root_lin_vel_b[:, :2])
    self.goal_vel_visualizer.visualize(base_pos_w, vel_des_arrow_quat, vel_des_arrow_scale)
    self.current_vel_visualizer.visualize(base_pos_w, vel_arrow_quat, vel_arrow_scale)


UniformVelocityCommand._debug_vis_callback = _debug_vis_above_head


def main():
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=1)
    env_cfg.seed = args_cli.seed
    env_cfg.episode_length_s = DURATION_S + 10.0
    env_cfg.sim.render_interval = env_cfg.decimation
    # as in the gap evaluation: dynamics set below, no randomization, push or reset perturbation
    for term in ["physics_material", "add_base_mass", "scale_leg_mass", "shift_torso_com", "base_external_force_torque",
                 "add_all_joint_default_pos", "scale_all_actuator_torque_constant"]:
        if hasattr(env_cfg.events, term):
            setattr(env_cfg.events, term, None)
    env_cfg.events.reset_base.params["pose_range"] = {}
    env_cfg.events.reset_base.params["velocity_range"] = {}
    env_cfg.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
    # keep a fall on screen
    env_cfg.terminations.base_orientation = None
    # commands are written every step; arrows as in the scenario videos
    command_cfg = env_cfg.commands.base_velocity
    command_cfg.heading_command = False
    command_cfg.rel_standing_envs = 0.0
    command_cfg.resampling_time_range = (1.0e6, 1.0e6)
    command_cfg.debug_vis = True
    command_cfg.goal_vel_visualizer_cfg.markers["arrow"].scale = (1.0, 0.1, 0.1)
    command_cfg.current_vel_visualizer_cfg.markers["arrow"].scale = (1.0, 0.1, 0.1)
    # follow camera
    env_cfg.viewer.origin_type = "asset_root"
    env_cfg.viewer.asset_name = "robot"
    env_cfg.viewer.env_index = 0
    env_cfg.viewer.eye = CAM_EYE
    env_cfg.viewer.lookat = CAM_LOOKAT

    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array")
    unwrapped = env.unwrapped
    device = unwrapped.device
    robot = unwrapped.scene["robot"]
    command_term = unwrapped.command_manager.get_term("base_velocity")
    base_id = robot.find_bodies("base")[0][0]
    set_torso_mass(robot, base_id, [args_cli.torso_mass_add])
    actor = build_actor(torch.load(args_cli.policy, map_location="cpu", weights_only=False)["model_state_dict"], device)

    add = args_cli.torso_mass_add
    if abs(add) < 1e-9:
        condition = f"Torso mass {TORSO_MASS_KG:.2f} kg, same as the training model (no dynamics gap)"
    else:
        condition = (f"Torso mass {TORSO_MASS_KG + add:.2f} kg vs {TORSO_MASS_KG:.2f} kg in training "
                     f"({add:+.2f} kg, {'heavier' if add > 0 else 'lighter'} than the model)")
    stat_lines = [s for s in args_cli.stat.split("|") if s]

    os.makedirs(args_cli.out_dir, exist_ok=True)
    fps = int(round(1.0 / unwrapped.step_dt))
    paths = {"caption": os.path.join(args_cli.out_dir, f"{args_cli.name}.mp4"),
             "nocaption": os.path.join(args_cli.out_dir, f"{args_cli.name}_nocaption.mp4")}
    if args_cli.graph:
        paths["graph"] = os.path.join(args_cli.out_dir, f"{args_cli.name}_graph.mp4")
    writers = {k: imageio.get_writer(p, fps=fps, codec="libx264", quality=8, macro_block_size=1) for k, p in paths.items()}

    history = {key: [] for key in ["t", "cmd_vx", "vx", "cmd_wz", "wz", "lin_err", "yaw_err"]}
    dt = unwrapped.step_dt
    fall_t = None
    with torch.inference_mode():
        obs = env.reset()[0]["policy"]
        k = 0
        while simulation_app.is_running():
            t = k * dt
            if t >= DURATION_S or (fall_t is not None and t >= fall_t + FALL_TAIL_S):
                break
            label, cmd = command_at(t)
            command_term.vel_command_b[:] = torch.tensor(cmd, device=device)
            obs[:, 0:3] = command_term.vel_command_b
            obs = env.step(actor(obs))[0]["policy"]
            k += 1
            vx, vy, wz, tilt = (v.item() for v in measured_velocity(robot))
            if fall_t is None and tilt > FALL_TILT_RAD:
                fall_t = k * dt
                print(f"[INFO] fell at t = {fall_t:.2f} s", flush=True)
            history["t"].append(k * dt)
            history["cmd_vx"].append(cmd[0])
            history["vx"].append(vx)
            history["cmd_wz"].append(cmd[2])
            history["wz"].append(wz)
            if fall_t is None:
                history["lin_err"].append(((vx - cmd[0]) ** 2 + (vy - cmd[1]) ** 2) ** 0.5)
                history["yaw_err"].append(abs(wz - cmd[2]))

            frame = env.render()
            writers["nocaption"].append_data(frame)
            if fall_t is None:
                status, color = ("Upright (no fall)" if k * dt < DURATION_S - 2 * dt
                                 else "Finished the scenario without falling", (140, 230, 140))
            else:
                status, color = f"FELL at t = {fall_t:.1f} s (torso tilt over 45 deg)", (255, 120, 120)
            captioned = caption_overlay(frame, args_cli.title, condition, k * dt, label, cmd, status, color, stat_lines)
            writers["caption"].append_data(captioned)
            if "graph" in writers:
                writers["graph"].append_data(graph_overlay(captioned, history))
    for w in writers.values():
        w.close()
    summary = f"fell at {fall_t:.1f} s" if fall_t is not None else "no fall"
    print(f"[RESULT] {args_cli.name}: {summary}, mean speed error "
          f"{sum(history['lin_err']) / max(1, len(history['lin_err'])):.3f} m/s, mean yaw error "
          f"{sum(history['yaw_err']) / max(1, len(history['yaw_err'])):.3f} rad/s -> {list(paths.values())}", flush=True)
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
