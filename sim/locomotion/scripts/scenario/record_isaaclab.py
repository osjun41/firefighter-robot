"""Record the scripted scenario in Isaac Lab, following one robot, with an exported ONNX policy.

The scenario (forward, 90 deg right turn, forward, left, backward) is defined in scenario_common.py.
The robot runs in nominal conditions (no randomization, observation noise or reset perturbation)
so the video can be compared with record_mujoco.py.

Usage (from the repository root):
    uv run --project sim/locomotion sim/locomotion/scripts/scenario/record_isaaclab.py --policy baseline \
        --out videos/scenario/isaaclab_x.mp4 --headless [--label NAME] [--no_caption]
"""

"""Launch Isaac Sim Simulator first."""

import argparse
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))  # repository root: policies/policy_select.py
from policies import policy_select  # noqa: E402
import onnxruntime  # noqa: F401,E402  Windows: load its DLLs before Isaac Sim starts, or the import fails afterwards
import os
import sys

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Record the scenario video in Isaac Lab.")
policy_select.add_argument(parser)
parser.add_argument("--out", type=str, required=True, help="Output video path (.mp4).")
parser.add_argument("--label", type=str, default=None, help="Policy name in the caption (default: config name).")
parser.add_argument("--no_caption", action="store_true", default=False, help="Do not draw the text captions.")
parser.add_argument(
    "--task", type=str, default="Velocity-ASMR-Biped-v0", help="Name of the task."
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
import numpy as np
import torch

import onnxruntime as ort
from omegaconf import OmegaConf

from isaaclab.envs.mdp.commands.velocity_command import UniformVelocityCommand
from isaaclab_tasks.utils import parse_env_cfg

# Import extensions to set up environment tasks
import berkeley_humanoid_lite.tasks  # noqa: F401

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scenario_common import (  # noqa: E402
    ARROW_HEIGHT,
    CAM_EYE,
    CAM_LOOKAT,
    ScenarioRunner,
    overlay,
    yaw_from_quat_wxyz,
)


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
    """Record the scenario in Isaac Lab."""
    deploy_cfg = OmegaConf.load(policy_select.config(args_cli.policy))
    label = args_cli.label or args_cli.policy

    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=1)
    # run the policy at the rate it was trained for
    env_cfg.decimation = round(deploy_cfg.policy_dt / env_cfg.sim.dt)
    env_cfg.sim.render_interval = env_cfg.decimation
    env_cfg.episode_length_s = 120.0

    # the command is written directly every step
    command_cfg = env_cfg.commands.base_velocity
    command_cfg.heading_command = False
    command_cfg.rel_standing_envs = 0.0
    command_cfg.resampling_time_range = (1.0e6, 1.0e6)
    # arrows sized to read like the MuJoCo ones (~0.45 m at 0.3 m/s); the default 0.5 x 0.5 x 0.5 hides the torso
    command_cfg.goal_vel_visualizer_cfg.markers["arrow"].scale = (1.0, 0.1, 0.1)
    command_cfg.current_vel_visualizer_cfg.markers["arrow"].scale = (1.0, 0.1, 0.1)

    # nominal conditions: no randomization, no observation noise, no reset perturbation
    events = env_cfg.events
    events.physics_material = None
    events.add_base_mass = None
    events.add_all_joint_default_pos = None
    events.scale_all_actuator_torque_constant = None
    events.base_external_force_torque = None
    events.reset_base.params["pose_range"] = {}
    events.reset_base.params["velocity_range"] = {}
    events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
    env_cfg.observations.policy.enable_corruption = False

    # follow the robot, same view as record_mujoco.py
    env_cfg.viewer.origin_type = "asset_root"
    env_cfg.viewer.asset_name = "robot"
    env_cfg.viewer.env_index = 0
    env_cfg.viewer.eye = CAM_EYE
    env_cfg.viewer.lookat = CAM_LOOKAT

    # create isaac environment
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array")
    unwrapped = env.unwrapped
    robot = unwrapped.scene["robot"]
    command_term = unwrapped.command_manager.get_term("base_velocity")

    session = ort.InferenceSession(deploy_cfg.policy_checkpoint_path, providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name
    print(f"[INFO]: Loading ONNX policy from: {deploy_cfg.policy_checkpoint_path}")

    title = f"Isaac Lab (학습 환경)  |  정책: {label}"
    runner = ScenarioRunner(unwrapped.step_dt)
    os.makedirs(os.path.dirname(os.path.abspath(args_cli.out)), exist_ok=True)
    writer = imageio.get_writer(
        args_cli.out, fps=int(round(1.0 / unwrapped.step_dt)), codec="libx264", quality=8, macro_block_size=1
    )

    obs, _ = env.reset()
    obs = obs["policy"]
    falls = 0
    while simulation_app.is_running():
        yaw = yaw_from_quat_wxyz(robot.data.root_quat_w[0].cpu().numpy())
        label_now, cmd, turn_info = runner.command(yaw)
        if runner.done:
            break
        with torch.inference_mode():
            command_term.vel_command_b[:] = torch.tensor(cmd, device=unwrapped.device)
            # the observation still holds the previous command; refresh it so the policy sees the new one
            obs[:, 0:3] = command_term.vel_command_b
            actions = session.run(None, {input_name: obs.cpu().numpy().astype(np.float32)})[0]
            obs, _, terminated, _, _ = env.step(torch.from_numpy(actions).to(unwrapped.device))
            obs = obs["policy"]
        if bool(terminated[0]):
            falls += 1
            print(f"[INFO] Robot fell at t={runner.t:.1f}s")
        frame = env.render()
        if not args_cli.no_caption:
            frame = overlay(frame, title, runner.t, label_now, cmd, turn_info, runner.note())
        writer.append_data(frame)
    writer.close()
    print(f"[INFO] Turn result (deg, s, reached): {runner.turn_result}, falls: {falls}, video: {args_cli.out}")

    # close the simulator
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
