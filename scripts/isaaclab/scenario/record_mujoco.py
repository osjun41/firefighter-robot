"""Record the scripted scenario in MuJoCo (sim2sim), following one robot, with a deployment config.

The scenario (forward, 90 deg right turn, forward, left, backward) is defined in scenario_common.py.
It uses the same MujocoSimulator and RlController as play_mujoco.py, with the gamepad replaced by the
scenario and the viewer replaced by an offscreen renderer. Velocity arrows are drawn like the Isaac Lab ones.

Usage (from the repository root):
    uv run --project scripts/isaaclab scripts/isaaclab/scenario/record_mujoco.py --config configs/policy_latest.yaml \
        --out videos/scenario/mujoco_x.mp4 [--label NAME] [--no_caption] [--no_obs_noise]
"""

import argparse
import importlib
import os
import types

import imageio
import numpy as np
import torch
import mujoco
import mujoco.viewer
from omegaconf import OmegaConf

from scenario_common import ARROW_HEIGHT, CAM_EYE, CAM_LOOKAT, ScenarioRunner, overlay, yaw_from_quat_wxyz

parser = argparse.ArgumentParser(description="Record the scenario video in MuJoCo.")
parser.add_argument("--config", type=str, required=True, help="Path to the deployment config of the policy.")
parser.add_argument("--out", type=str, required=True, help="Output video path (.mp4).")
parser.add_argument("--label", type=str, default=None, help="Policy name in the caption (default: config name).")
parser.add_argument("--no_caption", action="store_true", help="Do not draw the text captions.")
parser.add_argument("--no_obs_noise", action="store_true", help="Feed noise-free observations to the policy.")
parser.add_argument("--seed", type=int, default=0, help="Seed of the observation noise.")
args = parser.parse_args()


class ScriptedCommands:
    """Stands in for Se2Gamepad; the scenario writes the commands."""

    def __init__(self, *args, **kwargs):
        self.commands = {"velocity_x": 0.0, "velocity_y": 0.0, "velocity_yaw": 0.0, "mode_switch": 0}

    def run(self):
        pass


# run headless and as fast as possible: no viewer window, no gamepad, no real-time sleep
mujoco.viewer.launch_passive = lambda model, data: types.SimpleNamespace(sync=lambda: None)
mujoco_env = importlib.import_module("berkeley_humanoid_lite.environments.mujoco")
mujoco_env.Se2Gamepad = ScriptedCommands
mujoco_env.time.sleep = lambda seconds: None

from berkeley_humanoid_lite.environments import MujocoSimulator  # noqa: E402
from berkeley_humanoid_lite_lowlevel.policy.rl_controller import RlController  # noqa: E402

WIDTH, HEIGHT = 1280, 720


def add_arrow(scene, start, vec, rgba, width=0.02):
    """Add an arrow from start along vec to the scene."""
    if scene.ngeom >= scene.maxgeom or np.linalg.norm(vec) < 1e-3:
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3), np.zeros(9), np.array(rgba, dtype=np.float32)
    )
    mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_ARROW, width, start, start + vec)
    scene.ngeom += 1


def arrow_vec(base_rot, vel_xy_base):
    """Base-frame xy velocity as a world-frame arrow, 1.5 m long per m/s like the Isaac Lab markers."""
    return base_rot @ np.array([vel_xy_base[0], vel_xy_base[1], 0.0]) * 1.5


def main():
    cfg = OmegaConf.load(args.config)
    label = args.label or os.path.splitext(os.path.basename(args.config))[0]
    obs_noise = not args.no_obs_noise
    # same seed, same motion: the captioned and caption-free videos match
    torch.manual_seed(args.seed)

    robot = MujocoSimulator(cfg, obs_noise=obs_noise)
    obs = robot.reset()
    controller = RlController(cfg)
    controller.load_policy()
    model, data = robot.mj_model, robot.mj_data

    model.vis.global_.offwidth, model.vis.global_.offheight = WIDTH, HEIGHT
    renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    # same view as record_isaaclab.py: eye and lookat offsets from the root position
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    view = np.array(CAM_LOOKAT) - np.array(CAM_EYE)
    camera.distance = float(np.linalg.norm(view))
    camera.azimuth = float(np.degrees(np.arctan2(view[1], view[0])))
    camera.elevation = float(np.degrees(np.arcsin(view[2] / camera.distance)))

    title = f"MuJoCo (sim2sim)  |  정책: {label}  |  관측 노이즈 {'있음' if obs_noise else '없음'}"
    runner = ScenarioRunner(cfg.policy_dt)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    writer = imageio.get_writer(
        args.out, fps=int(round(1.0 / cfg.policy_dt)), codec="libx264", quality=8, macro_block_size=1
    )

    fell = False
    while True:
        label_now, cmd, turn_info = runner.command(yaw_from_quat_wxyz(data.qpos[3:7]))
        if runner.done:
            break
        vx, vy, wz = cmd
        # MujocoSimulator halves velocity_y (gamepad scaling) before it reaches the policy, so pre-scale it
        robot.command_controller.commands.update(velocity_x=vx, velocity_y=vy / 0.5, velocity_yaw=wz)
        obs = robot.step(torch.tensor(controller.update(obs.numpy())))

        base_rot = data.xmat[base_id].reshape(3, 3)
        vel_base = base_rot.T @ data.qvel[0:3]
        arrow_start = data.xpos[base_id] + np.array([0.0, 0.0, ARROW_HEIGHT])
        camera.lookat[:] = data.xpos[base_id] + np.array(CAM_LOOKAT)
        renderer.update_scene(data, camera=camera)
        add_arrow(renderer.scene, arrow_start, arrow_vec(base_rot, (vx, vy)), (0.0, 1.0, 0.0, 1.0))
        add_arrow(renderer.scene, arrow_start, arrow_vec(base_rot, vel_base[:2]), (0.0, 0.0, 1.0, 1.0))
        frame = renderer.render().copy()
        if not args.no_caption:
            frame = overlay(frame, title, runner.t, label_now, cmd, turn_info, runner.note())
        writer.append_data(frame)

        tilt = np.degrees(np.arccos(np.clip(base_rot[2, 2], -1.0, 1.0)))
        if tilt > 60.0:
            fell = True
            print(f"[INFO] Robot fell at t={runner.t:.1f}s")
            break
    writer.close()
    print(f"[INFO] Turn result (deg, s, reached): {runner.turn_result}, fell: {fell}, video: {args.out}")


if __name__ == "__main__":
    main()
