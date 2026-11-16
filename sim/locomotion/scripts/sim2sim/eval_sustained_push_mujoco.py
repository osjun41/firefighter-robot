"""Sustained push evaluation in MuJoCo (sim2sim), mirroring sim/locomotion/scripts/push_test/eval_sustained_push.py.

The robot stands for SETTLE_S, then a horizontal force pushes it along its heading (default backward) for
--window seconds at a point --app_offset metres above the torso CoM, then the force is released for --post
seconds. One trial per force runs in sequence (CPU only). Uses the deployment stack of play_mujoco.py
(MujocoSimulator + RlController), with training-matched observation noise unless --no_obs_noise.

Controllers: the policy alone; a feed-forward forward-velocity command (--ff_gain); a policy that also
observes the push signal (detected from its input size); switching to --brace_config during the push.

Usage (from the repository root):
    uv run --project sim/locomotion sim/locomotion/scripts/sim2sim/eval_sustained_push_mujoco.py --policy baseline --name baseline \
        --forces 0 30 1
"""

import argparse
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))  # repository root: policies/policy_select.py
from policies import policy_select  # noqa: E402
import csv
import importlib
import os
import sys
import types

import numpy as np
import torch
import mujoco
import mujoco.viewer
from omegaconf import OmegaConf

parser = argparse.ArgumentParser(description="Sustained push evaluation in MuJoCo.")
policy_select.add_argument(parser)
parser.add_argument("--name", type=str, required=True, help="Name of the result file.")
parser.add_argument("--out_dir", type=str, default="logs/sustained_push_mujoco", help="Output directory.")
parser.add_argument("--forces", type=float, nargs=3, default=[0, 30, 1], metavar=("START", "STOP", "STEP"))
parser.add_argument("--direction", type=float, default=180.0, help="Push direction in the heading frame (deg).")
parser.add_argument("--app_offset", type=float, default=0.20, help="Push point above the torso CoM (m).")
parser.add_argument("--window", type=float, default=10.0, help="Push duration (s).")
parser.add_argument("--post", type=float, default=3.0, help="Time after the push is released (s).")
parser.add_argument("--ramp", type=float, default=0.1, help="Time for the force to build up or fall off (s).")
parser.add_argument("--lead", type=float, default=0.0, help="Push signal lead before the force onset (s).")
parser.add_argument("--ff_gain", type=float, default=0.0, help="Forward velocity command per newton (m/s/N).")
parser.add_argument("--brace_config", type=str, default=None, help="Policy to switch to while the signal is on.")
parser.add_argument("--brace_hold", type=float, default=1.0, help="Keep the brace policy this long after release (s).")
parser.add_argument("--no_obs_noise", action="store_true", help="Feed noise-free observations to the policy.")
parser.add_argument("--seed", type=int, default=0, help="Seed of the observation noise.")
args = parser.parse_args()


class ScriptedCommands:
    """Stands in for Se2Gamepad; the evaluation writes the commands."""

    def __init__(self, *args, **kwargs):
        self.commands = {"velocity_x": 0.0, "velocity_y": 0.0, "velocity_yaw": 0.0, "mode_switch": 0}

    def run(self):
        pass


# run headless and as fast as possible
mujoco.viewer.launch_passive = lambda model, data: types.SimpleNamespace(sync=lambda: None)
mujoco_env = importlib.import_module("berkeley_humanoid_lite.environments.mujoco")
mujoco_env.Se2Gamepad = ScriptedCommands
mujoco_env.time.sleep = lambda seconds: None

from berkeley_humanoid_lite.environments import MujocoSimulator  # noqa: E402
from berkeley_humanoid_lite_lowlevel.policy.rl_controller import RlController  # noqa: E402

SETTLE_S = 2.0
FALL_TILT_DEG = float(np.degrees(0.78))
HOLD_DISPLACEMENT_M = 0.10
# must match berkeley_humanoid_lite.tasks.locomotion.velocity.mdp.push.PUSH_SIGNAL_SCALE
PUSH_SIGNAL_SCALE = 30.0


class SignalController(RlController):
    """RlController that appends the push signal to the observation when the policy expects it."""

    def load_policy(self):
        super().load_policy()
        self.signal = 0.0
        self.uses_signal = self.policy.model.get_inputs()[0].shape[-1] == self.cfg.num_observations + 1

    def update(self, robot_observations):
        if not self.uses_signal:
            return super().update(robot_observations)
        # same observation as RlController.update (no history), with the signal appended
        n = self.cfg.num_actions
        command_velocity = robot_observations[7 + 2 * n + 1:7 + 2 * n + 4]
        projected_gravity = self.quat_rotate_inverse(robot_observations[0:4], self.gravity_vector)
        observations = np.concatenate([
            command_velocity,
            robot_observations[4:7],
            projected_gravity,
            robot_observations[7:7 + n] - self.default_joint_positions,
            robot_observations[7 + n:7 + 2 * n],
            self.prev_actions,
            [self.signal],
        ]).astype(np.float32)[None]
        actions = np.clip(self.policy.forward(observations)[0], self.cfg.action_limit_lower,
                          self.cfg.action_limit_upper)
        self.prev_actions[:] = actions
        return actions * self.cfg.action_scale + self.default_joint_positions


def run_trial(cfg, brace_cfg, force):
    robot = MujocoSimulator(cfg, obs_noise=not args.no_obs_noise)
    model, data = robot.mj_model, robot.mj_data
    obs = robot.reset()
    controllers = {"policy": SignalController(cfg)}
    if brace_cfg is not None:
        controllers["brace"] = SignalController(brace_cfg)
    for controller in controllers.values():
        controller.load_policy()
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    # the torso is every body welded to the base (IMU, payload); Isaac Lab merges the payload into the base
    torso = np.nonzero(model.body_weldid == base_id)[0]
    torso_mass = model.body_mass[torso] / model.body_mass[torso].sum()

    dt = cfg.policy_dt
    steps = int(round((SETTLE_S + args.window + args.post) / dt))
    direction = np.radians(args.direction)
    fell, fall_time, max_tilt, max_disp, start = False, float("nan"), 0.0, 0.0, None
    for k in range(steps):
        t_push = k * dt - SETTLE_S
        pushing = 0.0 <= t_push < args.window
        signal_on = -args.lead <= t_push < args.window
        ramp = min(max(t_push / args.ramp, 0.0), 1.0) * min(max((args.window - t_push) / args.ramp, 0.0), 1.0)

        # push along the heading at a point above the torso CoM (xfrc_applied acts at the base body's own CoM)
        rot = data.xmat[base_id].reshape(3, 3)
        yaw = np.arctan2(rot[1, 0], rot[0, 0])
        magnitude = 0.0 if fell else force * ramp
        force_w = magnitude * np.array([np.cos(yaw + direction), np.sin(yaw + direction), 0.0])
        point_w = torso_mass @ data.xipos[torso] + rot @ np.array([0.0, 0.0, args.app_offset])
        data.xfrc_applied[base_id, :3] = force_w
        data.xfrc_applied[base_id, 3:] = np.cross(point_w - data.xipos[base_id], force_w)
        if t_push >= 0.0 and start is None:
            start = data.xpos[base_id, :2].copy()

        vx = args.ff_gain * force if signal_on else 0.0
        # MujocoSimulator halves velocity_y before it reaches the policy; only x is used here
        robot.command_controller.commands.update(velocity_x=vx, velocity_y=0.0, velocity_yaw=0.0)
        use_brace = "brace" in controllers and -args.lead <= t_push < args.window + args.brace_hold
        controller = controllers["brace" if use_brace else "policy"]
        for c in controllers.values():
            c.signal = force * float(signal_on) / PUSH_SIGNAL_SCALE
        actions = controller.update(obs.numpy())
        # keep the other controller's action history in step with what is applied
        for c in controllers.values():
            c.prev_actions[:] = controller.prev_actions
        obs = robot.step(torch.tensor(actions))

        if t_push < 0.0 or fell:
            continue
        tilt = np.degrees(np.arccos(np.clip(data.xmat[base_id].reshape(3, 3)[2, 2], -1.0, 1.0)))
        max_tilt = max(max_tilt, tilt)
        max_disp = max(max_disp, float(np.linalg.norm(data.xpos[base_id, :2] - start)))
        if tilt > FALL_TILT_DEG:
            fell, fall_time = True, t_push + dt
    return fell, fall_time, max_tilt, max_disp


def main():
    torch.manual_seed(args.seed)
    cfg = OmegaConf.load(policy_select.config(args.policy))
    brace_cfg = OmegaConf.load(args.brace_config) if args.brace_config else None
    forces = [round(args.forces[0] + i * args.forces[2], 3)
              for i in range(int(round((args.forces[1] - args.forces[0]) / args.forces[2])) + 1)]
    os.makedirs(args.out_dir, exist_ok=True)
    path = os.path.join(args.out_dir, f"{args.name}.csv")
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["name", "force_N", "fell", "fall_time_s", "held", "max_displacement_m", "max_tilt_deg"])
        for force in forces:
            fell, fall_time, max_tilt, max_disp = run_trial(cfg, brace_cfg, force)
            held = not fell and max_disp <= HOLD_DISPLACEMENT_M
            writer.writerow([args.name, force, int(fell), f"{fall_time:.2f}", int(held), f"{max_disp:.3f}",
                             f"{max_tilt:.1f}"])
            print(f"[RESULT] {force:6.1f} N  {'FELL' if fell else 'held' if held else 'drifted'}  "
                  f"disp {max_disp:.2f} m  tilt {max_tilt:.1f} deg", flush=True)
    print(f"[INFO] Wrote {path}", flush=True)


if __name__ == "__main__":
    main()
    sys.stdout.flush()
    # MujocoSimulator leaves a thread behind
    os._exit(0)
