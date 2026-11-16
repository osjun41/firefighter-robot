"""Command-tracking evaluation in MuJoCo under deployment timing gaps (cross-check of sim/locomotion/scripts/s2r/eval_timing_gap.py).

Same trials as eval_tracking_mujoco.py (tracking_common.COMMANDS, one per seed, training-matched observation noise),
but the policy loop runs on the 0.5 ms physics steps so timing can be set finely:
  * action delay: a policy output reaches the PD targets d_a later,
  * observation delay: the policy sees the sensors of d_o earlier,
  * jitter: each call is late by a uniform 0..J (the 40 ms schedule does not drift),
  * skip: each call is missed with probability p (the targets hold for another period),
  * the BHL deployment loop: the sensors are read when the targets are sent (d_a after the call) and used at the next
    call.
With no delay the loop is the one of eval_tracking_mujoco.py (same results). Conditions: nominal, the action delay
sweep and the three scenarios of eval_timing_gap.py (the optimistic one with a 2 ms jitter).

Usage (from the repository root):
    uv run --project sim/locomotion sim/locomotion/scripts/sim2sim/eval_timing_mujoco.py --policy baseline --name Baseline
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))  # repository root: policies/policy_select.py
from policies import policy_select  # noqa: E402
import csv
import importlib
import os
import types

import numpy as np
import torch
import mujoco
import mujoco.viewer
from omegaconf import OmegaConf

from tracking_common import COMMANDS, FIELDS, command_at, num_steps, tilt_of, trial_metrics, yaw_of

parser = argparse.ArgumentParser(description="Command tracking in MuJoCo under deployment timing gaps.")
policy_select.add_argument(parser)
parser.add_argument("--name", type=str, required=True, help="Name of the result file.")
parser.add_argument("--out_dir", type=str, default="logs/timing_gap/mujoco", help="Output directory.")
parser.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2], help="Seeds of the noise and loop randomness.")
parser.add_argument("--conditions", type=str, nargs="+", default=None, help="Run only these conditions.")
args = parser.parse_args()

ACT_DELAYS_MS = [2, 5, 10, 15, 20, 30, 40, 60, 80]
# name -> timing (ms, probability), as in eval_timing_gap.py
CONDITIONS = {"nominal": {}}
CONDITIONS.update({f"act_delay:{ms}": {"act": ms} for ms in ACT_DELAYS_MS})
CONDITIONS.update({
    "scenario:optimistic": {"act": 5, "obs": 5, "jitter": 2},
    "scenario:typical": {"act": 10, "obs": 10, "jitter": 5},
    "scenario:bhl_loop": {"act": 10, "bhl": True, "jitter": 10, "skip": 0.02},
})


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


def run_trial(robot, controller, cmd, timing, rng):
    model, data, cfg = robot.mj_model, robot.mj_data, robot.cfg
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    torso = np.nonzero(model.body_weldid == base_id)[0]
    torso_mass = model.body_mass[torso] / model.body_mass[torso].sum()
    substeps = int(round(cfg.policy_dt / cfg.physics_dt))
    def sub(ms):
        return int(round(ms / 1000.0 / cfg.physics_dt))
    act_delay, obs_delay, jitter = sub(timing.get("act", 0)), sub(timing.get("obs", 0)), sub(timing.get("jitter", 0))
    skip, bhl = timing.get("skip", 0.0), timing.get("bhl", False)
    history = max(act_delay, obs_delay) + 1

    obs = robot.reset()
    controller.prev_actions[:] = 0.0
    sensors = np.repeat(data.sensordata[None].copy(), history, axis=0)
    sent_sensors = data.sensordata.copy()
    held = controller.default_joint_positions.copy()  # a zero action, until the first output arrives
    held_history = np.repeat(held[None], history, axis=0)
    next_call, send = int(rng.integers(0, jitter + 1)), -1
    pos, yaw, tilt, torque = [], [], [], []
    for s in range(num_steps(cfg.policy_dt) * substeps):
        sensors[s % history] = data.sensordata
        if s == next_call:
            vx, vy, wz = command_at(s * cfg.physics_dt, cmd)
            # MujocoSimulator halves velocity_y (gamepad scaling) before it reaches the policy, so pre-scale it
            robot.command_controller.commands.update(velocity_x=vx, velocity_y=vy / 0.5, velocity_yaw=wz)
            if s > 0:  # the first call uses the reset observation, as eval_tracking_mujoco.py does
                now = data.sensordata.copy()
                data.sensordata[:] = sent_sensors if bhl else sensors[(s - obs_delay) % history]
                obs = robot._get_observations()
                data.sensordata[:] = now
            if rng.uniform() >= skip:
                held = np.asarray(controller.update(obs.numpy()), dtype=np.float32)
                send = s + act_delay
            next_call = (s // substeps + 1) * substeps + int(rng.integers(0, jitter + 1))
        if bhl and s == send:
            sent_sensors = data.sensordata.copy()
        held_history[s % history] = held
        robot._apply_actions(torch.tensor(held_history[(s - act_delay) % history]))
        mujoco.mj_step(model, data)
        if (s + 1) % substeps == 0:
            pos.append(torso_mass @ data.xipos[torso])
            yaw.append(yaw_of(data.xquat[base_id]))
            tilt.append(tilt_of(data.xquat[base_id]))
            torque.append(data.actuator_force.copy())
            if tilt[-1] > np.pi / 2:
                break
    robot._get_observations()  # eval_tracking_mujoco.py reads one more (noisy) observation; keeps the noise in step
    return trial_metrics(pos, yaw, tilt, torque, cfg.policy_dt)


def main():
    cfg = OmegaConf.load(policy_select.config(args.policy))
    robot = MujocoSimulator(cfg, obs_noise=True)
    controller = RlController(cfg)
    controller.load_policy()
    names = args.conditions or list(CONDITIONS)
    os.makedirs(args.out_dir, exist_ok=True)
    path = os.path.join(args.out_dir, f"{args.name}.csv")
    fields = ["condition", "act_ms", "obs_ms", "jitter_ms", "skip", "bhl"] + FIELDS
    rows = []
    for condition in names:
        timing = CONDITIONS[condition]
        for seed in args.seeds:
            torch.manual_seed(seed)
            rng = np.random.default_rng(seed)
            for name, cmd in COMMANDS:
                row = {"condition": condition, "act_ms": timing.get("act", 0), "obs_ms": timing.get("obs", 0),
                       "jitter_ms": timing.get("jitter", 0), "skip": timing.get("skip", 0.0),
                       "bhl": int(timing.get("bhl", False)), "sim": "mujoco", "obs_noise": 1, "seed": seed,
                       "command": name, "cmd_vx": cmd[0], "cmd_vy": cmd[1], "cmd_wz": cmd[2]}
                row.update(run_trial(robot, controller, cmd, timing, rng))
                rows.append(row)
        done = [r for r in rows if r["condition"] == condition]
        up = [r for r in done if not r["fell"]]
        err = np.mean([np.hypot(r["vx"] - r["cmd_vx"], r["vy"] - r["cmd_vy"]) for r in up]) if up else float("nan")
        print(f"[INFO] {args.name} {condition}: falls {sum(r['fell'] for r in done)}/{len(done)}, "
              f"mean velocity error {err:.3f} m/s", flush=True)
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
    print(f"-> {path}")


if __name__ == "__main__":
    main()
