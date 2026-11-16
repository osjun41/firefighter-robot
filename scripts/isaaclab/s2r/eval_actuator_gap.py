"""Evaluate walking policies when the robot's PD gains differ from the training gains (actuator gap study).

The policy acts as trained; only the simulated actuators change. Every environment gets its own joint stiffness (Kp),
damping (Kd) and torque limit, as factors on the nominal values, written to PhysX after the scene is built. The test
plan has
  * grid: Kp and Kd factors of all joints on a log grid (2^(k/2), k = -4..4, i.e. x0.25 .. x4), 9 x 9 settings,
  * group: Kp and Kd together on one actuator type only, the hip and knee joints (6512) or the ankle joints (5010),
  * kt: Kp, Kd and the torque limit of all joints together, as a torque constant error does (the motor driver's
    torque is off by one factor, whatever it commands),
  * random: independent Kp and Kd factors per joint, drawn log-uniformly from [1 / r, r] for r = 1.25, 1.5, 2, 3,
  * nominal.
The dynamics are the task's: the default task, course S2R DR000, has the nominal friction, masses and CoM and the
payload. Its other startup events are no-ops at alpha = 0; the gain and joint offset randomization and the reset
push are switched off. Each setting runs every command profile of a fixed bank (the same as eval_dynamics_gap.py:
piecewise-constant velocity commands within the training ranges), so results differ only by actuators and policy.

Per environment it records what eval_dynamics_gap.py does (fall and its time, return, linear and yaw velocity
tracking errors, mechanical power, action change per step, torso sway) and, for the actuators: the RMS joint
position error to the PD target, the fraction of joints whose PD torque is at the torque limit, and the action jerk
(norm of the second difference of the actions per step, a sign of oscillation).

Evaluate ONE policy per process (see eval_dynamics_gap.py), so this script takes exactly one --policy. When the GPU is
short of memory, run the plan in parts (--parts K --part i, one process each) and concatenate the part CSVs.

Usage (from the repository root):
    uv run --project scripts/isaaclab scripts/isaaclab/s2r/eval_actuator_gap.py --headless --out_dir logs/actuator_gap/eval \
        --policy DR000=logs/rsl_rl/asmr_s2r_dr000/<run>/model_5999.pt
"""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Evaluate a policy under PD gain gaps.")
parser.add_argument("--policy", type=str, required=True, metavar="NAME=CHECKPOINT",
                    help="Policy to evaluate (RSL-RL checkpoint).")
parser.add_argument("--out_dir", type=str, default="logs/actuator_gap/eval", help="Output directory.")
parser.add_argument("--plan", choices=["full", "quick", "nominal"], default="full",
                    help="Test plan: full, quick, or nominal (only the nominal actuators, for the quality metrics).")
parser.add_argument("--profiles", type=int, default=8, help="Command profiles per actuator setting.")
parser.add_argument("--repeats", type=int, default=2, help="Repeats of each profile in the grid and sweeps.")
parser.add_argument("--samples", type=int, default=96, help="Random gain vectors per ratio.")
parser.add_argument("--duration", type=float, default=20.0, help="Test duration (s).")
parser.add_argument("--seed", type=int, default=0, help="Seed of the test plan and command profiles.")
parser.add_argument("--parts", type=int, default=1, help="Split the plan into this many parts (GPU memory).")
parser.add_argument("--part", type=int, default=0, help="Part of the plan to run (writes <name>.part<i>.csv).")
parser.add_argument("--task", type=str, default="Velocity-ASMR-Biped-S2R-DR000-v0", help="Task to run.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import csv
import gymnasium as gym
import json
import numpy as np
import os
import re
import torch

import isaaclab.utils.math as math_utils
from isaaclab_tasks.utils import parse_env_cfg

import berkeley_humanoid_lite.tasks  # noqa: F401

NUM_JOINTS = 12
# joint groups by actuator type (course robot: 6512 on the hips and knees, 5010 on the ankles)
JOINT_GROUPS = {"legs": r".*_(hip_.*|knee_pitch)_joint", "ankles": r".*_ankle_.*_joint"}
SEGMENT_S = 5.0
FALL_TILT_RAD = 0.78


def scale_values(plan):
    if plan == "quick":
        return [0.25, 0.5, 1.0, 2.0, 4.0]
    return [float(2.0 ** (k / 2)) for k in range(-4, 5)]


def build_plan(rng):
    """List of (group, setting, profile index) per environment.

    A setting holds the scalar factors of the swept groups (kp, kd, effort, or the group name and its factor) or the
    per-joint factors of a random sample (kp_joint, kd_joint), and the ratio r of a random sample.
    """
    plan = []

    def add(group, setting, repeats):
        for _ in range(repeats):
            for k in range(args_cli.profiles):
                plan.append((group, setting, k))

    add("nominal", {}, args_cli.repeats)
    if args_cli.plan == "nominal":
        return plan
    values = scale_values(args_cli.plan)
    for kp in values:
        for kd in values:
            if kp != 1.0 or kd != 1.0:
                add("grid", {"kp": kp, "kd": kd}, args_cli.repeats)
    for value in values:
        if value == 1.0:
            continue
        for name in JOINT_GROUPS:
            add(f"group:{name}", {"joints": name, "kp": value, "kd": value}, args_cli.repeats)
        add("kt", {"kp": value, "kd": value, "effort": value}, args_cli.repeats)
    ratios = [1.5, 3.0] if args_cli.plan == "quick" else [1.25, 1.5, 2.0, 3.0]
    samples = 4 if args_cli.plan == "quick" else args_cli.samples
    for ratio in ratios:
        for _ in range(samples):
            factors = np.exp(rng.uniform(-np.log(ratio), np.log(ratio), size=(2, NUM_JOINTS)))
            add(f"random:{ratio:g}", {"ratio": ratio, "kp_joint": factors[0], "kd_joint": factors[1]}, 1)
    return plan


def build_profiles(rng, num_segments):
    """Piecewise-constant (vx, vy, wz) commands within the training ranges; a fifth of the segments stand."""
    profiles = np.zeros((args_cli.profiles, num_segments, 3), dtype=np.float32)
    for k in range(args_cli.profiles):
        for s in range(num_segments):
            if rng.uniform() < 0.2:
                continue
            profiles[k, s] = [rng.uniform(-0.5, 0.5), rng.uniform(-0.25, 0.25), rng.uniform(-1.0, 1.0)]
    return profiles


def build_actor(state_dict, device):
    layers, index = [], 0
    while f"actor.{index}.weight" in state_dict:
        weight = state_dict[f"actor.{index}.weight"]
        linear = torch.nn.Linear(weight.shape[1], weight.shape[0])
        linear.weight.data.copy_(weight)
        linear.bias.data.copy_(state_dict[f"actor.{index}.bias"])
        layers.append(linear)
        index += 2
        if f"actor.{index}.weight" in state_dict:
            layers.append(torch.nn.ELU())
    return torch.nn.Sequential(*layers).to(device).eval()


def gain_factors(plan, joint_names):
    """Per-environment Kp, Kd and torque limit factors (num_envs, num_joints), in the robot's joint order."""
    n = len(plan)
    kp = torch.ones(n, NUM_JOINTS)
    kd = torch.ones(n, NUM_JOINTS)
    effort = torch.ones(n, NUM_JOINTS)
    masks = {name: torch.tensor([re.fullmatch(pattern, joint) is not None for joint in joint_names])
             for name, pattern in JOINT_GROUPS.items()}
    for i, (_, setting, _) in enumerate(plan):
        if "kp_joint" in setting:
            kp[i] = torch.tensor(setting["kp_joint"], dtype=torch.float32)
            kd[i] = torch.tensor(setting["kd_joint"], dtype=torch.float32)
            continue
        joints = masks[setting["joints"]] if "joints" in setting else torch.ones(NUM_JOINTS, dtype=torch.bool)
        kp[i, joints] = setting.get("kp", 1.0)
        kd[i, joints] = setting.get("kd", 1.0)
        effort[i, joints] = setting.get("effort", 1.0)
    return kp, kd, effort


def apply_actuators(robot, kp_factor, kd_factor, effort_factor):
    """Scale every environment's PD gains and torque limits, in PhysX and in the actuator models (which report the
    PD torque). Returns the torque limits (num_envs, num_joints)."""
    device = robot.device
    effort_limit = torch.zeros(len(kp_factor), NUM_JOINTS, device=device)
    for actuator in robot.actuators.values():
        ids = actuator.joint_indices
        index = torch.arange(NUM_JOINTS)[ids] if isinstance(ids, slice) else torch.as_tensor(ids).cpu()
        stiffness = robot.data.default_joint_stiffness[:, index] * kp_factor[:, index].to(device)
        damping = robot.data.default_joint_damping[:, index] * kd_factor[:, index].to(device)
        limit = actuator.effort_limit_sim.clone() * effort_factor[:, index].to(device)
        actuator.stiffness[:] = stiffness
        actuator.damping[:] = damping
        actuator.effort_limit[:] = limit
        actuator.effort_limit_sim[:] = limit
        robot.write_joint_stiffness_to_sim(stiffness, joint_ids=ids)
        robot.write_joint_damping_to_sim(damping, joint_ids=ids)
        robot.write_joint_effort_limit_to_sim(limit, joint_ids=ids)
        effort_limit[:, index] = limit
    return effort_limit


def main():
    name, path = args_cli.policy.split("=", 1)
    if not os.path.isfile(path):
        raise FileNotFoundError(f"checkpoint not found: {path}")
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=1)
    rng = np.random.default_rng(args_cli.seed)
    plan = build_plan(rng)
    plan = plan[args_cli.part * len(plan) // args_cli.parts:(args_cli.part + 1) * len(plan) // args_cli.parts]
    num_envs = len(plan)
    env_cfg.scene.num_envs = num_envs
    env_cfg.seed = args_cli.seed
    env_cfg.episode_length_s = args_cli.duration + 1.0
    # actuators are set per environment below; the reset starts from the nominal pose without a push
    for term in ["add_all_joint_default_pos", "scale_all_actuator_torque_constant", "base_external_force_torque"]:
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

    env = gym.make(args_cli.task, cfg=env_cfg)
    unwrapped = env.unwrapped
    device = unwrapped.device
    robot = unwrapped.scene["robot"]
    command_term = unwrapped.command_manager.get_term("base_velocity")
    if robot.num_joints != NUM_JOINTS:
        raise ValueError(f"expected {NUM_JOINTS} joints, the robot has {robot.num_joints}")
    kp_factor, kd_factor, effort_factor = gain_factors(plan, robot.joint_names)
    effort_limit = apply_actuators(robot, kp_factor, kd_factor, effort_factor)

    dt = unwrapped.step_dt
    steps = int(round(args_cli.duration / dt))
    num_segments = int(np.ceil(args_cli.duration / SEGMENT_S))
    profiles = torch.tensor(build_profiles(np.random.default_rng(args_cli.seed + 1), num_segments), device=device)
    profile_ids = torch.tensor([k for _, _, k in plan], device=device)
    nominal = torch.tensor([group == "nominal" for group, _, _ in plan])
    os.makedirs(args_cli.out_dir, exist_ok=True)
    with open(os.path.join(args_cli.out_dir, "joint_names.json"), "w") as f:
        json.dump(robot.joint_names, f)

    @torch.inference_mode()
    def evaluate(actor):
        """Run every environment once; per-environment metrics (fall, means while upright)."""
        fell = torch.zeros(num_envs, dtype=torch.bool, device=device)
        fall_time = torch.full((num_envs,), float("nan"), device=device)
        upright_steps = torch.zeros(num_envs, device=device)
        keys = ["return", "lin_err", "ang_err", "power_W", "action_change", "action_jerk", "joint_err_sq", "sat_frac",
                "rollpitch_sq", "yaw_dev_sq"]
        sums = {key: torch.zeros(num_envs, device=device) for key in keys}
        obs = env.reset()[0]["policy"]
        # the gains survive the reset
        stiffness = robot.root_physx_view.get_dof_stiffnesses().to(device)
        expected = robot.data.default_joint_stiffness * kp_factor.to(device)
        if not torch.allclose(stiffness, expected, rtol=1e-4):
            raise RuntimeError("the PhysX joint stiffness differs from the planned gains after the reset")
        actions_1 = actions_2 = None
        for k in range(steps):
            command = profiles[profile_ids, min(int(k * dt / SEGMENT_S), num_segments - 1)]
            command_term.vel_command_b[:] = command
            obs[:, 0:3] = command
            actions = actor(obs)
            obs, reward, terminated, _, _ = env.step(actions)
            obs = obs["policy"]
            alive = (~fell).float()
            sums["return"] += reward * alive
            tilt = torch.acos(torch.clamp(-robot.data.projected_gravity_b[:, 2], -1.0, 1.0))
            fallen = (terminated | (tilt > FALL_TILT_RAD)) & ~fell
            fall_time[fallen] = (k + 1) * dt
            fell |= fallen
            alive = (~fell).float()
            # velocities as in the tracking rewards: linear in the yaw frame, yaw rate in the world frame
            lin_vel = math_utils.quat_rotate_inverse(math_utils.yaw_quat(robot.data.root_quat_w),
                                                     robot.data.root_lin_vel_w)[:, :2]
            sums["lin_err"] += torch.linalg.norm(lin_vel - command[:, :2], dim=-1) * alive
            sums["ang_err"] += torch.abs(robot.data.root_ang_vel_w[:, 2] - command[:, 2]) * alive
            # torso sway: roll / pitch rates (yaw frame) and the yaw rate minus the commanded turn
            ang_vel = math_utils.quat_rotate_inverse(math_utils.yaw_quat(robot.data.root_quat_w), robot.data.root_ang_vel_w)
            sums["rollpitch_sq"] += (ang_vel[:, 0] ** 2 + ang_vel[:, 1] ** 2) * alive
            sums["yaw_dev_sq"] += (ang_vel[:, 2] - command[:, 2]) ** 2 * alive
            sums["power_W"] += torch.sum(torch.abs(robot.data.applied_torque * robot.data.joint_vel), dim=-1) * alive
            # actuators: joint error to the PD target, joints at the torque limit
            joint_err = robot.data.joint_pos_target - robot.data.joint_pos
            sums["joint_err_sq"] += torch.mean(joint_err**2, dim=-1) * alive
            at_limit = robot.data.computed_torque.abs() >= effort_limit * 0.999
            sums["sat_frac"] += at_limit.float().mean(dim=-1) * alive
            if actions_1 is not None:
                sums["action_change"] += torch.linalg.norm(actions - actions_1, dim=-1) * alive
            if actions_2 is not None:
                sums["action_jerk"] += torch.linalg.norm(actions - 2 * actions_1 + actions_2, dim=-1) * alive
            actions_2, actions_1 = actions_1, actions.clone()
            upright_steps += alive
        means = {key: (value / upright_steps.clamp(min=1.0)).cpu() for key, value in sums.items() if key != "return"}
        # RMS angular rates (rad/s): roll-pitch, yaw deviation from the command, and all three (torso sway)
        means["sway_rms"] = torch.sqrt(means["rollpitch_sq"] + means["yaw_dev_sq"])
        means["rollpitch_rms"] = torch.sqrt(means.pop("rollpitch_sq"))
        means["yaw_dev_rms"] = torch.sqrt(means.pop("yaw_dev_sq"))
        means["joint_err_rms"] = torch.sqrt(means.pop("joint_err_sq"))
        return fell.cpu(), fall_time.cpu(), sums["return"].cpu(), means

    checkpoint = torch.load(path, map_location="cpu", weights_only=False)
    actor = build_actor(checkpoint["model_state_dict"], device)
    fell, fall_time, returns, means = evaluate(actor)

    metrics = ["lin_err", "ang_err", "power_W", "action_change", "action_jerk", "joint_err_rms", "sat_frac", "sway_rms",
               "rollpitch_rms", "yaw_dev_rms"]
    suffix = f".part{args_cli.part}" if args_cli.parts > 1 else ""
    path_out = os.path.join(args_cli.out_dir, f"{name}{suffix}.csv")
    with open(path_out, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["policy", "checkpoint", "group", "kp", "kd", "effort", "ratio", "profile", "fell", "fall_time_s",
                         "return", *metrics, "kp_joint", "kd_joint"])
        for i, (group, setting, profile) in enumerate(plan):
            random = "kp_joint" in setting
            scalars = ["", "", ""] if random else [f"{setting.get(key, 1.0):g}" for key in ["kp", "kd", "effort"]]
            writer.writerow([
                name, path, group, *scalars, f"{setting.get('ratio', 1.0):g}", profile, int(fell[i]),
                f"{fall_time[i].item():.2f}", f"{returns[i].item():.3f}",
                *[f"{means[key][i].item():.4f}" for key in metrics],
                ";".join(f"{v:.4f}" for v in kp_factor[i].tolist()), ";".join(f"{v:.4f}" for v in kd_factor[i].tolist()),
            ])
    print(f"[INFO] {name}: {num_envs} envs, falls {int(fell.sum())} ({100 * fell.float().mean():.1f}%), "
          f"nominal: falls {int(fell[nominal].sum())}/{int(nominal.sum())}, "
          f"lin err {means['lin_err'][nominal].mean():.3f} m/s, return {returns[nominal].mean():.2f} -> {path_out}",
          flush=True)

    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
