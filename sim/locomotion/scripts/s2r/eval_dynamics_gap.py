"""Evaluate walking policies when the test dynamics differ from the training dynamics.

Every environment gets its own dynamics, set directly in PhysX after the scene is built:
foot friction, torso mass, leg link mass (inertia scaled with it) and torso CoM (x, z). The test plan has
  * one sweep per parameter, the others at their nominal values, and
  * random combinations: for each gap size beta, parameter vectors drawn uniformly from the box the training
    randomization of width alpha = beta would use (see config/biped_s2r/env_cfg.py).
Our robot's condition is tested too: the upper-body payload (config/biped/payload.py) on the nominal dynamics,
and on the same random combinations as above. A task that has a payload event (add_payload, e.g. the course robot's
tasks) has the payload in its nominal robot: every environment gets it on top of its dynamics, and the separate
payload groups are left out.
Each dynamics setting runs every command profile of a fixed bank (piecewise-constant velocity commands within
the training ranges, the same for every policy and setting), so results differ only by dynamics and policy.
Actuation, reset state and the observation noise are as in training; there is no reset push.

Per environment it records: fall (base tilt past the training termination) and its time, the training reward
summed while upright, and means while upright of the linear and yaw velocity tracking errors (frames as in the
tracking rewards), mechanical power, the action change per step and the torso sway (RMS of the roll and pitch
rates and of the yaw rate minus the commanded turn rate).

Isaac Lab runs one simulation per process; when the GPU is short of memory, run the plan in parts
(--parts K --part i, one process each) and concatenate the part CSVs.

Evaluate ONE policy per process (repeat the command per policy): on the course robot, a policy evaluated after another
one in the same process falls at the first step on high foot friction (mu >= 1.2), as if contact state survived the
reset; the first policy of a process is unaffected (Berkeley Humanoid Lite: negligible).

Usage (from the repository root):
    uv run --project sim/locomotion sim/locomotion/scripts/s2r/eval_dynamics_gap.py --headless --out_dir logs/s2r_study/eval \
        --policy s2r_dr000 --policy s2r_dr100
"""

"""Launch Isaac Sim Simulator first."""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))  # repository root: policies/policy_select.py
from policies import policy_select  # noqa: E402

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Evaluate policies under sim-to-real dynamics gaps.")
policy_select.add_argument(parser, multiple=True)
parser.add_argument("--out_dir", type=str, default="logs/s2r_study/eval", help="Output directory.")
parser.add_argument("--plan", choices=["full", "quick", "nominal"], default="full",
                    help="Test plan: full, quick, or nominal (only the nominal dynamics, for the quality metrics).")
parser.add_argument("--profiles", type=int, default=8, help="Command profiles per dynamics setting.")
parser.add_argument("--repeats", type=int, default=2, help="Repeats of each profile in the sweeps.")
parser.add_argument("--samples", type=int, default=48, help="Random dynamics vectors per gap size.")
parser.add_argument("--payload_repeats", type=int, default=8, help="Repeats of each profile with the payload alone.")
parser.add_argument("--duration", type=float, default=20.0, help="Test duration (s).")
parser.add_argument("--seed", type=int, default=0, help="Seed of the test plan and command profiles.")
parser.add_argument("--parts", type=int, default=1, help="Split the plan into this many parts (GPU memory).")
parser.add_argument("--part", type=int, default=0, help="Part of the plan to run (writes <name>.part<i>.csv).")
parser.add_argument(
    "--task", type=str, default="Velocity-ASMR-Biped-S2R-DR000-v0", help="Task to run."
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
import numpy as np
import os
import torch

import isaaclab.utils.math as math_utils
from isaaclab_tasks.utils import parse_env_cfg

import berkeley_humanoid_lite.tasks  # noqa: F401
from berkeley_humanoid_lite.tasks.locomotion.velocity.config.biped.payload import (
    PAYLOAD_MASS_KG,
    PAYLOAD_POSITION_M,
    PAYLOAD_SIZE_M,
)
from berkeley_humanoid_lite.tasks.locomotion.velocity.config.biped_s2r.env_cfg import (
    NOMINAL_FRICTION,
    dynamics_ranges,
)
from berkeley_humanoid_lite.tasks.locomotion.velocity.mdp.dynamics import box_inertia, combine_with_payload

PARAMS = ["friction", "torso_mass_add", "leg_mass_scale", "torso_com_x", "torso_com_z"]
NOMINAL = {"friction": NOMINAL_FRICTION, "torso_mass_add": 0.0, "leg_mass_scale": 1.0,
           "torso_com_x": 0.0, "torso_com_z": 0.0}
# physical floors for large gaps (torso mass 3.16 kg)
LIMITS = {"friction": (0.05, None), "torso_mass_add": (-2.5, None), "leg_mass_scale": (0.3, None)}
SEGMENT_S = 5.0
FALL_TILT_RAD = 0.78


def sweep_values(plan):
    if plan == "quick":
        return {"friction": [0.2, 0.8, 1.6], "torso_mass_add": [-1.0, 0.0, 3.0], "leg_mass_scale": [0.7, 1.0, 1.3],
                "torso_com_x": [-0.06, 0.0, 0.06], "torso_com_z": [0.0, 0.06]}
    return {
        "friction": [0.1, 0.15, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0, 1.2, 1.6, 2.0],
        "torso_mass_add": [-2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0],
        "leg_mass_scale": [0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6],
        "torso_com_x": [-0.10, -0.08, -0.06, -0.04, -0.02, 0.0, 0.02, 0.04, 0.06, 0.08, 0.10],
        "torso_com_z": [-0.06, -0.03, 0.0, 0.03, 0.06, 0.09, 0.12],
    }


def build_plan(rng, payload_groups=True):
    """List of (group, dynamics dict, profile index) per environment."""
    plan = []
    if args_cli.plan == "nominal":
        for _ in range(args_cli.repeats):
            for k in range(args_cli.profiles):
                plan.append(("sweep:nominal", dict(NOMINAL), k))
        return plan
    for name, values in sweep_values(args_cli.plan).items():
        for value in values:
            for _ in range(args_cli.repeats):
                for k in range(args_cli.profiles):
                    plan.append((f"sweep:{name}", dict(NOMINAL, **{name: value}), k))
    gap_sizes = [1.0, 2.0] if args_cli.plan == "quick" else [0.5, 1.0, 1.5, 2.0, 3.0]
    samples = 4 if args_cli.plan == "quick" else args_cli.samples
    combined = []
    for beta in gap_sizes:
        r = dynamics_ranges(beta)
        boxes = {"friction": r["friction"], "torso_mass_add": r["torso_mass_add"], "leg_mass_scale": r["leg_mass_scale"],
                 "torso_com_x": r["torso_com"], "torso_com_z": r["torso_com"]}
        for _ in range(samples):
            dyn = {name: float(rng.uniform(*boxes[name])) for name in PARAMS}
            for name, (low, high) in LIMITS.items():
                dyn[name] = max(dyn[name], low)
            combined.append((beta, dyn))
            for k in range(args_cli.profiles):
                plan.append((f"combined:{beta:g}", dyn, k))
    if not payload_groups:
        return plan
    # our robot: the upper-body payload on the nominal dynamics, and on the same gap samples as above (paired)
    for _ in range(args_cli.payload_repeats):
        for k in range(args_cli.profiles):
            plan.append(("payload", dict(NOMINAL, payload_kg=PAYLOAD_MASS_KG), k))
    for beta, dyn in combined:
        for k in range(args_cli.profiles):
            plan.append((f"payload_combined:{beta:g}", dict(dyn, payload_kg=PAYLOAD_MASS_KG), k))
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


def apply_dynamics(robot, plan, base_id, leg_ids, nominal_payload=None):
    """Set friction, masses, inertias and CoMs of every environment (PhysX views work on the CPU).

    The payload of a plan entry (payload_kg, config/biped/payload.py), or nominal_payload (mass, position, size) on
    every environment. Returns the payload mass of each environment.
    """
    view = robot.root_physx_view
    n = len(plan)
    ids = torch.arange(n)
    values = {name: torch.tensor([dyn[name] for _, dyn, _ in plan], dtype=torch.float32) for name in PARAMS}

    materials = view.get_material_properties()
    materials[:, :, 0] = values["friction"][:, None]
    materials[:, :, 1] = values["friction"][:, None]
    view.set_material_properties(materials, ids)

    default_mass = robot.data.default_mass.cpu()
    masses = default_mass.clone()
    masses[:, base_id] += values["torso_mass_add"]
    masses[:, leg_ids] *= values["leg_mass_scale"][:, None]
    view.set_masses(masses, ids)
    # inertia scales with mass (uniform density)
    inertias = robot.data.default_inertia.cpu() * (masses / default_mass)[:, :, None]
    view.set_inertias(inertias, ids)

    coms = view.get_coms()
    coms[:, base_id, 0] += values["torso_com_x"]
    coms[:, base_id, 2] += values["torso_com_z"]
    view.set_coms(coms, ids)

    # upper-body payload on top of the (changed) torso, as the payload event does in training
    if nominal_payload is not None:
        payload_kg = torch.full((n,), float(nominal_payload["mass"]))
        position, size = nominal_payload["position"], nominal_payload["size"]
        given_inertia = nominal_payload.get("inertia")  # several rigid parts (F3): their summed inertia
    else:
        given_inertia = None
        payload_kg = torch.tensor([dyn.get("payload_kg", 0.0) for _, dyn, _ in plan], dtype=torch.float32)
        position, size = PAYLOAD_POSITION_M, PAYLOAD_SIZE_M
    with_payload = torch.nonzero(payload_kg > 0).flatten()
    if len(with_payload) > 0:
        m = len(with_payload)
        new_mass, new_com, new_inertia = combine_with_payload(
            masses[with_payload, base_id], coms[with_payload, base_id, :3],
            inertias[with_payload, base_id].reshape(m, 3, 3), payload_kg[with_payload],
            torch.tensor(position, dtype=torch.float32).expand(m, 3),
            box_inertia(payload_kg[with_payload], size) if given_inertia is None
            else torch.tensor(given_inertia, dtype=torch.float32).expand(m, 3, 3),
        )
        masses[with_payload, base_id] = new_mass
        coms[with_payload, base_id, :3] = new_com
        inertias[with_payload, base_id] = new_inertia.reshape(m, 9)
        view.set_masses(masses, ids)
        view.set_coms(coms, ids)
        view.set_inertias(inertias, ids)
    return payload_kg


def main():
    for spec in policy_select.chosen(args_cli.policy):
        policy_select.checkpoint(spec)  # fail early if the policy or its checkpoint is missing
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=1)
    # a payload event makes the payload part of the nominal robot; it is applied below with the dynamics
    payload_term = getattr(env_cfg.events, "add_payload", None)
    nominal_payload = dict(payload_term.params) if payload_term is not None else None
    if payload_term is not None:
        env_cfg.events.add_payload = None
    rng = np.random.default_rng(args_cli.seed)
    plan = build_plan(rng, payload_groups=nominal_payload is None)
    plan = plan[args_cli.part * len(plan) // args_cli.parts:(args_cli.part + 1) * len(plan) // args_cli.parts]
    num_envs = len(plan)
    env_cfg.scene.num_envs = num_envs
    env_cfg.seed = args_cli.seed
    env_cfg.episode_length_s = args_cli.duration + 1.0
    # dynamics are set per environment below; the reset starts from the nominal pose without a push
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

    env = gym.make(args_cli.task, cfg=env_cfg)
    unwrapped = env.unwrapped
    device = unwrapped.device
    robot = unwrapped.scene["robot"]
    command_term = unwrapped.command_manager.get_term("base_velocity")
    base_id = robot.find_bodies("base")[0][0]
    leg_ids = robot.find_bodies("leg_.*")[0]
    payload_kg = apply_dynamics(robot, plan, base_id, leg_ids, nominal_payload)
    payload_envs = [i for i, (group, _, _) in enumerate(plan) if group == "payload" or (
        nominal_payload is not None and all(plan[i][1][p] == NOMINAL[p] for p in PARAMS if p != "friction"))]
    if payload_envs:
        i = payload_envs[0]
        view = robot.root_physx_view
        print(f"[INFO] torso with payload: mass {view.get_masses()[i, base_id]:.3f} kg, "
              f"CoM {view.get_coms()[i, base_id, :3].numpy().round(4)}, "
              f"inertia diag {view.get_inertias()[i, base_id].reshape(3, 3).diagonal().numpy().round(5)}", flush=True)

    dt = unwrapped.step_dt
    steps = int(round(args_cli.duration / dt))
    num_segments = int(np.ceil(args_cli.duration / SEGMENT_S))
    profiles = torch.tensor(build_profiles(np.random.default_rng(args_cli.seed + 1), num_segments), device=device)
    profile_ids = torch.tensor([k for _, _, k in plan], device=device)
    nominal = torch.tensor([all(dyn[p] == NOMINAL[p] for p in PARAMS) and "payload_kg" not in dyn
                            for _, dyn, _ in plan], device=device)
    os.makedirs(args_cli.out_dir, exist_ok=True)

    @torch.inference_mode()
    def evaluate(actor):
        """Run every environment once; per-environment metrics (fall, means while upright)."""
        fell = torch.zeros(num_envs, dtype=torch.bool, device=device)
        fall_time = torch.full((num_envs,), float("nan"), device=device)
        upright_steps = torch.zeros(num_envs, device=device)
        sums = {key: torch.zeros(num_envs, device=device)
                for key in ["return", "lin_err", "ang_err", "power_W", "action_change", "rollpitch_sq", "yaw_dev_sq"]}
        obs = env.reset()[0]["policy"]
        last_actions = None
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
            if last_actions is not None:
                sums["action_change"] += torch.linalg.norm(actions - last_actions, dim=-1) * alive
            last_actions = actions.clone()
            upright_steps += alive
        means = {key: (value / upright_steps.clamp(min=1.0)).cpu() for key, value in sums.items() if key != "return"}
        # RMS angular rates (rad/s): roll-pitch, yaw deviation from the command, and all three (torso sway)
        means["sway_rms"] = torch.sqrt(means["rollpitch_sq"] + means["yaw_dev_sq"])
        means["rollpitch_rms"] = torch.sqrt(means.pop("rollpitch_sq"))
        means["yaw_dev_rms"] = torch.sqrt(means.pop("yaw_dev_sq"))
        return fell.cpu(), fall_time.cpu(), sums["return"].cpu(), means

    for spec in policy_select.chosen(args_cli.policy):
        name, path = spec, str(policy_select.checkpoint(spec))
        checkpoint = torch.load(path, map_location="cpu", weights_only=False)
        actor = build_actor(checkpoint["model_state_dict"], device)
        fell, fall_time, returns, means = evaluate(actor)

        suffix = f".part{args_cli.part}" if args_cli.parts > 1 else ""
        path_out = os.path.join(args_cli.out_dir, f"{name}{suffix}.csv")
        with open(path_out, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["policy", "checkpoint", "group", *PARAMS, "payload_kg", "profile", "fell", "fall_time_s", "return",
                             "lin_err", "ang_err", "power_W", "action_change", "sway_rms", "rollpitch_rms", "yaw_dev_rms"])
            for i, (group, dyn, profile) in enumerate(plan):
                writer.writerow([
                    name, path, group, *[f"{dyn[p]:.4f}" for p in PARAMS], f"{payload_kg[i].item():g}", profile, int(fell[i]),
                    f"{fall_time[i].item():.2f}", f"{returns[i].item():.3f}",
                    *[f"{means[key][i].item():.4f}" for key in ["lin_err", "ang_err", "power_W", "action_change",
                                                                  "sway_rms", "rollpitch_rms", "yaw_dev_rms"]],
                ])
        nominal_cpu = nominal.cpu()
        print(f"[INFO] {name}: {num_envs} envs, falls {int(fell.sum())} ({100 * fell.float().mean():.1f}%), "
              f"nominal: falls {int(fell[nominal_cpu].sum())}/{int(nominal_cpu.sum())}, "
              f"lin err {means['lin_err'][nominal_cpu].mean():.3f} m/s, return {returns[nominal_cpu].mean():.2f} "
              f"-> {path_out}", flush=True)

    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
