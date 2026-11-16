"""Evaluate walking policies when the deployment timing differs from the training timing (timing gap study).

Training assumes an ideal loop: the policy runs exactly every 40 ms (25 Hz) on the state measured at that instant, and
its joint targets act at once on all joints. Here the task runs with decimation 1 (one 5 ms physics step per env
step) and this script runs a deployment loop per environment, which the test plan varies:
  * action delay: a policy output reaches the PD targets d_a later (inference, CAN),
  * observation delay: the policy sees the state measured d_o earlier, for all sensors, for the IMU terms only (base
    angular velocity, projected gravity) or for the joint terms only (joint positions and velocities),
  * period: the loop runs every P instead of 40 ms,
  * jitter: each call is late by a uniform 0..J (the schedule does not drift),
  * skip: each call is missed with probability p, so the targets hold for another period,
  * the BHL deployment loop (berkeley_humanoid_lite_lowlevel/scripts/run_locomotion.py): the state is read when the
    targets are sent (d_a after the call) and used at the next call, so the observation is about one period old,
  * scenarios that combine these.
The velocity command and the last action in the observation are the controller's own and never late; the last
action is the policy's previous output, as in the real controller (rl_controller.py). Times are whole 5 ms physics
steps. With no delay and P = 40 ms the loop is the training one (--check compares it with the task's own loop).

Per environment it records what eval_actuator_gap.py does: fall and its time, return, tracking errors, power, joint
error to the PD target, torque saturation and torso sway, averaged over the 5 ms steps while upright, and the action
change and jerk per policy call.

Evaluate ONE policy per process (see eval_dynamics_gap.py), so this script takes exactly one --policy.

Usage (from the repository root):
    uv run --project sim/locomotion sim/locomotion/scripts/s2r/eval_timing_gap.py --headless --out_dir logs/timing_gap/eval \
        --policy s2r_dr000
"""

"""Launch Isaac Sim Simulator first."""

import argparse
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))  # repository root: policies/policy_select.py
from policies import policy_select  # noqa: E402

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Evaluate a policy under deployment timing gaps.")
policy_select.add_argument(parser)
parser.add_argument("--out_dir", type=str, default="logs/timing_gap/eval", help="Output directory.")
parser.add_argument("--plan", choices=["full", "nominal"], default="full", help="Test plan.")
parser.add_argument("--profiles", type=int, default=8, help="Command profiles per timing setting.")
parser.add_argument("--repeats", type=int, default=4, help="Repeats of each profile per setting.")
parser.add_argument("--duration", type=float, default=20.0, help="Test duration (s).")
parser.add_argument("--seed", type=int, default=0, help="Seed of the command profiles and the loop randomness.")
parser.add_argument("--task", type=str, default="Velocity-ASMR-Biped-S2R-DR000-v0", help="Task to run.")
parser.add_argument("--check", type=str, default=None, metavar="OUT.npy",
                    help="Nominal plan without observation noise; save the base xy at every policy call.")
parser.add_argument("--native", action="store_true",
                    help="With --check: run the task's own decimation instead of the 5 ms loop.")
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
import sys
import torch

import isaaclab.utils.math as math_utils
from isaaclab_tasks.utils import parse_env_cfg

import berkeley_humanoid_lite.tasks  # noqa: F401

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from video_common import FALL_TILT_RAD, build_actor  # noqa: E402

STEP_MS = 5
POLICY_MS = 40
SEGMENT_S = 5.0
DELAYS_MS = [5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80]
PERIODS_MS = [30, 35, 45, 50, 60, 80]
JITTERS_MS = [5, 10, 20]
SKIPS = [0.01, 0.05, 0.10]
# (name, timing): optimistic and typical delays, and the BHL loop (state read when the targets are sent)
SCENARIOS = [
    ("optimistic", {"act": 5, "obs": 5}),
    ("typical", {"act": 10, "obs": 10, "jitter": 5}),
    ("bhl_loop", {"act": 10, "bhl": True, "jitter": 10, "skip": 0.02}),
]
IMU_TERMS = ["base_ang_vel", "projected_gravity"]
JOINT_TERMS = ["joint_pos", "joint_vel"]


def timing(group, value, act=0, obs=0, obs_imu=None, obs_joint=None, period=POLICY_MS, jitter=0, skip=0.0, bhl=False):
    return {"group": group, "value": value, "act_ms": act, "obs_imu_ms": obs if obs_imu is None else obs_imu,
            "obs_joint_ms": obs if obs_joint is None else obs_joint, "period_ms": period, "jitter_ms": jitter,
            "skip": skip, "bhl": bhl}


def build_plan():
    """List of (timing, profile index) per environment."""
    settings = [timing("nominal", 0)]
    if args_cli.plan == "full":
        for ms in DELAYS_MS:
            settings += [timing("act_delay", ms, act=ms), timing("obs_delay", ms, obs=ms),
                         timing("obs_delay_imu", ms, obs_imu=ms, obs_joint=0),
                         timing("obs_delay_joint", ms, obs_imu=0, obs_joint=ms)]
        settings += [timing("period", ms, period=ms) for ms in PERIODS_MS]
        settings += [timing("jitter", ms, jitter=ms) for ms in JITTERS_MS]
        settings += [timing("skip", p, skip=p) for p in SKIPS]
        settings += [timing(f"scenario:{name}", 0, **kwargs) for name, kwargs in SCENARIOS]
    return [(setting, k) for setting in settings for _ in range(args_cli.repeats) for k in range(args_cli.profiles)]


def build_profiles(rng, num_segments):
    """Piecewise-constant (vx, vy, wz) commands within the training ranges; a fifth of the segments stand
    (the same bank as eval_actuator_gap.py)."""
    profiles = np.zeros((args_cli.profiles, num_segments, 3), dtype=np.float32)
    for k in range(args_cli.profiles):
        for s in range(num_segments):
            if rng.uniform() < 0.2:
                continue
            profiles[k, s] = [rng.uniform(-0.5, 0.5), rng.uniform(-0.25, 0.25), rng.uniform(-1.0, 1.0)]
    return profiles


def term_columns(observation_manager, names):
    """Column indices of the named terms in the policy observation."""
    columns, start = {}, 0
    for name, dims in zip(observation_manager.active_terms["policy"], observation_manager.group_obs_term_dim["policy"]):
        columns[name] = list(range(start, start + int(np.prod(dims))))
        start += int(np.prod(dims))
    return torch.tensor([c for name in names for c in columns[name]])


def main():
    name, path = args_cli.policy, str(policy_select.checkpoint(args_cli.policy))
    if not os.path.isfile(path):
        raise FileNotFoundError(f"checkpoint not found: {path}")
    if args_cli.check:
        args_cli.plan = "nominal"
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=1)
    plan = build_plan()
    num_envs = len(plan)
    env_cfg.scene.num_envs = num_envs
    env_cfg.seed = args_cli.seed
    env_cfg.episode_length_s = args_cli.duration + 1.0
    native = args_cli.check is not None and args_cli.native
    if not native:
        if round(env_cfg.sim.dt * 1000) != STEP_MS or round(env_cfg.sim.dt * env_cfg.decimation * 1000) != POLICY_MS:
            raise ValueError("expected a 5 ms physics step and a 40 ms policy step")
        env_cfg.decimation = 1
        env_cfg.sim.render_interval = 1
    if args_cli.check:
        env_cfg.observations.policy.enable_corruption = False
    # the reset starts from the nominal pose without a push; the actuators stay nominal
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
    om = unwrapped.observation_manager
    cmd_cols = term_columns(om, ["velocity_commands"]).to(device)
    act_cols = term_columns(om, ["actions"]).to(device)
    imu_cols = term_columns(om, IMU_TERMS).to(device)
    joint_cols = term_columns(om, JOINT_TERMS).to(device)
    effort_limit = torch.cat([a.effort_limit_sim.expand(num_envs, -1) for a in robot.actuators.values()], dim=1)
    joint_order = torch.cat([torch.arange(robot.num_joints)[a.joint_indices] if isinstance(a.joint_indices, slice)
                             else torch.as_tensor(a.joint_indices).cpu() for a in robot.actuators.values()])
    effort_limit = effort_limit[:, torch.argsort(joint_order).to(device)]

    # per-environment loop timing, in steps
    tick_ms = POLICY_MS if native else STEP_MS
    def steps_of(key):
        return torch.tensor([round(s[key] / tick_ms) for s, _ in plan], device=device)
    act_delay, obs_imu_delay, obs_joint_delay = steps_of("act_ms"), steps_of("obs_imu_ms"), steps_of("obs_joint_ms")
    period, jitter = steps_of("period_ms"), steps_of("jitter_ms")
    skip = torch.tensor([s["skip"] for s, _ in plan], device=device)
    bhl = torch.tensor([s["bhl"] for s, _ in plan], device=device)
    history = int(max(act_delay.max(), obs_imu_delay.max(), obs_joint_delay.max())) + 1

    dt = tick_ms / 1000.0
    steps = int(round(args_cli.duration / dt))
    num_segments = int(np.ceil(args_cli.duration / SEGMENT_S))
    profiles = torch.tensor(build_profiles(np.random.default_rng(args_cli.seed + 1), num_segments), device=device)
    profile_ids = torch.tensor([k for _, k in plan], device=device)
    generator = torch.Generator(device=device).manual_seed(args_cli.seed)
    arange = torch.arange(num_envs, device=device)
    os.makedirs(args_cli.out_dir, exist_ok=True)

    def late():
        """Uniform lateness 0..jitter (steps) of a call after its slot on the schedule."""
        return (torch.rand(num_envs, device=device, generator=generator) * (jitter + 1)).long().clamp(max=jitter)

    @torch.inference_mode()
    def evaluate(actor):
        fell = torch.zeros(num_envs, dtype=torch.bool, device=device)
        fall_time = torch.full((num_envs,), float("nan"), device=device)
        upright_steps = torch.zeros(num_envs, device=device)
        keys = ["return", "lin_err", "ang_err", "power_W", "joint_err_sq", "sat_frac", "rollpitch_sq", "yaw_dev_sq"]
        sums = {key: torch.zeros(num_envs, device=device) for key in keys}
        call_sums = {key: torch.zeros(num_envs, device=device) for key in ["action_change", "action_jerk", "calls", "skips"]}
        obs = env.reset()[0]["policy"]
        num_actions = act_cols.numel()
        obs_history = obs.unsqueeze(0).repeat(history, 1, 1)
        held_history = torch.zeros(history, num_envs, num_actions, device=device)
        sent_obs = obs.clone()  # BHL loop: the state read when the last targets were sent
        output = torch.zeros(num_envs, num_actions, device=device)  # the policy's last output (held targets)
        output_1 = torch.zeros_like(output)
        calls = torch.zeros(num_envs, dtype=torch.long, device=device)
        next_call = late()
        send_step = torch.full((num_envs,), -1, dtype=torch.long, device=device)
        check = []
        for k in range(steps):
            command = profiles[profile_ids, min(int(k * dt / SEGMENT_S), num_segments - 1)]
            command_term.vel_command_b[:] = command
            obs_history[k % history] = obs
            due = next_call == k
            if bool(due.any()):
                seen = obs.clone()
                seen[:, imu_cols] = obs_history[(k - obs_imu_delay) % history, arange][:, imu_cols]
                seen[:, joint_cols] = obs_history[(k - obs_joint_delay) % history, arange][:, joint_cols]
                seen = torch.where(bhl.unsqueeze(1), sent_obs, seen)
                seen[:, cmd_cols] = command
                seen[:, act_cols] = output
                actions = actor(seen)
                missed = torch.rand(num_envs, device=device, generator=generator) < skip
                call = due & ~missed
                alive = (~fell).float()
                change = torch.linalg.norm(actions - output, dim=-1)
                jerk = torch.linalg.norm(actions - 2 * output + output_1, dim=-1)
                call_sums["action_change"] += change * (call & (calls >= 1)).float() * alive
                call_sums["action_jerk"] += jerk * (call & (calls >= 2)).float() * alive
                call_sums["calls"] += (call & (calls >= 2)).float() * alive
                call_sums["skips"] += (due & missed).float() * alive
                output_1 = torch.where(call.unsqueeze(1), output, output_1)
                output = torch.where(call.unsqueeze(1), actions, output)
                calls += call.long()
                send_step = torch.where(call, k + act_delay, send_step)
                # the next slot of the fixed schedule (a call is never later than its period)
                next_slot = (torch.div(k, period, rounding_mode="floor") + 1) * period
                next_call = torch.where(due, next_slot + late(), next_call)
                if args_cli.check:
                    check.append(robot.data.root_pos_w[:, :2].cpu().numpy().copy())
            sent = bhl & (send_step == k)
            sent_obs[sent] = obs[sent]
            held_history[k % history] = output
            applied = held_history[(k - act_delay) % history, arange]
            obs, reward, terminated, _, _ = env.step(applied)
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
            ang_vel = math_utils.quat_rotate_inverse(math_utils.yaw_quat(robot.data.root_quat_w), robot.data.root_ang_vel_w)
            sums["rollpitch_sq"] += (ang_vel[:, 0] ** 2 + ang_vel[:, 1] ** 2) * alive
            sums["yaw_dev_sq"] += (ang_vel[:, 2] - command[:, 2]) ** 2 * alive
            sums["power_W"] += torch.sum(torch.abs(robot.data.applied_torque * robot.data.joint_vel), dim=-1) * alive
            joint_err = robot.data.joint_pos_target - robot.data.joint_pos
            sums["joint_err_sq"] += torch.mean(joint_err**2, dim=-1) * alive
            sums["sat_frac"] += (robot.data.computed_torque.abs() >= effort_limit * 0.999).float().mean(dim=-1) * alive
            upright_steps += alive
        means = {key: (value / upright_steps.clamp(min=1.0)).cpu() for key, value in sums.items() if key != "return"}
        means["sway_rms"] = torch.sqrt(means["rollpitch_sq"] + means["yaw_dev_sq"])
        means["rollpitch_rms"] = torch.sqrt(means.pop("rollpitch_sq"))
        means["yaw_dev_rms"] = torch.sqrt(means.pop("yaw_dev_sq"))
        means["joint_err_rms"] = torch.sqrt(means.pop("joint_err_sq"))
        means["action_change"] = (call_sums["action_change"] / call_sums["calls"].clamp(min=1.0)).cpu()
        means["action_jerk"] = (call_sums["action_jerk"] / call_sums["calls"].clamp(min=1.0)).cpu()
        means["skipped"] = call_sums["skips"].cpu()
        if args_cli.check:
            np.save(args_cli.check, np.stack(check))
        return fell.cpu(), fall_time.cpu(), sums["return"].cpu(), means

    checkpoint = torch.load(path, map_location="cpu", weights_only=False)
    actor = build_actor(checkpoint["model_state_dict"], device)
    fell, fall_time, returns, means = evaluate(actor)
    if args_cli.check:
        print(f"[INFO] check: saved the base xy at every call to {args_cli.check}, falls {int(fell.sum())}/{num_envs}")
        env.close()
        return

    metrics = ["lin_err", "ang_err", "power_W", "action_change", "action_jerk", "joint_err_rms", "sat_frac", "sway_rms",
               "rollpitch_rms", "yaw_dev_rms", "skipped"]
    path_out = os.path.join(args_cli.out_dir, f"{name}.csv")
    with open(path_out, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["policy", "checkpoint", "group", "value", "act_ms", "obs_imu_ms", "obs_joint_ms", "period_ms",
                         "jitter_ms", "skip", "bhl", "profile", "fell", "fall_time_s", "return", *metrics])
        for i, (setting, profile) in enumerate(plan):
            writer.writerow([name, path, setting["group"], f"{setting['value']:g}", setting["act_ms"], setting["obs_imu_ms"],
                             setting["obs_joint_ms"], setting["period_ms"], setting["jitter_ms"], f"{setting['skip']:g}",
                             int(setting["bhl"]), profile, int(fell[i]), f"{fall_time[i]:.3f}", f"{returns[i]:.3f}",
                             *[f"{means[m][i]:.4f}" for m in metrics]])
    nominal = torch.tensor([s["group"] == "nominal" for s, _ in plan])
    print(f"[INFO] {name}: {num_envs} envs, falls {int(fell.sum())} ({100 * fell.float().mean():.1f}%), "
          f"nominal: falls {int(fell[nominal].sum())}/{int(nominal.sum())}, "
          f"lin err {means['lin_err'][nominal].mean():.3f} m/s, return {returns[nominal].mean():.2f} -> {path_out}")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
