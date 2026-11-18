"""Record one standing push trial in Isaac Lab, following the robot, with a red arrow for the push.

The robot stands for SETTLE_S, then a sustained horizontal force is applied at the torso (base) CoM for
WINDOW_S, as in run_push_test.py (same nominal conditions). The fall termination is disabled so the fall
stays on screen; the push stops when the robot falls and the video ends FALL_TAIL_S later. The camera is
the same as in the scenario videos (sim/locomotion/scripts/scenario). Captions are in English.

Usage (from the repository root):
    uv run --project sim/locomotion sim/locomotion/scripts/push_test/record_push.py --policy baseline --direction 0 --force 12 \
        --case "Withstands in place" --out videos/push/x.mp4 [--out_nocaption videos/push/x_nocaption.mp4] --headless
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
parser = argparse.ArgumentParser(description="Record a standing push trial in Isaac Lab.")
policy_select.add_argument(parser)
parser.add_argument("--direction", type=float, required=True, help="Push direction in deg (0 = toward the front).")
parser.add_argument("--force", type=float, required=True, help="Sustained push force in N.")
parser.add_argument("--case", type=str, default="", help="Case name shown in the caption.")
parser.add_argument("--label", type=str, default=None, help="Policy name in the caption (default: config name).")
parser.add_argument("--out", type=str, required=True, help="Output video path with captions (.mp4).")
parser.add_argument("--out_nocaption", type=str, default=None, help="Also write the same video without captions.")
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
from PIL import Image, ImageDraw, ImageFont

from omegaconf import OmegaConf

import isaaclab.utils.math as math_utils
from isaaclab.markers import VisualizationMarkers
from isaaclab.markers.config import RED_ARROW_X_MARKER_CFG

SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(SCRIPTS_DIR, "push_test"))
sys.path.insert(0, os.path.join(SCRIPTS_DIR, "scenario"))
from push_common import (  # noqa: E402
    DIRECTION_NAMES,
    FALL_TILT_DEG,
    SETTLE_S,
    STEP_AIR_TIME_S,
    WINDOW_S,
    OnnxPolicy,
    horizontal_force_w,
    make_standing_env_cfg,
    tilt_deg,
    world_to_body,
)
from scenario_common import CAM_EYE, CAM_LOOKAT  # noqa: E402

# keep recording this long after a fall
FALL_TAIL_S = 1.5
# push arrow: length per newton; the arrow mesh spans x in [-0.25, 0.75] (tip) and is 1.0 wide at scale 1
ARROW_M_PER_N = 0.035
ARROW_MESH_LENGTH = 1.0
ARROW_MESH_TIP = 0.75
ARROW_THICKNESS = 0.1

# DejaVu on Linux, Arial on Windows and macOS
FONT = next((p for p in ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", r"C:\Windows\Fonts\arial.ttf", "/Library/Fonts/Arial.ttf") if os.path.isfile(p)), "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
FONT_BOLD = next((p for p in ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", r"C:\Windows\Fonts\arialbd.ttf", "/Library/Fonts/Arial Bold.ttf") if os.path.isfile(p)), "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf")
COLOR_STATUS = {"standing": (230, 230, 230), "holding": (130, 220, 140), "stepping": (255, 205, 110),
                "fell": (255, 120, 120)}
STATUS_TEXT = {"standing": "Standing still (no push yet)", "holding": "Holding in place",
               "stepping": "Stepping to hold", "fell": f"FELL (tilt > {FALL_TILT_DEG:.0f} deg)"}


def overlay(frame, lines, status):
    """Draw the captions on an RGB frame and return the new frame."""
    img = Image.fromarray(frame).convert("RGBA")
    layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(layer)
    W, H = img.size
    fonts = [ImageFont.truetype(FONT, 24), ImageFont.truetype(FONT_BOLD, 30), ImageFont.truetype(FONT, 22),
             ImageFont.truetype(FONT, 22), ImageFont.truetype(FONT_BOLD, 26)]
    colors = [(215, 215, 215), (255, 255, 255), (235, 235, 235), (235, 235, 235), COLOR_STATUS[status]]
    texts = lines + [STATUS_TEXT[status]]
    h = 18 + sum(f.size + 10 for f in fonts)
    draw.rounded_rectangle((16, 14, W - 16, 14 + h), radius=12, fill=(0, 0, 0, 150))
    y = 24
    for text, font, color in zip(texts, fonts, colors):
        draw.text((32, y), text, font=font, fill=color)
        y += font.size + 10
    # bottom left: legend
    font = ImageFont.truetype(FONT, 22)
    top = H - 16 - (font.size + 24)
    draw.rounded_rectangle((16, top, 560, H - 16), radius=12, fill=(0, 0, 0, 150))
    draw.rectangle((32, top + 17, 54, top + 31), fill=(230, 40, 40, 255))
    draw.text((66, top + 11), "Red arrow: applied push (length ~ force)", font=font, fill=(240, 240, 240))
    return np.array(Image.alpha_composite(img, layer).convert("RGB"))


def main():
    """Record the push trial."""
    deploy_cfg = OmegaConf.load(policy_select.config(args_cli.policy))
    label = args_cli.label or args_cli.policy
    direction_name = DIRECTION_NAMES.get(int(args_cli.direction) % 360, f"{args_cli.direction:g} deg")

    env_cfg = make_standing_env_cfg(args_cli.task, args_cli.device, 1, deploy_cfg.policy_dt, terminate_on_fall=False)
    # follow the robot, same view as the scenario videos
    env_cfg.viewer.origin_type = "asset_root"
    env_cfg.viewer.asset_name = "robot"
    env_cfg.viewer.env_index = 0
    env_cfg.viewer.eye = CAM_EYE
    env_cfg.viewer.lookat = CAM_LOOKAT

    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array")
    unwrapped = env.unwrapped
    device = unwrapped.device
    robot = unwrapped.scene["robot"]
    contact = unwrapped.scene["contact_forces"]
    base_id = robot.find_bodies("base")[0][0]
    feet_ids = contact.find_bodies(".*_ankle_roll")[0]
    command_term = unwrapped.command_manager.get_term("base_velocity")
    policy = OnnxPolicy(deploy_cfg.policy_checkpoint_path)

    arrow_cfg = RED_ARROW_X_MARKER_CFG.replace(prim_path="/Visuals/push_arrow")
    # unit prototype, so the per-frame scale below is the whole scale
    arrow_cfg.markers["arrow"].scale = (1.0, 1.0, 1.0)
    arrow = VisualizationMarkers(arrow_cfg)
    force_w = horizontal_force_w(
        torch.tensor([args_cli.direction], device=device), torch.tensor([args_cli.force], device=device)
    )
    push_dir = force_w / max(args_cli.force, 1e-6)
    arrow_len = args_cli.force * ARROW_M_PER_N
    arrow_quat = math_utils.quat_from_euler_xyz(
        torch.zeros(1, device=device), torch.zeros(1, device=device),
        torch.deg2rad(torch.tensor([args_cli.direction], device=device)),
    )
    arrow_scale = torch.tensor([[arrow_len / ARROW_MESH_LENGTH, ARROW_THICKNESS, ARROW_THICKNESS]], device=device)

    fps = int(round(1.0 / unwrapped.step_dt))
    writers = []
    for path in [args_cli.out, args_cli.out_nocaption]:
        if path:
            os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
            writers.append(imageio.get_writer(path, fps=fps, codec="libx264", quality=8, macro_block_size=1))

    obs, _ = env.reset()
    obs = obs["policy"]
    dt = unwrapped.step_dt
    fell_at = None
    stepped = False
    start_pos = None
    max_tilt = 0.0
    k = 0
    while simulation_app.is_running():
        t = k * dt
        t_push = t - SETTLE_S
        if t_push >= WINDOW_S or (fell_at is not None and t - fell_at >= FALL_TAIL_S):
            break
        pushing = 0.0 <= t_push and fell_at is None
        force_b = world_to_body(robot.data.root_quat_w, force_w * float(pushing))
        robot.set_external_force_and_torque(
            force_b.unsqueeze(1), torch.zeros_like(force_b).unsqueeze(1), body_ids=[base_id]
        )
        if pushing and start_pos is None:
            start_pos = robot.data.root_pos_w[0, :2].clone()

        with torch.inference_mode():
            # stand still
            command_term.vel_command_b[:] = 0.0
            obs[:, 0:3] = 0.0
            obs, _, _, _, _ = env.step(policy(obs))
            obs = obs["policy"]

        # the arrow ends at the torso CoM, pointing along the push
        com = robot.data.body_com_pos_w[:, base_id]
        if pushing:
            arrow.set_visibility(True)
            arrow.visualize(com - push_dir * arrow_len * ARROW_MESH_TIP / ARROW_MESH_LENGTH, arrow_quat, arrow_scale)
        else:
            arrow.set_visibility(False)

        tilt = tilt_deg(robot.data.projected_gravity_b)[0].item()
        moved = 0.0 if start_pos is None else torch.linalg.norm(robot.data.root_pos_w[0, :2] - start_pos).item()
        if t_push >= 0 and fell_at is None:
            max_tilt = max(max_tilt, tilt)
            stepped |= contact.data.current_air_time[0, feet_ids].max().item() > STEP_AIR_TIME_S
            if tilt > FALL_TILT_DEG:
                fell_at = t + dt
        status = "standing" if t_push < 0 else "fell" if fell_at is not None else "stepping" if stepped else "holding"

        if t_push < 0:
            push_text = "Push: none yet"
        elif fell_at is None:
            push_text = (f"Push: {args_cli.force:.1f} N {direction_name}, sustained at the torso CoM   "
                         f"({t_push + dt:3.1f} / {WINDOW_S:.0f} s)")
        else:
            push_text = f"Push: {args_cli.force:.1f} N {direction_name}, released after the fall"
        lines = [
            f"Sustained push test  |  Policy: {label}  |  Isaac Lab, zero velocity command",
            f"Case: {args_cli.case}" if args_cli.case else f"{args_cli.force:.1f} N {direction_name}",
            push_text,
            f"Tilt {tilt:4.1f} deg     Moved {moved:4.2f} m     t = {t + dt:4.1f} s",
        ]
        frame = env.render()
        writers[0].append_data(overlay(frame, lines, status))
        if len(writers) > 1:
            writers[1].append_data(frame)
        k += 1

    for writer in writers:
        writer.close()
    result = "fell" if fell_at is not None else "stepped" if stepped else "held in place"
    print(f"[INFO] Result: {result} (max tilt {max_tilt:.1f} deg, moved {moved:.2f} m) -> {args_cli.out}")

    # close the simulator
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
