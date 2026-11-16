"""Shared pieces of the dynamics-gap videos: the command scenario, torso mass changes and the English overlays.

Import it after the simulation app has been launched (it is used by record_gap_video.py and scenario_fall_rate.py).
"""

import numpy as np
import torch
from PIL import Image, ImageDraw, ImageFont

import isaaclab.utils.math as math_utils

FONT_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_BOLD_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"

# base tilt that counts as a fall (the training termination)
FALL_TILT_RAD = 0.78
# torso (base link) mass of the model
TORSO_MASS_KG = 3.16
# follow camera and velocity arrows, as in the earlier scenario and push videos (scripts/isaaclab/scenario)
CAM_EYE = (2.0, 2.0, 1.15)
CAM_LOOKAT = (0.0, 0.0, 0.55)
ARROW_HEIGHT = 0.95

# (caption, start time in s, (vx, vy, wz) command in the base frame); a segment lasts until the next one starts.
# Moderate commands inside the training ranges (vx +-0.5, vy +-0.25 m/s, wz +-1 rad/s): a start from standing
# straight to the maximum 0.5 m/s made even the fixed-dynamics policy fall in 4 of 32 runs on its own dynamics.
SCENARIO = [
    ("Stand still", 0.0, (0.0, 0.0, 0.0)),
    ("Walk forward 0.4 m/s", 1.5, (0.4, 0.0, 0.0)),
    ("Turn left in place 0.8 rad/s", 5.5, (0.0, 0.0, 0.8)),
    ("Walk forward 0.3 m/s, turning right 0.5 rad/s", 8.5, (0.3, 0.0, -0.5)),
    ("Side step left 0.2 m/s", 12.5, (0.0, 0.2, 0.0)),
    ("Walk backward 0.3 m/s", 15.5, (-0.3, 0.0, 0.0)),
    ("Stand still", 18.5, (0.0, 0.0, 0.0)),
]
DURATION_S = 20.5


def command_at(t):
    """Caption and (vx, vy, wz) command of the scenario at time t."""
    label, cmd = SCENARIO[0][0], SCENARIO[0][2]
    for name, start, c in SCENARIO:
        if t + 1e-9 >= start:
            label, cmd = name, c
    return label, cmd


def build_actor(state_dict, device):
    """Deterministic actor MLP (Linear + ELU) from an RSL-RL checkpoint state dict."""
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


def set_torso_mass(robot, base_id, add_kg):
    """Add add_kg (one value per environment) to the torso mass; its inertia scales with the mass (PhysX, CPU)."""
    view = robot.root_physx_view
    n = robot.num_instances
    ids = torch.arange(n)
    default_mass = robot.data.default_mass.cpu()
    masses = default_mass.clone()
    masses[:, base_id] += torch.as_tensor(add_kg, dtype=torch.float32).reshape(n)
    view.set_masses(masses, ids)
    view.set_inertias(robot.data.default_inertia.cpu() * (masses / default_mass)[:, :, None], ids)


def measured_velocity(robot):
    """(vx, vy) in the yaw frame and the world yaw rate, as in the tracking rewards; and the base tilt (rad)."""
    lin = math_utils.quat_rotate_inverse(math_utils.yaw_quat(robot.data.root_quat_w), robot.data.root_lin_vel_w)
    tilt = torch.acos(torch.clamp(-robot.data.projected_gravity_b[:, 2], -1.0, 1.0))
    return lin[:, 0], lin[:, 1], robot.data.root_ang_vel_w[:, 2], tilt


_fonts = {}


def _font(size, bold=False):
    key = (size, bold)
    if key not in _fonts:
        _fonts[key] = ImageFont.truetype(FONT_BOLD_PATH if bold else FONT_PATH, size)
    return _fonts[key]


def _panel(draw, box, alpha=150):
    draw.rounded_rectangle(box, radius=12, fill=(0, 0, 0, alpha))


def caption_overlay(frame, title, condition, t, label, cmd, status, status_color, stat_lines):
    """English captions: title and condition, current command, status (walking / fell), arrow legend, statistics."""
    img = Image.fromarray(frame).convert("RGBA")
    layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(layer)
    W, H = img.size
    vx, vy, wz = cmd
    lines = [
        (title, _font(26, True), (255, 255, 255)),
        (condition, _font(22), (255, 220, 120)),
        (f"Command: {label}", _font(22), (235, 235, 235)),
        (f"vx {vx:+.2f} m/s   vy {vy:+.2f} m/s   wz {wz:+.2f} rad/s        t = {t:4.1f} s", _font(19),
         (215, 215, 215)),
        (status, _font(22, True), status_color),
    ]
    h = 16 + sum(f.size + 9 for _, f, _ in lines)
    _panel(draw, (16, 14, W - 16, 14 + h))
    y = 22
    for text, f, color in lines:
        draw.text((32, y), text, font=f, fill=color)
        y += f.size + 9

    # bottom left: arrow legend
    legend = [((40, 220, 40), "Green arrow: commanded velocity"), ((60, 90, 255), "Blue arrow: actual velocity")]
    f = _font(19)
    line_h = f.size + 9
    top = H - 16 - line_h * len(legend) - 18
    _panel(draw, (16, top, 400, H - 16))
    y = top + 10
    for color, text in legend:
        draw.rectangle((32, y + 4, 54, y + 18), fill=color + (255,))
        draw.text((64, y), text, font=f, fill=(240, 240, 240))
        y += line_h

    # bottom right: statistics of this condition
    if stat_lines:
        widths = [draw.textlength(s, font=f) for s in stat_lines]
        box_w = max(widths) + 36
        top = H - 16 - line_h * len(stat_lines) - 18
        _panel(draw, (W - 16 - box_w, top, W - 16, H - 16))
        y = top + 10
        for s in stat_lines:
            draw.text((W - 16 - box_w + 18, y), s, font=f, fill=(240, 240, 240))
            y += line_h
    return np.array(Image.alpha_composite(img, layer).convert("RGB"))


def graph_overlay(frame, history, window_s=6.0):
    """Strip charts of the commanded vs actual forward speed and yaw rate over the last window_s, plus the mean
    tracking errors so far (while upright). history: dict of lists t, cmd_vx, vx, cmd_wz, wz, lin_err, yaw_err."""
    img = Image.fromarray(frame).convert("RGBA")
    layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(layer)
    W, H = img.size
    pw, ph = 440, 100  # plot size (the panel must end above the statistics box at the bottom right)
    x0 = W - 16 - pw - 36
    top = 198
    box = (x0, top, W - 16, top + 2 * (ph + 46) + 92)
    _panel(draw, box, alpha=165)
    t = np.array(history["t"])
    t_end = t[-1] if len(t) else 0.0
    t_start = max(0.0, t_end - window_s)
    sel = t >= t_start
    small = _font(17)
    plots = [("Forward speed vx (m/s)", "cmd_vx", "vx", (-0.7, 0.7)), ("Yaw rate wz (rad/s)", "cmd_wz", "wz", (-1.4, 1.4))]
    for i, (name, key_cmd, key_act, (lo, hi)) in enumerate(plots):
        px, py = x0 + 22, top + 14 + i * (ph + 46)
        draw.text((px, py), name, font=small, fill=(235, 235, 235))
        gy = py + 26
        draw.rectangle((px, gy, px + pw, gy + ph), outline=(120, 120, 120, 255))
        zero = gy + ph * (hi - 0.0) / (hi - lo)
        draw.line((px, zero, px + pw, zero), fill=(90, 90, 90, 255))

        def to_xy(tt, v):
            return (px + pw * (tt - t_start) / window_s, gy + ph * (hi - np.clip(v, lo, hi)) / (hi - lo))

        for key, color, width in [(key_cmd, (40, 220, 40, 255), 3), (key_act, (90, 150, 255, 255), 2)]:
            values = np.array(history[key])[sel]
            if len(values) > 1:
                pts = [to_xy(tt, v) for tt, v in zip(t[sel], values)]
                draw.line(pts, fill=color, width=width, joint="curve")
    # legend and running errors
    ly = top + 2 * (ph + 46) + 10
    draw.rectangle((x0 + 22, ly + 5, x0 + 44, ly + 9), fill=(40, 220, 40, 255))
    draw.text((x0 + 52, ly - 3), "command", font=small, fill=(235, 235, 235))
    draw.rectangle((x0 + 150, ly + 5, x0 + 172, ly + 9), fill=(90, 150, 255, 255))
    draw.text((x0 + 180, ly - 3), "actual", font=small, fill=(235, 235, 235))
    lin = np.mean(history["lin_err"]) if history["lin_err"] else 0.0
    yaw = np.mean(history["yaw_err"]) if history["yaw_err"] else 0.0
    draw.text((x0 + 22, ly + 22), "Mean tracking error so far:", font=small, fill=(255, 220, 120))
    draw.text((x0 + 22, ly + 44), f"speed {lin:.3f} m/s, yaw rate {yaw:.3f} rad/s", font=small, fill=(255, 220, 120))
    return np.array(Image.alpha_composite(img, layer).convert("RGB"))
