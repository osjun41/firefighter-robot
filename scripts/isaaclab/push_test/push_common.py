"""Shared setup for the standing push test: environment config, ONNX policy, force helpers.

Import this module only after the Isaac Sim app has been launched.
"""

import numpy as np
import torch

import onnxruntime as ort

import isaaclab.utils.math as math_utils
from isaaclab_tasks.utils import parse_env_cfg

# Import extensions to set up environment tasks
import berkeley_humanoid_lite.tasks  # noqa: F401

# timeline of a trial: stand still, then push; a trial passes if the robot does not fall within WINDOW_S
SETTLE_S = 2.0
WINDOW_S = 5.0
IMPULSE_S = 0.2
# fall: base tilt beyond the bad_orientation termination of the training task (0.78 rad)
FALL_TILT_DEG = float(np.degrees(0.78))
# a foot in the air longer than this counts as a step
STEP_AIR_TIME_S = 0.08
# push directions in the robot frame: 0 = toward the robot's front (pushed from behind), 90 = toward its left
DIRECTIONS_DEG = [0, 45, 90, 135, 180, 225, 270, 315]
DIRECTION_NAMES = {0: "forward", 45: "forward-left", 90: "left", 135: "backward-left",
                   180: "backward", 225: "backward-right", 270: "right", 315: "forward-right"}


def make_standing_env_cfg(task, device, num_envs, policy_dt, terminate_on_fall=True):
    """Task config for standing still in nominal conditions, with the command written by the caller."""
    env_cfg = parse_env_cfg(task, device=device, num_envs=num_envs)
    # run the policy at the rate it was trained for
    env_cfg.decimation = round(policy_dt / env_cfg.sim.dt)
    env_cfg.sim.render_interval = env_cfg.decimation
    env_cfg.episode_length_s = 60.0

    # the command is written directly every step
    command_cfg = env_cfg.commands.base_velocity
    command_cfg.heading_command = False
    command_cfg.rel_standing_envs = 0.0
    command_cfg.resampling_time_range = (1.0e6, 1.0e6)
    command_cfg.debug_vis = False

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

    if not terminate_on_fall:
        env_cfg.terminations.base_orientation = None
    return env_cfg


class OnnxPolicy:
    """Runs an ONNX policy on a batch of observations, one row at a time if the batch size is fixed."""

    def __init__(self, checkpoint_path: str, action_limits: tuple[float, float] | None = None):
        self.session = ort.InferenceSession(checkpoint_path, providers=["CPUExecutionProvider"])
        # the bound the policy was trained with (deployment config action_limit_lower/upper), as RlController clips
        self.action_limits = action_limits
        model_input = self.session.get_inputs()[0]
        self.input_name = model_input.name
        self.fixed_batch = isinstance(model_input.shape[0], int)
        self.num_inputs = model_input.shape[-1]

    def __call__(self, observations: torch.Tensor) -> torch.Tensor:
        obs = observations.detach().cpu().numpy().astype(np.float32)
        if obs.shape[1] + 1 == self.num_inputs:
            # a policy that also observes the push signal: no push is planned
            obs = np.concatenate([obs, np.zeros((obs.shape[0], 1), dtype=np.float32)], axis=1)
        if self.fixed_batch:
            actions = np.concatenate([self.session.run(None, {self.input_name: row[None]})[0] for row in obs])
        else:
            actions = self.session.run(None, {self.input_name: obs})[0]
        if self.action_limits is not None:
            actions = np.clip(actions, *self.action_limits)
        return torch.from_numpy(actions).to(observations.device)


def horizontal_force_w(directions_deg: torch.Tensor, magnitudes: torch.Tensor) -> torch.Tensor:
    """World-frame horizontal forces; the robot starts facing +x, so the angle is relative to its heading."""
    theta = torch.deg2rad(directions_deg)
    return torch.stack([magnitudes * torch.cos(theta), magnitudes * torch.sin(theta), torch.zeros_like(theta)], dim=-1)


def world_to_body(quat_wxyz: torch.Tensor, vec_w: torch.Tensor) -> torch.Tensor:
    """Rotate world-frame vectors (n, 3) into the body frames given by quaternions (n, 4)."""
    w, x, y, z = quat_wxyz.unbind(-1)
    rot = torch.stack([
        1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
        2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
        2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y),
    ], dim=-1).view(-1, 3, 3)
    return torch.bmm(rot.transpose(1, 2), vec_w.unsqueeze(-1)).squeeze(-1)


def tilt_deg(projected_gravity_b: torch.Tensor) -> torch.Tensor:
    """Angle between the base z axis and the world vertical."""
    return torch.rad2deg(torch.acos(torch.clamp(-projected_gravity_b[:, 2], -1.0, 1.0)))


class PushArrow:
    """Red arrow drawn into the scene, ending at the push point and pointing along the push."""

    # the arrow mesh spans x in [-0.25, 0.75] (tip) and is 1.0 wide at scale 1
    MESH_TIP = 0.75

    def __init__(self, prim_path="/Visuals/push_arrow", metres_per_newton=0.035, thickness=0.1):
        from isaaclab.markers import VisualizationMarkers
        from isaaclab.markers.config import RED_ARROW_X_MARKER_CFG

        cfg = RED_ARROW_X_MARKER_CFG.replace(prim_path=prim_path)
        # unit prototype, so the scale passed per frame is the whole scale
        cfg.markers["arrow"].scale = (1.0, 1.0, 1.0)
        self.markers = VisualizationMarkers(cfg)
        self.metres_per_newton = metres_per_newton
        self.thickness = thickness

    def update(self, tip_w: torch.Tensor, force_w: torch.Tensor):
        """Show arrows for forces (n, 3) acting at points tip_w (n, 3); hide them if all forces are zero."""
        magnitude = torch.linalg.norm(force_w, dim=-1)
        if not bool((magnitude > 1e-3).any()):
            self.markers.set_visibility(False)
            return
        self.markers.set_visibility(True)
        length = magnitude * self.metres_per_newton
        direction = force_w / magnitude.clamp(min=1e-6).unsqueeze(-1)
        yaw = torch.atan2(direction[:, 1], direction[:, 0])
        zeros = torch.zeros_like(yaw)
        quat = math_utils.quat_from_euler_xyz(zeros, zeros, yaw)
        width = torch.full_like(length, self.thickness)
        scale = torch.stack([length, width, width], dim=-1)
        self.markers.visualize(tip_w - direction * (length * self.MESH_TIP).unsqueeze(-1), quat, scale)


CAPTION_FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
CAPTION_FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
STATUS_COLORS = {"neutral": (230, 230, 230), "good": (130, 220, 140), "warn": (255, 205, 110), "bad": (255, 120, 120)}


def draw_caption(frame, lines, status, status_kind="neutral", legend="Red arrow: applied push (length ~ force)"):
    """Draw English captions on an RGB frame: a title, a bold line, plain lines, a colored status line."""
    from PIL import Image, ImageDraw, ImageFont

    img = Image.fromarray(frame).convert("RGBA")
    layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(layer)
    W, H = img.size
    title, bold, *plain = lines
    rows = [(title, ImageFont.truetype(CAPTION_FONT, 24), (215, 215, 215)),
            (bold, ImageFont.truetype(CAPTION_FONT_BOLD, 30), (255, 255, 255))]
    rows += [(text, ImageFont.truetype(CAPTION_FONT, 22), (235, 235, 235)) for text in plain]
    rows.append((status, ImageFont.truetype(CAPTION_FONT_BOLD, 26), STATUS_COLORS[status_kind]))
    height = 18 + sum(font.size + 10 for _, font, _ in rows)
    draw.rounded_rectangle((16, 14, W - 16, 14 + height), radius=12, fill=(0, 0, 0, 150))
    y = 24
    for text, font, color in rows:
        draw.text((32, y), text, font=font, fill=color)
        y += font.size + 10
    if legend:
        font = ImageFont.truetype(CAPTION_FONT, 22)
        top = H - 16 - (font.size + 24)
        draw.rounded_rectangle((16, top, 40 + 34 + int(draw.textlength(legend, font=font)), H - 16), radius=12,
                               fill=(0, 0, 0, 150))
        draw.rectangle((32, top + 17, 54, top + 31), fill=(230, 40, 40, 255))
        draw.text((66, top + 11), legend, font=font, fill=(240, 240, 240))
    return np.array(Image.alpha_composite(img, layer).convert("RGB"))
