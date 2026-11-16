"""Shared scenario, camera and caption overlay for the Isaac Lab and MuJoCo scenario videos."""

import os

import numpy as np
from PIL import Image, ImageDraw, ImageFont

# Korean captions need a CJK font: Noto Sans CJK on Linux, Malgun Gothic on Windows, Apple SD Gothic on macOS
_FONT_CANDIDATES = (
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc",
    r"C:\Windows\Fonts\malgun.ttf",
    "/System/Library/Fonts/AppleSDGothicNeo.ttc",
)
FONT_PATH = next((p for p in _FONT_CANDIDATES if os.path.isfile(p)), _FONT_CANDIDATES[0])

TURN_TARGET_DEG = -90.0  # right turn
TURN_RATE = -1.0  # rad/s, the max yaw rate in the training command range
TURN_TIMEOUT_S = 8.0  # a 90 deg turn at 1 rad/s takes ~1.6 s; move on if the policy cannot turn

# velocity arrows are drawn this high above the root (the root is at foot level, top of torso ~0.8 m)
ARROW_HEIGHT = 0.95
# follow camera, as offsets from the root position in world axes
CAM_EYE = (2.0, 2.0, 1.15)
CAM_LOOKAT = (0.0, 0.0, 0.55)

# (label, duration in s or None for the turn, (vx, vy, wz) command in the base frame)
SCENARIO = [
    ("준비 (정지)", 1.0, (0.0, 0.0, 0.0)),
    ("전진", 3.0, (0.3, 0.0, 0.0)),
    ("오른쪽 90° 회전", None, (0.0, 0.0, TURN_RATE)),
    ("전진", 2.0, (0.3, 0.0, 0.0)),
    ("왼쪽으로 이동", 2.0, (0.0, 0.2, 0.0)),
    ("후진", 3.0, (-0.3, 0.0, 0.0)),
]


def _font(size):
    if not os.path.isfile(FONT_PATH):
        raise FileNotFoundError(f"no Korean font found (looked for {_FONT_CANDIDATES}). Linux: sudo apt install fonts-noto-cjk")
    # pick the Korean face inside the CJK collection
    for idx in range(10):
        try:
            font = ImageFont.truetype(FONT_PATH, size, index=idx)
        except OSError:
            break
        if "KR" in " ".join(font.getname()):
            return font
    return ImageFont.truetype(FONT_PATH, size)


_fonts = {}


def _get_fonts():
    if not _fonts:
        _fonts.update(title=_font(26), main=_font(30), small=_font(22))
    return _fonts["title"], _fonts["main"], _fonts["small"]


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def yaw_from_quat_wxyz(q):
    w, x, y, z = q
    return np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def overlay(frame, title, t, label, cmd, turn_info=None, note=None):
    """Draw the captions on an RGB frame (H, W, 3 uint8) and return the new frame."""
    font_title, font_main, font_small = _get_fonts()
    img = Image.fromarray(frame).convert("RGBA")
    layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(layer)
    W, H = img.size

    # top: title and current command
    vx, vy, wz = cmd
    lines = [
        (title, font_title, (220, 220, 220)),
        (f"현재 명령: {label}", font_main, (255, 255, 255)),
        (f"vx = {vx:+.2f} m/s   vy = {vy:+.2f} m/s   wz = {wz:+.2f} rad/s      t = {t:4.1f} s", font_small,
         (230, 230, 230)),
    ]
    if turn_info is not None:
        lines.append((turn_info, font_small, (255, 220, 120)))
    if note is not None:
        lines.append((note, font_small, (255, 140, 140)))
    h = 18 + sum(f.size + 10 for _, f, _ in lines)
    draw.rounded_rectangle((16, 14, W - 16, 14 + h), radius=12, fill=(0, 0, 0, 150))
    y = 24
    for text, f, color in lines:
        draw.text((32, y), text, font=f, fill=color)
        y += f.size + 10

    # bottom left: arrow legend
    legend = [
        ((40, 220, 40), "초록 화살표: 명령 속도 (평면 이동 방향·크기)"),
        ((60, 90, 255), "파란 화살표: 실제 속도 (로봇 몸통 기준)"),
        (None, "※ 회전 속도(wz)는 화살표에 나타나지 않습니다"),
    ]
    line_h = font_small.size + 10
    top = H - 20 - line_h * len(legend) - 16
    draw.rounded_rectangle((16, top, 600, H - 16), radius=12, fill=(0, 0, 0, 150))
    y = top + 12
    for color, text in legend:
        x = 32
        if color is not None:
            draw.rectangle((x, y + 5, x + 22, y + 5 + 14), fill=color + (255,))
            x += 34
        draw.text((x, y), text, font=font_small, fill=(240, 240, 240))
        y += line_h

    return np.array(Image.alpha_composite(img, layer).convert("RGB"))


class ScenarioRunner:
    """Steps through SCENARIO. Call command(yaw_deg) once per policy step until done is True."""

    def __init__(self, policy_dt):
        self.dt = policy_dt
        self.phase = 0
        self.t_phase = 0.0
        self.t = 0.0
        self.turn_accum = 0.0
        self.prev_yaw = None
        self.turn_result = None
        self.done = False

    def command(self, yaw_deg):
        """Return (label, cmd, turn_info) for this step and advance time."""
        label, duration, cmd = SCENARIO[self.phase]
        turn_info = None
        if duration is None:
            # turn until the measured heading has changed by TURN_TARGET_DEG, or the time limit
            if self.prev_yaw is None:
                self.prev_yaw = yaw_deg
            self.turn_accum += wrap_deg(yaw_deg - self.prev_yaw)
            self.prev_yaw = yaw_deg
            turn_info = (f"회전량 {abs(self.turn_accum):4.0f}° / {abs(TURN_TARGET_DEG):.0f}°   "
                         f"(시간 제한 {TURN_TIMEOUT_S:.0f} s 중 {self.t_phase:3.1f} s)")
            reached = self.turn_accum <= TURN_TARGET_DEG
            if reached or self.t_phase >= TURN_TIMEOUT_S:
                self.turn_result = (self.turn_accum, self.t_phase, reached)
                self._next()
                return self.command(yaw_deg)
        elif self.t_phase >= duration - 1e-9:
            self._next()
            if self.done:
                return label, cmd, None
            return self.command(yaw_deg)
        self.t_phase += self.dt
        self.t += self.dt
        return label, cmd, turn_info

    def _next(self):
        self.phase += 1
        self.t_phase = 0.0
        if self.phase >= len(SCENARIO):
            self.done = True
            self.phase = len(SCENARIO) - 1

    def note(self):
        """Caption line with the result of the turn, once it is over."""
        if self.turn_result is None:
            return None
        accum, t, reached = self.turn_result
        if reached:
            return f"회전 완료: {abs(accum):.0f}° ({t:.1f} s)"
        return f"회전 시간 제한 도달: {abs(accum):.0f}°만 회전 ({t:.1f} s) → 다음 동작으로 넘어감"
