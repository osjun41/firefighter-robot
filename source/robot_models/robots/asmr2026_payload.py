"""Payload options of the ASMR 2026 robot: what is carried on the course torso.

    none      no payload (the bare course robot, 12.21 kg)
    box_1kg   one 1 kg, 10 cm box on the torso top plate. The default: the baseline policy trained with it.
    f3        the parts of our F3 design, one rigid box each (mass, centre, size), from cad/final/out_f3/layout.json:
              batteries, computer, CAN adapters, IMU, relays, pan-tilt servos, camera, nozzle, the printed pod and
              the end of the hose. About 4.1 kg. There is no torso frame: the parts are separate pieces.

Positions are in the robot base frame (x forward, y left, z up). The CAD layout has z = 0 on the floor, and the base
frame of the standing robot is 0.0398 m below it (STANDING_ROOT_HEIGHT), so z_base = z_layout + 0.0398. With that
shift the F3 deck lies on the torso top plate (z = 0.8338), as the 1 kg box does.

Mass: the course torso (5.57 kg) already holds its own frame, electronics and batteries. F3 parts are ADDED to it and
nothing is taken out of the torso, so the F3 total is an upper bound if some part repeats what the torso holds.

Each part is a rigid solid box without collision. Because they are rigid, a simulator may keep them as separate bodies
(MuJoCo) or merge them into the base link (Isaac Lab): `combined(name)` gives the equivalent single body (mass,
centre, inertia about the centre, by the parallel-axis theorem).

Standard library and NumPy only. Select an option with `--payload NAME` (add_argument) in the scripts.
"""
import json
from pathlib import Path

import numpy as np

from .asmr2026_model import STANDING_ROOT_HEIGHT

ROOT = Path(__file__).resolve().parents[3]
F3_LAYOUT = ROOT / "cad" / "final" / "out_f3" / "layout.json"
NAMES = ("none", "box_1kg", "f3")
DEFAULT = "box_1kg"
Z_LAYOUT_TO_BASE = -STANDING_ROOT_HEIGHT  # m, base-frame z of the floor in the standing pose

_BOX = dict(name="payload_upper", mass=1.0, position=(0.0, 0.0, 0.8838), size=(0.10, 0.10, 0.10))


def check(name):
    if name not in NAMES:
        raise ValueError(f"unknown payload {name!r}. Available: {', '.join(NAMES)}")
    return name


def parts(name=DEFAULT):
    """List of rigid boxes: dicts with name, mass (kg), position (base-frame centre, m) and size (edge lengths, m)."""
    check(name)
    if name == "none":
        return []
    if name == "box_1kg":
        return [dict(_BOX)]
    layout = json.loads(F3_LAYOUT.read_text(encoding="utf-8"))
    return [dict(name=f"payload_{i['name']}", mass=float(i["mass_kg"]),
                 position=(i["com_m"][0], i["com_m"][1], i["com_m"][2] + Z_LAYOUT_TO_BASE),
                 size=tuple(i["size_m"]))
            for i in layout["items"]]


def box_inertia(mass, size):
    """Diagonal of the inertia of a solid box about its centre."""
    x, y, z = size
    return mass / 12.0 * np.array([y * y + z * z, x * x + z * z, x * x + y * y])


def combined(name=DEFAULT):
    """Equivalent single body of a payload option, see `combine`."""
    return combine(parts(name))


def combine(items):
    """Equivalent single body of rigid boxes (dicts with mass, position, size): (mass kg, centre (3,), inertia about
    the centre (3, 3), in base-frame axes). No parts give mass 0, centre 0 and zero inertia."""
    mass = sum(p["mass"] for p in items)
    if not items:
        return 0.0, np.zeros(3), np.zeros((3, 3))
    centre = sum(p["mass"] * np.array(p["position"]) for p in items) / mass
    inertia = np.zeros((3, 3))
    for p in items:
        d = np.array(p["position"]) - centre
        inertia += np.diag(box_inertia(p["mass"], p["size"])) + p["mass"] * (d @ d * np.eye(3) - np.outer(d, d))
    return mass, centre, inertia


def describe(name=DEFAULT):
    mass, centre, _ = combined(name)
    return f"{name}: {len(parts(name))} part(s), {mass:.3f} kg, centre ({centre[0]:+.3f}, {centre[1]:+.3f}, {centre[2]:.3f}) m"


def sim_payload(name=DEFAULT):
    """The `sim_payload` entry of a deployment config (policy.yaml) for MuJoCo, or None for no payload.
    One box: mass, position, size. Several: {"parts": [{name, mass, position, size}, ...]}."""
    items = parts(name)
    if not items:
        return None
    if len(items) == 1:
        return {k: (list(v) if isinstance(v, tuple) else v) for k, v in items[0].items() if k != "name"}
    return {"parts": [{k: (list(v) if isinstance(v, tuple) else v) for k, v in p.items()} for p in items]}


def add_argument(parser):
    """Add `--payload NAME` to an argparse parser. Default None keeps the task's own payload (box_1kg)."""
    parser.add_argument("--payload", default=None, choices=NAMES, metavar="NAME",
                        help=f"payload on the torso: {', '.join(NAMES)} (default: the task's own, {DEFAULT})")
