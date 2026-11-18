"""Water trolley of design F3: an IV stand on 5 casters that a person pushes behind the robot. It carries the 2 L water
bag, the pump box and its own 6S pack. A boom brings the hose and the pump cable over the robot.

    uv run python docs/hardware/trolley.py          -> trolley_side.svg, trolley_side.png and the numbers (2 s)
    uv run python docs/hardware/trolley.py check    self-check (1 s)

Status: planned. Sizes and masses are ASSUMED until we buy the stand and weigh the parts.
Frame: x forward toward the robot, z up, origin on the floor under the pole axis. Units: m, kg, N.
sim/water/water.py reads the heights and the hose from this file.
"""
import math
import sys
from pathlib import Path

import matplotlib
from scipy.optimize import brentq

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, FancyBboxPatch, Polygon, Rectangle  # noqa: E402

HERE = Path(__file__).resolve().parent
G = 9.81

# ---------------- robot side (ASMR robot with the F3 parts, stand pose: sim/payload/final_check.py statics) ----------------
NOZZLE_Z = 0.905         # nozzle over the floor at tilt 0 (cad/final/out_f2/layout.json nozzle_fitting)
NOZZLE_LOW_Z = 0.855     # ASSUMED: nozzle at full tilt down (-40 deg)
NOZZLE_FALLEN_Z = 0.10   # ASSUMED: the robot lies on the floor
ANCHOR_Z = 0.835         # hose exit of the anchor at the rear of the deck (cad/final/out_f3/kit.json: legs at 835 mm)
ROBOT_KG, ROBOT_CG_Z = 16.35, 0.49  # ASMR robot with the F3 parts: 16.35 kg, centre of mass 0.49 m over the floor

# ---------------- stand: 5-leg IV stand (Clinton CL-IVS-352 class: 23 in base, 1 in and 3/4 in poles) ----------------
LEG_R = 0.27             # caster stem circle radius
SWIVEL = 0.03            # the wheel trails its stem by this much: worst case for tipping
HOOK_Z, HOOK_X = 1.90, -0.12  # the bag hangs on the rear hook, away from the robot
BOOM_Z, BOOM_X = 1.45, 0.60   # boom height and reach of its tip toward the robot
HANDLE_Z = 1.00          # push handle with the kill switch box
D_TIP = LEG_R * math.cos(math.radians(36)) - SWIVEL  # front tip line between two legs: 0.188 m

# ---------------- water path ----------------
TANK_L = 2.0             # 2 L hydration bag with a bottom outlet
WATER_Z = (1.80, 1.53)   # water surface: full bag, empty bag (the outlet)
PUMP_Z = 1.20            # pump inlet, 0.33 m under the bag outlet: the water fills the pump (gravity priming)
SUCTION_L = 0.30         # bag outlet to the pump inlet
HOSE_L = 3.0             # delivery hose: pump - boom - free span - robot - nozzle (2.6 m path + 0.4 m to adjust)
HOSE_BORE, HOSE_OD = 3.2e-3, 6.0e-3  # silicone tube 3 x 6 mm (3.2 mm bore in the model, as for F2)
FREE_L = 1.1             # free span: boom tip to the hose anchor
SILICONE = 1150.0        # kg/m3
CABLE_KG_M = 0.010       # pump cable, 2 x 22 AWG (ASSUMED 5 g/m per wire)
HOSE_KG_M = SILICONE * math.pi / 4 * (HOSE_OD ** 2 - HOSE_BORE ** 2) + 998 * math.pi / 4 * HOSE_BORE ** 2 + CABLE_KG_M
BREAKAWAY_N = 1.5        # ASSUMED: the boom tip clip lets the hose go at this pull (set it with test TT2). The gap study
                         # (archive/sim2sim_bhl/gap_study.md (old BHL run)): a constant 3 N pull adds falls, so keep the pull low.
OFFSET = (0.1, 0.4)      # keep the boom tip this far behind the hose anchor (horizontal)

# ---------------- masses: (kg, x, z of the centre), ASSUMED ----------------
MASSES = {"stand (light type)": (4.0, 0.0, 0.45), "bag and 2 L of water": (TANK_L * 0.998 + 0.15, HOOK_X, 1.67),
          "pump box": (0.60, -0.05, 1.155), "handle and kill switch box": (0.38, -0.15, 1.03),
          "6S pack in a LiPo bag": (0.26, -0.04, 0.46), "boom and clamp": (0.36, BOOM_X / 2, BOOM_Z),
          "hose and cable on the trolley": (0.06, 0.35, BOOM_Z)}
HEAVY_STAND_KG = 10.0    # hospital stand (MidCentral MCM210: 22 lb)


def tip_force(masses, d=D_TIP):
    """Smallest hose pull at the boom tip that lifts the rear casters: about the front tip line, for the worst pull
    direction (horizontal toward the robot, tilted down by atan((BOOM_X - d) / BOOM_Z))."""
    restore = G * sum(m * (d - x) for m, x, _ in masses.values())
    return restore / math.hypot(BOOM_Z, BOOM_X - d)


def catenary(h, v, length):
    """Hose of `length` from (0, 0) to (h, v), h > 0. Returns (a, x0, c): z = a cosh((x - x0) / a) + c."""
    s = math.sqrt(length ** 2 - v ** 2)
    a = brentq(lambda k: 2 * k * math.sinh(h / (2 * k)) - s, h / 1000, 100.0)
    x0 = h / 2 - a * math.atanh(v / length)
    return a, x0, -a * math.cosh(x0 / a)


def hose_at_anchor(h, length=FREE_L):
    """Lowest point (z over the floor) of the free span and the force of the hose on the robot (N, backward and
    downward), with the anchor h m ahead of the boom tip. Per unit weight: horizontal a, vertical = arc to the vertex."""
    a, x0, c = catenary(h, ANCHOR_Z - BOOM_Z, length)
    low = BOOM_Z + (a + c if 0 <= x0 <= h else min(0.0, ANCHOR_Z - BOOM_Z))
    w = HOSE_KG_M * G
    return dict(lowest_z=low, back_n=w * a, down_n=w * a * math.sinh((h - x0) / a))


def numbers():
    full = dict(MASSES)
    empty = dict(MASSES, **{"bag and 2 L of water": (0.15, HOOK_X, 1.67)})
    heavy = {k: (m + (HEAVY_STAND_KG - 4.0 if k.startswith("stand") else 0.0), x, z) for k, (m, x, z) in full.items()}
    heavy_empty = dict(heavy, **{"bag and 2 L of water": (0.15, HOOK_X, 1.67)})
    mass = sum(m for m, _, _ in full.values())
    zc = sum(m * z for m, _, z in full.values()) / mass
    xc = sum(m * x for m, x, _ in full.values()) / mass
    # a caster hits an obstacle: the trolley tips when its kinetic energy lifts the centre of mass over the tip line
    v_block = math.sqrt(2 * G * (math.hypot(zc, D_TIP - xc) - zc))
    # holding the robot weight at the boom tip (a fall): ballast on the base needed, static and with a factor 2 jerk
    w_r = ROBOT_KG * G
    rest_empty = G * sum(m * (D_TIP - x) for m, x, _ in empty.values())
    ballast = [(f * w_r * (BOOM_X - D_TIP) - rest_empty) / (G * D_TIP) for f in (1.0, 2.0)]
    taut = math.sqrt(FREE_L ** 2 - (BOOM_Z - ANCHOR_Z) ** 2)
    return dict(mass_full=mass, mass_empty=mass - TANK_L * 0.998, zc=zc, xc=xc,
                tip=dict(light_full=tip_force(full), light_empty=tip_force(empty), heavy_full=tip_force(heavy),
                         heavy_empty=tip_force(heavy_empty)),
                push_tip=[G * sum(m * (D_TIP - x) for m, x, _ in c.values()) / HANDLE_Z for c in (empty, full)],
                tip_bag_on_axis=tip_force(dict(full, **{"bag and 2 L of water": (TANK_L * 0.998 + 0.15, 0.0, 1.67)})),
                pull_angle=math.degrees(math.atan2(BOOM_X - D_TIP, BOOM_Z)),
                v_block=v_block, ballast=ballast, hold_lean_n=[w_r * ROBOT_CG_Z / ANCHOR_Z * math.tan(math.radians(p))
                                                              for p in (10, 20)],
                taut=taut, hose_kg_m=HOSE_KG_M, span_n=HOSE_KG_M * G * FREE_L,
                hose={h: hose_at_anchor(h) for h in (OFFSET[0], 0.25, OFFSET[1])},
                taut_angle=math.degrees(math.atan2(BOOM_Z - ANCHOR_Z, taut)),
                robot_moment=BREAKAWAY_N * math.cos(math.atan2(BOOM_Z - ANCHOR_Z, taut)) * ANCHOR_Z)


# ---------------- side view ----------------
INK, GREY, LIGHT, WATER, RED, YELLOW = "#1a1a1a", "#6b6b6b", "#d9d9d6", "#2a78d6", "#c8352d", "#e8c547"


def draw(stem):
    fig, ax = plt.subplots(figsize=(10.5, 9.6))
    ax.set_xlim(-1.0, 1.62)
    ax.set_ylim(-0.12, 2.26)
    ax.set_aspect("equal")
    ax.axis("off")
    line = lambda pts, **k: ax.plot(*zip(*pts), **({"color": INK, "lw": 1.0} | k))  # noqa: E731
    rect = lambda x, z, w, h, **k: ax.add_patch(Rectangle((x, z), w, h, **({"fc": "white", "ec": INK, "lw": 1.0} | k)))  # noqa: E731

    def label(xy, text, xytext, ha="left"):
        ax.annotate(text, xy, xytext=xytext, fontsize=8.0, ha=ha, va="center", color=INK,
                    arrowprops=dict(arrowstyle="-", color=GREY, lw=0.6, shrinkA=0, shrinkB=1))

    # floor
    line([(-0.30, 0), (1.45, 0)], lw=1.2)
    for x in [i * 0.05 - 0.27 for i in range(35)]:
        line([(x, 0), (x - 0.03, -0.03)], lw=0.4, color=GREY)
    # base: one leg points back, two legs point forward at +-36 deg (the worst case for a forward pull)
    for a in (36, 108, 180):
        xs = LEG_R * math.cos(math.radians(a))
        line([(0, 0.11), (xs, 0.085)], lw=2.2)
        line([(xs, 0.085), (xs, 0.05), (xs - SWIVEL, 0.03)], lw=1.0)
        ax.add_patch(Circle((xs - SWIVEL, 0.025), 0.025, fc="white", ec=INK, lw=1.0, zorder=3))
    rect(-0.035, 0.08, 0.07, 0.05, fc=LIGHT)
    ax.add_patch(Polygon([(D_TIP, 0), (D_TIP - 0.018, -0.045), (D_TIP + 0.018, -0.045)], fc=INK, zorder=4))
    ax.text(D_TIP + 0.03, -0.075, f"front tip line {D_TIP:.2f} m from the pole (worst direction)", fontsize=7.2,
            color=INK)
    ax.text(-0.28, -0.075, f"5 legs, caster circle {2 * LEG_R:.2f} m", fontsize=7.2, color=INK, ha="center")
    # pole, lock knob, hook head
    rect(-0.0125, 0.13, 0.025, 1.10, fc=LIGHT)
    rect(-0.0095, 1.20, 0.019, HOOK_Z + 0.03 - 1.20, fc=LIGHT)
    ax.add_patch(Circle((0.018, 1.21), 0.008, fc=GREY, zorder=3))
    line([(-0.13, HOOK_Z + 0.03), (0.13, HOOK_Z + 0.03)], lw=2.0)
    for s in (-1, 1):
        ax.add_patch(matplotlib.patches.Arc((s * 0.12 - s * 0.015, HOOK_Z + 0.045), 0.03, 0.05, theta1=0 if s > 0 else 180,
                                            theta2=180 if s > 0 else 360, color=INK, lw=1.2))
    # bag on the rear hook
    xb = HOOK_X
    line([(xb, HOOK_Z + 0.03), (xb, 1.865)], lw=0.8)
    ax.add_patch(FancyBboxPatch((xb - 0.035, 1.565), 0.07, 0.295, boxstyle="round,pad=0,rounding_size=0.02",
                                fc="white", ec=INK, lw=1.0, zorder=2))
    ax.add_patch(FancyBboxPatch((xb - 0.033, 1.567), 0.066, WATER_Z[0] - 1.567, boxstyle="round,pad=0,rounding_size=0.018",
                                fc=WATER, alpha=0.25, ec="none", zorder=2.5))
    ax.add_patch(Polygon([(xb - 0.02, 1.566), (xb + 0.02, 1.566), (xb + 0.006, WATER_Z[1]), (xb - 0.006, WATER_Z[1])],
                         fc="white", ec=INK, lw=0.8, zorder=2))
    line([(xb - 0.045, WATER_Z[0]), (xb + 0.045, WATER_Z[0])], lw=0.6, color=WATER, ls=(0, (3, 2)))
    # pump box, bag hose with the pinch clamp, kill switch handle, battery, wires
    rect(-0.0825, 1.08, 0.07, 0.15)
    rect(-0.072, 1.16, 0.05, 0.045, ls=(0, (2, 1.5)), lw=0.7)
    ax.text(-0.047, 1.177, "M1", fontsize=6.0, ha="center", color=GREY)
    ax.add_patch(Circle((-0.0875, 1.12), 0.009, fc=INK, zorder=3))
    hose = dict(color=WATER, lw=2.4, solid_capstyle="round", zorder=4)
    line([(xb, WATER_Z[1]), (xb, 1.40), (-0.06, 1.33), (-0.06, 1.23)], **hose)
    rect(xb - 0.014, 1.43, 0.028, 0.018, fc=GREY, zorder=5)
    rect(-0.235, HANDLE_Z - 0.008, 0.2225, 0.016, fc=LIGHT)
    rect(-0.235, HANDLE_Z - 0.013, 0.08, 0.026, fc=INK)
    rect(-0.235, HANDLE_Z + 0.013, 0.07, 0.06, fc=YELLOW)
    ax.add_patch(matplotlib.patches.Wedge((-0.20, HANDLE_Z + 0.073), 0.022, 0, 180, fc=RED, ec=INK, lw=0.8, zorder=3))
    rect(-0.0625, 0.40, 0.05, 0.13, fc="#f2f2f0")
    line([(-0.04, 0.53), (-0.04, 1.08)], lw=0.8)
    line([(-0.165, HANDLE_Z + 0.04), (-0.10, HANDLE_Z + 0.04), (-0.10, 1.10), (-0.0825, 1.10)], lw=0.8)
    # boom, clamp, delivery hose with the pump cable, check valve CV1, breakaway clip
    rect(-0.0, BOOM_Z - 0.01, BOOM_X, 0.02, fc=LIGHT)
    rect(-0.022, BOOM_Z - 0.03, 0.044, 0.06, fc="white")
    line([(-0.03, 1.23), (-0.03, BOOM_Z + 0.025), (0.03, BOOM_Z + 0.02), (BOOM_X - 0.01, BOOM_Z + 0.02)], **hose)
    line([(-0.02, 1.23), (-0.02, BOOM_Z + 0.033), (0.03, BOOM_Z + 0.033), (BOOM_X, BOOM_Z + 0.033)], lw=0.8,
         ls=(0, (4, 2)))
    rect(-0.042, 1.265, 0.024, 0.04, fc="white", zorder=5)
    ax.annotate("", (-0.03, 1.300), (-0.03, 1.272), arrowprops=dict(arrowstyle="-|>", color=INK, lw=0.7), zorder=6)
    rect(BOOM_X - 0.015, BOOM_Z - 0.035, 0.03, 0.07, fc=GREY, zorder=5)
    # free span to the anchor, robot sketch
    xa = BOOM_X + 0.25
    a, x0, c = catenary(xa - BOOM_X, ANCHOR_Z - BOOM_Z, FREE_L)
    xs = [BOOM_X + i * (xa - BOOM_X) / 60 for i in range(61)]
    line([(x, BOOM_Z + a * math.cosh((x - BOOM_X - x0) / a) + c) for x in xs], **hose)
    line([(x + 0.012, BOOM_Z + 0.01 + a * math.cosh((x - BOOM_X - x0) / a) + c) for x in xs], lw=0.8, ls=(0, (4, 2)))
    xr = xa + 0.015                                     # robot reference: ankle line
    robot = dict(fc="#f2f2f0", ec=GREY, lw=0.9)
    for dx in (0.0, 0.03):
        line([(xr + dx - 0.06, 0.02), (xr + dx + 0.13, 0.02)], color=GREY, lw=3.0)
        line([(xr + dx, 0.07), (xr + dx + 0.05, 0.25), (xr + dx + 0.03, 0.45)], color=GREY, lw=4.0)
    ax.add_patch(FancyBboxPatch((xr - 0.10, 0.47), 0.25, 0.31, boxstyle="round,pad=0,rounding_size=0.02", **robot))
    rect(xr - 0.08, 0.78, 0.2, 0.012, **robot)
    rect(xr + 0.01, 0.79, 0.05, 0.07, **robot)
    ax.add_patch(FancyBboxPatch((xr + 0.02, 0.865), 0.10, 0.065, boxstyle="round,pad=0,rounding_size=0.01", **robot))
    line([(xr + 0.12, NOZZLE_Z), (xr + 0.16, NOZZLE_Z)], lw=2.0)
    ax.annotate("", (xr + 0.33, NOZZLE_Z + 0.03), (xr + 0.17, NOZZLE_Z), arrowprops=dict(arrowstyle="-|>", color=WATER,
                                                                                          lw=1.2, ls=(0, (3, 2))))
    ax.text(xr + 0.07, 0.53, "robot F3\n(sketch)", fontsize=7.2, color=GREY, ha="center")
    line([(xa, ANCHOR_Z), (xa + 0.02, 0.835), (xr + 0.05, 0.86), (xr + 0.07, NOZZLE_Z), (xr + 0.12, NOZZLE_Z)], **hose)
    rect(xa - 0.02, ANCHOR_Z - 0.015, 0.03, 0.03, fc=INK, zorder=5)
    rect(xr + 0.065, 0.872, 0.018, 0.022, fc="white", zorder=6)

    # labels: left column for the trolley, right side for the boom, the hose and the robot
    label((xb - 0.035, 1.72), f"{TANK_L:g} L water bag (hydration bladder) on\nthe rear hook. Hook {HOOK_Z:.2f} m, fill line\n"
          f"{WATER_Z[0]:.2f} m, outlet {WATER_Z[1]:.2f} m", (-1.0, 1.93))
    label((xb, 1.44), "pinch clamp on the bag outlet:\nclose it to refill or to store", (-1.0, 1.70))
    label((-0.042, 1.285), "check valve CV1 at the pump outlet", (-1.0, 1.52))
    label((-0.0825, 1.15), f"pump box on the pole: KLP02 pump M1\n(inlet {PUMP_Z:.2f} m), DMC-101 U7, opto U8,\n"
          "main switch S10 with lamp H10", (-1.0, 1.33))
    label((-0.20, HANDLE_Z + 0.09), f"kill switch S11 (e-stop box) on the\npush handle at {HANDLE_Z:.2f} m", (-1.0, 1.08))
    label((-0.0625, 0.47), "6S 1300 mAh pack BT4 in a LiPo\nbag, fuse F20 at its plug", (-1.0, 0.55))
    label((BOOM_X * 0.6, BOOM_Z), f"boom at {BOOM_Z:.2f} m: 2020 profile on a\nprinted pole clamp. The hose and\n"
          "the pump cable run on top", (0.22, 1.90))
    label((BOOM_X, BOOM_Z - 0.035), f"breakaway clip: lets the\nhose go at about {BREAKAWAY_N:g} N", (0.86, 1.62))
    label((BOOM_X + 0.012, 1.10), f"free span {FREE_L:g} m: silicone hose\n3 × 6 mm and the 2-wire pump\n"
          "cable, taped together", (0.86, 1.33))
    label((xr + 0.074, 0.894), "check valve CV2 on the tilt\narm, then the 1.0 mm nozzle", (1.02, 1.08))
    label((xa - 0.02, ANCHOR_Z - 0.01), "hose anchor and plug J10\nat the rear of the deck", (0.98, 0.40))
    # height scale on the right, with the robot heights
    xh = 1.50
    line([(xh, 0), (xh, 2.0)], lw=0.6, color=GREY)
    for i in range(21):
        line([(xh, i * 0.1), (xh + (0.012 if i % 5 else 0.024), i * 0.1)], lw=0.6, color=GREY)
    for i in (0, 5, 10, 15, 20):
        ax.text(xh + 0.035, i * 0.1, f"{i / 10:.1f}", fontsize=6.8, color=GREY, va="center")
    ax.text(xh, 2.05, "height (m)", fontsize=7.0, color=GREY, ha="center")
    for z, name, x0 in ((NOZZLE_Z, "0.905 nozzle", xr + 0.34), (ANCHOR_Z, f"{ANCHOR_Z:.3f} anchor", xa + 0.02)):
        line([(x0, z), (xh, z)], lw=0.5, color=GREY, ls=(0, (2, 2)))
        ax.text(xh - 0.01, z + 0.012, name, fontsize=6.8, color=INK, ha="right")

    # horizontal dimensions
    def dim(x1, x2, z, text):
        ax.annotate("", (x1, z), (x2, z), arrowprops=dict(arrowstyle="<|-|>", color=INK, lw=0.6, shrinkA=0, shrinkB=0))
        ax.text((x1 + x2) / 2, z + 0.015, text, fontsize=6.8, ha="center", color=INK)
    dim(0, BOOM_X, BOOM_Z + 0.10, f"boom reach {BOOM_X:.2f}")
    dim(BOOM_X, xa, 0.30, f"keep {OFFSET[0]:g} to {OFFSET[1]:g}")
    line([(BOOM_X, BOOM_Z - 0.04), (BOOM_X, 0.27)], lw=0.5, color=GREY, ls=(0, (2, 2)))
    line([(xa, ANCHOR_Z - 0.02), (xa, 0.27)], lw=0.5, color=GREY, ls=(0, (2, 2)))
    ax.text(-1.0, 2.25, "Water trolley F3, side view", fontsize=10.0, color=INK, weight="bold", va="top")
    ax.text(-1.0, 2.185, "Planned. Sizes in m. The masses and the stand type are assumed. Blue: water. Dashed black: "
            "pump cable.\nThe robot is a sketch, with its hose anchor in the middle of the walking zone.",
            fontsize=7.4, color=GREY, va="top")
    plt.rcParams["svg.hashsalt"] = "sobang"   # stable ids: the same input gives the same file
    fig.savefig(f"{stem}.svg", bbox_inches="tight", metadata={"Date": None})
    fig.savefig(f"{stem}.png", dpi=130, bbox_inches="tight")
    plt.close(fig)


def main():
    n = numbers()
    t, h = n["tip"], n["hose"]
    print(f"trolley mass {n['mass_full']:.1f} kg full, {n['mass_empty']:.1f} kg empty, centre of mass "
          f"{n['zc']:.2f} m high, {n['xc']:+.3f} m forward")
    print(f"hose pull at the boom tip that tips the trolley (worst direction): light stand {t['light_full']:.1f} N full, "
          f"{t['light_empty']:.1f} N empty. 10 kg stand {t['heavy_full']:.1f} N full, {t['heavy_empty']:.1f} N empty")
    print(f"worst pull: {n['pull_angle']:.0f} deg down from the horizontal. Bag on the pole axis instead of the rear hook: "
          f"{n['tip_bag_on_axis']:.1f} N full")
    print(f"push at the handle that tips the trolley when the front casters are blocked: {n['push_tip'][0]:.0f} N empty, "
          f"{n['push_tip'][1]:.0f} N full")
    print(f"speed at which a blocked caster tips the full trolley (point mass): {n['v_block']:.2f} m/s")
    print(f"force to hold the robot leaning 10 / 20 deg at the anchor: {n['hold_lean_n'][0]:.0f} / {n['hold_lean_n'][1]:.0f} N")
    print(f"ballast on the base to hold the robot weight at the boom tip: {n['ballast'][0]:.0f} kg static, "
          f"{n['ballast'][1]:.0f} kg with a factor 2 jerk")
    print(f"hose with water and cable {1000 * n['hose_kg_m']:.0f} g/m. Free span {FREE_L} m weighs {n['span_n']:.2f} N. "
          f"It goes taut at {n['taut']:.2f} m offset.")
    for k, v in h.items():
        print(f"  offset {k:.2f} m: lowest point {v['lowest_z']:.2f} m, on the robot {v['down_n']:.2f} N down, "
              f"{v['back_n']:.3f} N back")
    print(f"taut hose at the breakaway force {BREAKAWAY_N:g} N: {n['taut_angle']:.0f} deg up, pitch moment on the robot "
          f"{n['robot_moment']:.1f} N m")
    draw(str(HERE / "trolley_side"))
    return n


def check():
    # the worst-direction formula equals a brute-force search over the pull angle
    m = {"block": (5.0, 0.0, 0.5)}
    restore = G * 5.0 * D_TIP
    brute = min(restore / (BOOM_Z * math.cos(t) + (BOOM_X - D_TIP) * math.sin(t))
                for t in [math.radians(i / 10) for i in range(0, 900)])
    assert abs(tip_force(m) - brute) < 1e-3, (tip_force(m), brute)
    # the catenary passes through both ends and has the hose length
    a, x0, c = catenary(0.25, ANCHOR_Z - BOOM_Z, FREE_L)
    z = lambda x: a * math.cosh((x - x0) / a) + c  # noqa: E731
    assert abs(z(0.0)) < 1e-9 and abs(z(0.25) - (ANCHOR_Z - BOOM_Z)) < 1e-9
    assert abs(a * (math.sinh((0.25 - x0) / a) + math.sinh(x0 / a)) - FREE_L) < 1e-9
    # design: the clip lets the hose go well before the empty trolley tips, and the hose loop stays above the hips
    n = numbers()
    assert n["tip"]["light_empty"] > 2 * BREAKAWAY_N, n["tip"]
    assert all(v["lowest_z"] > 0.55 for v in n["hose"].values()), n["hose"]
    print("trolley.py check: OK")


if __name__ == "__main__":
    check() if sys.argv[1:] == ["check"] else main()
