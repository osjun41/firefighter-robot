"""Radar (spider) chart of standing push test results: a pretrained policy against ours.

Each axis is a push direction in the robot frame. The line is the largest force with no fall at or below
it; the shaded band reaches the largest force that was survived at all (a gray zone where the outcome
flips). One panel per push type (short impulse, sustained). Text is in English.

Usage (from the repository root):
    uv run --project scripts/isaaclab scripts/isaaclab/push_test/plot_push_radar.py --pretrained logs/push_test/policy_biped_25hz_a_fine.csv \
        --ours logs/push_test/ours_baseline_fine.csv --out logs/push_test/push_radar.png
"""

import argparse
import csv

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import PathPatch  # noqa: E402
from matplotlib.path import Path  # noqa: E402

parser = argparse.ArgumentParser(description="Radar chart of the standing push test.")
parser.add_argument("--pretrained", type=str, required=True, help="Result CSV of the pretrained policy.")
parser.add_argument("--ours", type=str, required=True, help="Result CSV of our policy.")
parser.add_argument("--pretrained_label", type=str, default="Pretrained (policy_biped_25hz_a)")
parser.add_argument("--ours_label", type=str, default="Ours (baseline, 6000 iter)")
parser.add_argument("--out", type=str, required=True, help="Output image path (.png).")
args = parser.parse_args()

# light surface and a colorblind-safe blue/orange pair (checked for CVD separation and contrast)
SURFACE = "#fcfcfb"
TEXT_PRIMARY = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
GRID = "#d9d8d3"
COLOR = {"ours": "#2a78d6", "pretrained": "#eb6834"}

DIRS = [0, 45, 90, 135, 180, 225, 270, 315]
DIR_LABEL = {0: "Forward", 45: "Fwd-Left", 90: "Left", 135: "Back-Left",
             180: "Backward", 225: "Back-Right", 270: "Right", 315: "Fwd-Right"}


def boundaries(path, typ):
    """Per direction: safe = largest force with no fall at it or below; best = largest force survived."""
    rows = [r for r in csv.DictReader(open(path)) if r["type"] == typ]
    safe, best = [], []
    for d in DIRS:
        rr = sorted([r for r in rows if int(r["direction_deg"]) == d], key=lambda r: float(r["force_N"]))
        forces = [float(r["force_N"]) for r in rr]
        fell = [r["fell"] == "1" for r in rr]
        s = 0.0
        for f, fl in zip(forces, fell):
            if fl:
                break
            s = f
        safe.append(s)
        best.append(max(f for f, fl in zip(forces, fell) if not fl))
    step = forces[1] - forces[0]
    return np.array(safe), np.array(best), step


def xy(r):
    # robot faces up (forward = +y on the page), left is to the left; 0 deg = forward, counterclockwise
    th = np.radians(DIRS)
    return -r * np.sin(th), r * np.cos(th)


def closed(a):
    return np.append(a, a[0])


def radar(ax, data, rmax, rstep, title, unit_note):
    ax.set_facecolor(SURFACE)
    ax.set_aspect("equal")
    ax.axis("off")
    th = np.radians(DIRS)
    # grid: octagon rings + spokes (recessive)
    for r in np.arange(rstep, rmax + 1e-9, rstep):
        gx, gy = xy(np.full(len(DIRS), r))
        ax.plot(closed(gx), closed(gy), color=GRID, lw=0.8, zorder=1)
        a = np.radians(-22.5)
        ax.text(-r * np.sin(a), r * np.cos(a), f"{r:g}", color=TEXT_SECONDARY, fontsize=8, ha="center", va="center",
                zorder=1.5, bbox=dict(boxstyle="round,pad=0.15", fc=SURFACE, ec="none"))
    for t in th:
        ax.plot([0, -rmax * np.sin(t)], [0, rmax * np.cos(t)], color=GRID, lw=0.8, zorder=1)
    # direction labels
    for d, t in zip(DIRS, th):
        lx, ly = -1.16 * rmax * np.sin(t), 1.16 * rmax * np.cos(t)
        ax.text(lx, ly, f"{DIR_LABEL[d]}\n({d}°)", color=TEXT_PRIMARY, fontsize=9.5, ha="center", va="center")
    # series: gray-zone band (safe -> best), then the safe boundary line + markers
    for key, (safe, best, label) in data.items():
        c = COLOR[key]
        sx, sy = xy(safe)
        bx, by = xy(best)
        if np.any(best > safe):
            verts = list(zip(closed(bx), closed(by))) + list(zip(closed(sx)[::-1], closed(sy)[::-1]))
            codes = ([Path.MOVETO] + [Path.LINETO] * len(DIRS)) * 2
            ax.add_patch(PathPatch(Path(verts, codes), facecolor=c, alpha=0.18, edgecolor="none", zorder=2))
        ax.plot(closed(sx), closed(sy), color=c, lw=2.0, zorder=3, label=label)
        ax.plot(sx, sy, "o", color=c, ms=6.5, mec=SURFACE, mew=1.5, zorder=4)
        # selective direct label: the weakest direction only
        i = int(np.argmin(safe))
        ax.annotate(f"{safe[i]:g} N", (sx[i], sy[i]), xytext=(7, -12 if key == "ours" else 6),
                    textcoords="offset points",
                    color=TEXT_PRIMARY, fontsize=9, fontweight="bold", zorder=5)
    ax.set_xlim(-1.35 * rmax, 1.35 * rmax)
    ax.set_ylim(-1.3 * rmax, 1.55 * rmax)
    ax.text(0, 1.42 * rmax, title, color=TEXT_PRIMARY, fontsize=13, fontweight="bold", ha="center", va="bottom")
    ax.text(0, -1.3 * rmax, unit_note, color=TEXT_SECONDARY, fontsize=8.5, ha="center", va="top")


plt.rcParams["font.family"] = "DejaVu Sans"
fig, axes = plt.subplots(1, 2, figsize=(13, 7.4), dpi=200, facecolor=SURFACE)
for ax, typ, rmax, rstep, title, note in [
    (axes[0], "impulse", 60, 10, "Short push (0.2 s)", "Radius: max force withstood (N), 1 N resolution"),
    (axes[1], "sustained", 20, 5, "Sustained push (5 s)", "Radius: max force withstood (N), 0.5 N resolution"),
]:
    p_safe, p_best, _ = boundaries(args.pretrained, typ)
    o_safe, o_best, _ = boundaries(args.ours, typ)
    radar(ax, {"pretrained": (p_safe, p_best, args.pretrained_label), "ours": (o_safe, o_best, args.ours_label)},
          rmax, rstep, title, note)

fig.suptitle("Standing push resistance — Berkeley Humanoid Lite biped (Isaac Lab)",
             color=TEXT_PRIMARY, fontsize=15, fontweight="bold", y=0.985)
fig.text(0.5, 0.935, "Top view, robot facing up. Direction = where the push goes (Forward = pushed from behind). "
         "Line: largest force with no fall at or below it.",
         color=TEXT_SECONDARY, fontsize=9.5, ha="center")
handles, labels = axes[0].get_legend_handles_labels()
handles.append(plt.Rectangle((0, 0), 1, 1, fc=TEXT_SECONDARY, alpha=0.18, ec="none"))
labels.append("Gray zone: fell at some forces above the line, survived at others")
fig.legend(handles, labels, loc="lower center", ncol=3, frameon=False, fontsize=10.5, bbox_to_anchor=(0.5, 0.045),
           labelcolor=TEXT_PRIMARY)
fig.text(0.5, 0.015, "Zero velocity command, nominal conditions (no domain randomization / observation noise). "
         "Horizontal force at the torso CoM (~0.64 m). Fall = tilt > 45° within 5 s of push onset.",
         color=TEXT_SECONDARY, fontsize=8.5, ha="center")
fig.subplots_adjust(left=0.02, right=0.98, top=0.915, bottom=0.12, wspace=0.08)
fig.savefig(args.out, facecolor=SURFACE)
print("saved", args.out)
