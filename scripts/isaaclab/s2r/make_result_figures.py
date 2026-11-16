"""Key figures of the dynamics-gap study (English) from the final evaluation, as small PNG files.

DR000 to DR200 share one hue: grey (no randomization) to saturated red (widest randomization).
Run from the repository root:  uv run --project scripts/isaaclab python scripts/isaaclab/s2r/make_result_figures.py [out_dir]
"""

import colorsys
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(ROOT, "logs", "s2r_study"))
import build_page  # noqa: E402

build_page.EVAL_DIR = os.path.join(ROOT, "logs", "s2r_study", "eval_final")
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "logs", "s2r_study", "figures")

WIDTHS = [("DR000", 0.0), ("DR050", 0.5), ("DR100", 1.0), ("DR200", 2.0)]


def width_color(alpha):
    """Grey at alpha 0 to saturated red at alpha 2 (one hue, saturation grows with the width)."""
    s = {0.0: 0.0, 0.5: 0.35, 1.0: 0.7, 2.0: 1.0}[alpha]
    v = {0.0: 0.55, 0.5: 0.72, 1.0: 0.80, 2.0: 0.72}[alpha]
    return colorsys.hsv_to_rgb(0.0, s, v)


COLORS = {name: width_color(a) for name, a in WIDTHS}
LABELS = {"DR000": "DR000 (α = 0, fixed)", "DR050": "DR050 (α = 0.5)", "DR100": "DR100 (α = 1)", "DR200": "DR200 (α = 2)"}
PARAMS = {
    "torso_mass_add": ("Torso mass change (kg)", 1.0, 0.0, 1.5),
    "leg_mass_scale": ("Leg link mass & inertia scale (×)", 1.0, 1.0, 0.2),
    "friction": ("Foot–ground friction coefficient μ (–)", 1.0, 0.8, 0.4),
    "torso_com_x": ("Torso CoM offset, x forward (cm)", 100.0, 0.0, 0.03),
    "torso_com_z": ("Torso CoM offset, z up (cm)", 100.0, 0.0, 0.03),
}
NOTE = "Isaac Lab, Berkeley Humanoid Lite biped, policies after 12000 iterations (seed 42)."

plt.rcParams.update({"font.size": 10, "axes.titlesize": 11, "axes.labelsize": 10, "legend.fontsize": 9,
                     "axes.spines.top": False, "axes.spines.right": False, "figure.dpi": 100})


def save(fig, name):
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, name)
    fig.savefig(path, dpi=100, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    # palette PNG keeps the files small enough for e-mail
    Image.open(path).convert("RGB").quantize(colors=128, method=Image.Quantize.MEDIANCUT).save(path, optimize=True)
    print(f"{path}: {os.path.getsize(path) / 1024:.0f} KB")


def pct(v):
    return 100.0 * v


def fig_single_axis(res):
    fig, axes = plt.subplots(2, 3, figsize=(12, 6.6))
    for ax, (key, (label, scale, nominal, half)) in zip(axes.flat, PARAMS.items()):
        lo = nominal - half if key != "friction" else max(0.1, nominal - half)
        ax.axvspan((lo) * scale, (nominal + half) * scale, color="0.92", zorder=0)
        ax.axvline(nominal * scale, color="0.4", ls="--", lw=1)
        for name, _ in WIDTHS:
            pts = res[name]["sweeps"][key]
            ax.plot([p["x"] * scale for p in pts], [pct(p["fell"]) for p in pts], "-o", ms=3.5, lw=1.8,
                    color=COLORS[name], label=LABELS[name])
        ax.set_xlabel(label)
        ax.set_ylabel("Fall rate (%)")
        ax.set_ylim(-3, 103)
    ax = axes.flat[5]
    ax.axis("off")
    handles, labels = axes.flat[0].get_legend_handles_labels()
    ax.legend(handles, labels, loc="upper left", frameon=False, title="Training randomization width")
    ax.text(0.0, 0.42, "One parameter changed at a time,\nothers at their nominal values.\n"
            "32 runs per point (8 velocity-command\nprofiles × 4, 20 s each).\n"
            "Fall: torso tilt over 45°.\nDashed line: training (nominal) value.\n"
            "Grey band: training range at α = 1.", transform=ax.transAxes, va="top", fontsize=9.5)
    fig.suptitle("Fall rate vs. a single dynamics gap (real − model)", fontsize=13, fontweight="bold")
    fig.text(0.5, -0.01, NOTE, ha="center", fontsize=8.5, color="0.35")
    fig.tight_layout()
    save(fig, "fig1_single_parameter_gaps.png")


def fig_combined(res):
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.4), sharey=True)
    for ax, key, title in [(axes[0], "combined", "Combined gap, nominal robot"),
                           (axes[1], "payload_combined", "Combined gap + 1 kg upper-body payload")]:
        for name, _ in WIDTHS:
            e = res[name]
            start = e["nominal"]["fell"] if key == "combined" else e["payload"]["fell"]
            betas = sorted(e[key], key=float)
            ax.plot([0.0] + [float(b) for b in betas], [pct(start)] + [pct(e[key][b]["fell"]) for b in betas], "-o",
                    ms=4, lw=2, color=COLORS[name], label=LABELS[name])
        ax.set_title(title)
        ax.set_xlabel("Gap size β (all five parameters drawn from the α = β training box)")
        ax.set_xticks([0, 0.5, 1, 1.5, 2, 3])
        ax.grid(axis="y", color="0.9")
    axes[0].set_ylabel("Fall rate (%)")
    axes[0].legend(frameon=False, loc="upper left")
    fig.suptitle("Fall rate when all dynamics are wrong at once (768 runs per point)", fontsize=13, fontweight="bold")
    fig.text(0.5, -0.04, "β = 1: torso ±1.5 kg, legs ±20%, CoM ±3 cm, μ 0.4–1.2 (random combination). "
             "Payload: 1 kg box on top of the torso (torso CoM +4.9 cm). " + NOTE, ha="center", fontsize=8.5,
             color="0.35", wrap=True)
    fig.tight_layout()
    save(fig, "fig2_combined_gaps.png")


def fig_tolerance(res):
    keys = ["torso_mass_add", "leg_mass_scale", "friction", "torso_com_x"]
    tested = {"torso_mass_add": (-2, 5), "leg_mass_scale": (0.5, 1.6), "friction": (0.1, 2.0), "torso_com_x": (-10, 10)}
    fig, axes = plt.subplots(2, 2, figsize=(11, 5.6))
    for ax, key in zip(axes.flat, keys):
        label, scale, nominal, half = PARAMS[key]
        a, b = tested[key]
        for i, (name, alpha) in enumerate(WIDTHS):
            y = len(WIDTHS) - 1 - i
            e = res[name]["envelope"][key]
            if e is None:
                continue
            lo, hi = e["lo"] * scale, e["hi"] * scale
            ax.plot([lo, hi], [y, y], lw=9, color=COLORS[name], solid_capstyle="butt")
            for end, open_ in [(lo, e["lo_open"]), (hi, e["hi_open"])]:
                if open_:
                    ax.annotate("", xy=(end + (0.04 if end == hi else -0.04) * (b - a), y), xytext=(end, y),
                                arrowprops=dict(arrowstyle="-|>", color=COLORS[name], lw=2))
            text = f"{lo:g} … {hi:g}" if key != "leg_mass_scale" else f"×{lo:g} … ×{hi:g}"
            ax.text(b + 0.06 * (b - a), y, text, va="center", fontsize=8.5)
        ax.axvline(nominal * scale, color="0.4", ls="--", lw=1)
        ax.set_yticks(range(len(WIDTHS)))
        ax.set_yticklabels([LABELS[n].split(" (")[0] for n, _ in reversed(WIDTHS)])
        ax.set_xlim(a - 0.06 * (b - a), b + 0.28 * (b - a))
        ax.set_xlabel(label)
        ax.spines["left"].set_visible(False)
        ax.tick_params(axis="y", length=0)
    fig.suptitle("Tolerated single-parameter gap (fall rate ≤ 5%, 32 runs per value)", fontsize=13, fontweight="bold")
    fig.text(0.5, -0.03, "Bars: contiguous range around the nominal value (dashed) with at most 1 fall in 32 runs; "
             "arrows: held to the end of the tested range. CoM z: no effect from −6 to +12 cm for all policies.\n"
             + NOTE, ha="center", fontsize=8.5, color="0.35")
    fig.tight_layout()
    save(fig, "fig4_tolerated_gaps.png")


def fig_convergence(res):
    iters = [3000, 6000, 9000, 12000]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    for name, _ in WIDTHS:
        rows = [res[f"{name}@{it}"] if it != 12000 else res[name] for it in iters]
        axes[0].plot(iters, [r["nominal"]["ang_err"] for r in rows], "-o", ms=4, lw=2, color=COLORS[name],
                     label=LABELS[name])
        axes[1].plot(iters, [pct(r["combined"]["2"]["fell"]) for r in rows], "-o", ms=4, lw=2, color=COLORS[name])
    axes[0].set_ylabel("Yaw-rate tracking error on nominal dynamics (rad/s)")
    axes[1].set_ylabel("Fall rate at combined gap β = 2 (%)")
    for ax in axes:
        ax.set_xlabel("Training iterations")
        ax.set_xticks(iters)
        ax.grid(color="0.92")
    axes[0].legend(frameon=False)
    fig.suptitle("Convergence: robustness settles by 3000 iterations, tracking quality by ~9000 (α ≤ 1)",
                 fontsize=12.5, fontweight="bold")
    fig.text(0.5, -0.03, NOTE.replace("policies after 12000 iterations", "checkpoints of the same runs"),
             ha="center", fontsize=8.5, color="0.35")
    fig.tight_layout()
    save(fig, "fig5_convergence.png")


def fig_ablation(res):
    def sweep(name, key, value):
        return next(p for p in res[name]["sweeps"][key] if abs(p["x"] - value) < 1e-6)["fell"]

    cases = [("Torso +2 kg", lambda n: sweep(n, "torso_mass_add", 2.0)),
             ("Legs ×1.4", lambda n: sweep(n, "leg_mass_scale", 1.4)),
             ("Friction μ = 0.15", lambda n: sweep(n, "friction", 0.15)),
             ("Combined β = 1.5", lambda n: res[n]["combined"]["1.5"]["fell"]),
             ("Payload + β = 1", lambda n: res[n]["payload_combined"]["1"]["fell"])]
    policies = [("DR000@6000", "No randomization", COLORS["DR000"]),
                ("Friction100", "Friction only (α = 1)", (0.55, 0.75, 0.95)),
                ("Inertia100", "Mass / inertia / CoM only (α = 1)", (0.20, 0.40, 0.70)),
                ("DR100@6000", "Both (α = 1)", COLORS["DR100"])]
    fig, ax = plt.subplots(figsize=(11, 4.2))
    x = np.arange(len(cases))
    w = 0.2
    for i, (name, label, color) in enumerate(policies):
        vals = [pct(f(name)) for _, f in cases]
        bars = ax.bar(x + (i - 1.5) * w, vals, w, color=color, label=label)
        for bar, v in zip(bars, vals):
            ax.text(bar.get_x() + bar.get_width() / 2, v + 1.2, f"{v:.0f}", ha="center", fontsize=8)
    ax.set_xticks(x)
    ax.set_xticklabels([c for c, _ in cases])
    ax.set_ylabel("Fall rate (%)")
    ax.set_ylim(0, max(70, ax.get_ylim()[1]))
    ax.legend(frameon=False, ncol=2, loc="upper right")
    ax.grid(axis="y", color="0.92")
    fig.suptitle("What to randomize: mass / inertia / CoM gives almost all of the robustness", fontsize=12.5,
                 fontweight="bold")
    fig.text(0.5, -0.03, "All four policies after 6000 iterations (same budget). 32 runs per single-parameter value, "
             "768 per combined gap.", ha="center", fontsize=8.5, color="0.35")
    fig.tight_layout()
    save(fig, "fig6_what_to_randomize.png")


def goal_data():
    """Per policy and trial: values of the goal-reaching evaluation (eval_goal_reaching.py), runs without a fall."""
    import csv
    import glob

    data = {}
    for path in glob.glob(os.path.join(ROOT, "logs", "s2r_study", "eval_goal", "*.csv")):
        rows = [r for r in csv.DictReader(open(path)) if r["fell"] == "0"]
        trials = {}
        for r in rows:
            trials.setdefault(r["trial"], []).append(r)
        data[os.path.basename(path)[:-4]] = trials
    return data


def mean_ci(values):
    v = np.array(values, dtype=float)
    return v.mean(), 1.96 * v.std(ddof=1) / np.sqrt(len(v)) if len(v) > 1 else 0.0


GOAL_POLICIES = [(n, COLORS[n], LABELS[n], "-", "o") for n, _ in WIDTHS]
WALKS = ["forward 0.20 m/s", "forward 0.35 m/s", "forward 0.50 m/s", "backward 0.20 m/s", "backward 0.35 m/s"]
TURNS = ["turn left 0.5 rad/s", "turn right 0.5 rad/s", "turn left 1.0 rad/s", "turn right 1.0 rad/s"]


def fig_goal_reaching():
    g = goal_data()
    fig, axes = plt.subplots(2, 2, figsize=(12, 8.4))
    panels = [(axes[0, 0], WALKS, "distance_err_pct", "Distance error after the stop (%)\n(− fell short, + overshot)",
               "Walk 4 s, then stop 2 s"),
              (axes[0, 1], WALKS, "lateral_cm", "Sideways drift after the stop (cm)", "Walk 4 s, then stop 2 s"),
              (axes[1, 0], TURNS, "turn_err_deg", "Turn angle error after the stop (deg)\n(− turned less, + turned more)",
               "Turn in place 3 s from standing, then stop 2 s")]
    offsets = np.linspace(-0.2, 0.2, len(GOAL_POLICIES))
    for ax, trials, key, ylabel, title in panels:
        x = np.arange(len(trials))
        for off, (name, color, label, ls, marker) in zip(offsets, GOAL_POLICIES):
            stats = [mean_ci([abs(float(r[key])) if key == "lateral_cm" else float(r[key]) for r in g[name][t]])
                     for t in trials]
            ax.errorbar(x + off, [m for m, _ in stats], yerr=[c for _, c in stats], fmt=marker, ms=5, color=color,
                        ecolor=color, elinewidth=1.2, capsize=2, label=label)
        ax.axhline(0, color="0.5", lw=0.8)
        ax.set_xticks(x)
        ax.set_xticklabels([t.replace(" m/s", "\nm/s").replace(" rad/s", "\nrad/s") for t in trials], fontsize=8.5)
        ax.set_ylabel(ylabel)
        ax.set_title(title, fontsize=10.5)
        ax.grid(axis="y", color="0.92")
    # standing
    ax = axes[1, 1]
    names = [p[0] for p in GOAL_POLICIES]
    sway = [mean_ci([float(r["sway_rms"]) for r in g[n]["stand"]]) for n in names]
    drift = [mean_ci([float(r["base_drift_cm"]) for r in g[n]["stand"]]) for n in names]
    x = np.arange(len(names))
    ax.bar(x - 0.2, [m for m, _ in sway], 0.38, yerr=[c for _, c in sway], color=[p[1] for p in GOAL_POLICIES],
           capsize=2, label="Torso sway (rad/s, left axis)")
    ax.set_ylabel("Torso sway while standing:\nRMS roll + pitch + yaw rate (rad/s)")
    ax2 = ax.twinx()
    ax2.bar(x + 0.2, [m for m, _ in drift], 0.38, yerr=[c for _, c in drift], color="white",
            edgecolor=[p[1] for p in GOAL_POLICIES], hatch="///", capsize=2, label="Base drift (cm, right axis)")
    ax2.set_ylabel("Base drift while standing 5 s (cm)")
    ax2.spines["right"].set_visible(True)
    ax.set_xticks(x)
    ax.set_xticklabels(names)
    ax.set_title("Stand 5 s (spraying posture, no load)", fontsize=10.5)
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, frameon=False, fontsize=8.5, loc="upper left")
    ax.set_ylim(0, max(m for m, _ in sway) * 1.45)
    ax2.set_ylim(0, max(m for m, _ in drift) * 1.45)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.suptitle("Goal-reaching quality on the nominal dynamics (mean ± 95% CI, 32 runs per trial)", fontsize=13,
                 fontweight="bold", y=1.03)
    fig.legend(handles, labels, loc="upper center", ncol=len(labels), frameon=False, fontsize=9, bbox_to_anchor=(0.5, 1.0))
    fig.text(0.5, -0.03, "Errors accumulated over the whole maneuver (where the robot ends up), not instantaneous. "
             "Walks start and end standing, so some shortfall comes from accelerating and stopping.\n" + NOTE,
             ha="center", fontsize=8.5, color="0.35")
    fig.tight_layout()
    save(fig, "fig7_goal_reaching.png")


def fig_tradeoff_goal(res):
    g = goal_data()

    def walk_err(n):
        return np.mean([abs(float(r["distance_err_pct"])) for t in WALKS for r in g[n][t]])

    def turn_err(n):
        return np.mean([abs(float(r["turn_err_deg"])) for t in TURNS for r in g[n][t]])

    fig, axes = plt.subplots(2, 2, figsize=(11, 8.2), sharey="row")
    cases = [("combined", "1.5", "Combined gap β = 1.5"), ("payload_combined", "1", "1 kg payload + combined gap β = 1")]
    metrics = [(walk_err, "Walk distance error, mean |error| (%)", "walking"),
               (turn_err, "Turn-in-place angle error, mean |error| (deg)", "turning")]
    for row, (key, beta, title) in enumerate(cases):
        for col, (fn, xlabel, short) in enumerate(metrics):
            ax = axes[row, col]
            xs = [fn(n) for n, _ in WIDTHS]
            ys = [pct(res[n][key][beta]["fell"]) for n, _ in WIDTHS]
            ax.plot(xs, ys, "-", color="0.8", lw=1.2, zorder=1)
            for (name, a), x, y in zip(WIDTHS, xs, ys):
                ax.scatter([x], [y], s=90, color=COLORS[name], zorder=2, edgecolor="white", linewidth=1)
                ax.annotate(f"α = {a:g}", (x, y), textcoords="offset points", xytext=(7, 6), fontsize=9.5)
            ax.set_title(f"{title}  vs  {short}", fontsize=10.5)
            ax.set_xlabel(xlabel)
            ax.grid(color="0.92")
            lo, hi = min(xs), max(xs)
            ax.set_xlim(lo - 0.1 * (hi - lo), hi + 0.22 * (hi - lo))
        axes[row, 0].set_ylabel("Fall rate (%)")
    fig.suptitle("Generality vs. goal-reaching quality (lower left is better)", fontsize=13, fontweight="bold")
    fig.text(0.5, -0.03, "Quality: nominal dynamics, walks forward/backward 0.2–0.5 m/s for 4 s and turns in place "
             "0.5–1.0 rad/s for 3 s from standing (32 runs each), error after stopping. Fall rates: 768 runs per point.\n"
             + NOTE, ha="center", fontsize=8.5, color="0.35")
    fig.tight_layout()
    save(fig, "fig3_tradeoff.png")


def main():
    res = build_page.evaluations()
    fig_single_axis(res)
    fig_combined(res)
    fig_tolerance(res)
    fig_convergence(res)
    fig_ablation(res)
    fig_goal_reaching()
    fig_tradeoff_goal(res)


if __name__ == "__main__":
    main()
