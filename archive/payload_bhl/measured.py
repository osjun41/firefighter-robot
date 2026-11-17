"""Put the masses that the team weighs into the robot model.

    uv run python sim/payload/measured.py                     # masses.csv -> models/bhl_F3_measured.xml, results/measured_F3.md
    uv run python sim/payload/measured.py --design F3_1pack   # the same for another design
    uv run python sim/payload/measured.py --template new.csv  # a blank sheet for a design
    uv run python sim/payload/measured.py --check             # self-check, no measurements needed

masses.csv has one row for each thing to weigh. "body" is the model body that the thing moves with.
A whole actuator module (with its drive board) counts in the body that holds its housing.
A torso or leg body takes the sum of its rows, but only when every row of that body is weighed.
Its centre of mass stays where the BHL model has it, and its inertia scales with the mass.
A payload body (the parts of sim/payload/models.py) takes its weighed mass, at its CAD position.
The "whole_robot" row is a check only: the script compares it with the model total.
"""
import argparse
import csv
import os

import mujoco
import numpy as np

import models

HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, "masses.csv")
SIDES = ("left", "right")
# (body suffix, things to weigh). {S} is Left or Right. The actuator housing decides the body (see the README).
LEG = [("hip_roll", ["{S} hip yaw actuator (6512 module with its drive board)",
                     "{S} hip roll link: printed parts, bearings and screws (no actuator)"]),
       ("hip_yaw", ["{S} hip pitch actuator (6512 module with its drive board)",
                    "{S} hip yaw link: printed parts, bearings and screws (no actuator)"]),
       ("hip_pitch", ["{S} knee actuator (6512 module with its drive board)",
                      "{S} thigh: printed parts, bearings and screws (no actuator)",
                      "{S} leg cables (power and CAN) and cable ties"]),
       ("knee_pitch", ["{S} ankle pitch actuator (5010 module with its drive board)",
                       "{S} shin: printed parts, bearings and screws (no actuator)"]),
       ("ankle_pitch", ["{S} ankle link between the two ankle joints: printed parts and screws"]),
       ("ankle_roll", ["{S} ankle roll actuator (5010 module with its drive board)",
                       "{S} foot: printed foot, sole and screws (no actuator)"])]
BASE = ["Left hip roll actuator (6512 module with its drive board)",
        "Right hip roll actuator (6512 module with its drive board)",
        "Torso frame: extrusions, plates, brackets, handles and screws, without the hip roll actuators and the payload rows"]
FIELDS = ["body", "item", "model_g", "measured_g", "note"]


def body_name(body):
    return body if body == "base" or body.startswith("leg_") else "payload_" + body


def template(design):
    """Rows of a blank sheet: torso, legs, payload parts with their CAD mass, and the whole-robot check."""
    s = models.build(design)
    rows = [{"body": "base", "item": t, "model_g": "", "measured_g": "",
             "note": f"body total in the model {s.body('base').mass * 1000:.0f} g"} for t in BASE]
    for side in SIDES:
        for link, things in LEG:
            b = f"leg_{side}_{link}"
            rows += [{"body": b, "item": t.format(S=side.capitalize()), "model_g": "", "measured_g": "",
                      "note": f"body total in the model {s.body(b).mass * 1000:.0f} g"} for t in things]
    rows += [{"body": n, "item": n.replace("_", " "), "model_g": f"{m * 1000:.1f}", "measured_g": "",
              "note": "CAD or datasheet value"} for n, m, _, _ in models.PARTS[design]]
    total = sum(r[1] for r in models.PARTS[design]) - sum(r[1] for r in models.REMOVE[design]) + models.URDF_MASS["biped"]
    rows.append({"body": "whole_robot", "item": "Whole robot on a scale, everything mounted", "model_g": f"{total * 1000:.0f}",
                 "measured_g": "", "note": "check only"})
    return rows


def read(path=CSV):
    with open(path, newline="") as f:
        return [r for r in csv.DictReader(f) if r["body"].strip() and not r["body"].startswith("#")]


def grams(v):
    return float(v) if v and v.strip() else None


def scale(b, r):
    """Multiply the mass and the inertia of an MjSpec body by r. The centre of mass stays."""
    b.mass *= r
    if np.isnan(b.fullinertia[0]):
        b.inertia = [v * r for v in b.inertia]
    else:
        b.fullinertia = [v * r for v in b.fullinertia]


def apply(s, rows):
    """Set the weighed masses on the MjSpec s. Returns (body, model g, new g or None, note) for each body."""
    bodies = {}
    for r in rows:
        if r["body"] != "whole_robot":
            bodies.setdefault(r["body"], []).append(grams(r["measured_g"]))
    out = []
    for body, vals in bodies.items():
        b = s.body(body_name(body))
        if b is None:
            raise SystemExit(f"masses.csv: the model has no body {body!r}")
        model = b.mass * 1000
        missing = vals.count(None)
        if missing:
            out.append((body, model, None, f"{missing} of {len(vals)} rows not weighed, model value kept"))
            continue
        new = sum(vals)
        if new <= 0:
            raise SystemExit(f"masses.csv: {body} sums to {new} g")
        scale(b, new / model)
        note = "check the rows: more than 20 % off" if abs(new / model - 1) > 0.2 else ""
        out.append((body, model, new, note))
    return out


def stats(m):
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    feet = (d.body("leg_left_ankle_roll").xpos + d.body("leg_right_ankle_roll").xpos) / 2
    return m.body_subtreemass[1], d.subtree_com[1] - [feet[0], feet[1], 0]   # x, y from the middle of the ankles


def report(design, out, rows, m0, m1):
    (t0, c0), (t1, c1) = stats(m0), stats(m1)
    lines = [f"# Measured masses, design {design}", "",
             f"Made by `sim/payload/measured.py` from `sim/payload/masses.csv`. Model: `models/bhl_{design}_measured.xml`.", "",
             "| Body | Model (g) | Weighed (g) | Change | Note |", "|---|---|---|---|---|"]
    for body, model, new, note in out:
        ch = f"{(new / model - 1) * 100:+.1f} %" if new is not None else "-"
        lines.append(f"| {body} | {model:.0f} | {new:.0f} | {ch} | {note} |" if new is not None
                     else f"| {body} | {model:.0f} | - | - | {note} |")
    lines += ["", f"Robot mass: {t0:.3f} kg in the CAD model, {t1:.3f} kg with the weighed masses.",
              f"Centre of mass height (stand pose): {c0[2]:.3f} m, now {c1[2]:.3f} m. "
              f"Forward of the middle of the ankles: {c0[0] * 1000:+.0f} mm, now {c1[0] * 1000:+.0f} mm."]
    whole = [grams(r["measured_g"]) for r in rows if r["body"] == "whole_robot"]
    if whole and whole[0]:
        lines.append(f"Whole robot on the scale: {whole[0] / 1000:.3f} kg. "
                     f"The model with the weighed masses differs by {(t1 - whole[0] / 1000) * 1000:+.0f} g.")
    return "\n".join(lines) + "\n"


def build(design, rows):
    s = models.build(design)
    out = apply(s, rows)
    return s, out


def self_check():
    design = "F3"
    m0 = models.build(design).compile()
    # 1. every body weighed at its model mass: nothing changes
    s = models.build(design)
    rows = [{"body": b, "measured_g": f"{s.body(body_name(b)).mass * 1000!r}"}
            for b in ["base"] + [f"leg_{sd}_{k}" for sd in SIDES for k, _ in LEG] + [n for n, *_ in models.PARTS[design]]]
    apply(s, rows)
    assert abs(s.compile().body_subtreemass[1] - m0.body_subtreemass[1]) < 1e-9
    # 2. one thigh 10 % heavier in two rows: the total grows by 10 % of the thigh, the thigh keeps its centre of mass
    thigh = models.build(design).body("leg_left_hip_pitch").mass * 1000
    s = models.build(design)
    out = apply(s, [{"body": "leg_left_hip_pitch", "measured_g": f"{thigh * 0.6}"},
                    {"body": "leg_left_hip_pitch", "measured_g": f"{thigh * 0.5}"}])
    m1 = s.compile()
    i = m1.body("leg_left_hip_pitch").id
    assert abs(m1.body_subtreemass[1] - m0.body_subtreemass[1] - thigh * 0.1e-3) < 1e-9
    assert np.allclose(m1.body_ipos[i], m0.body_ipos[i]) and np.allclose(m1.body_inertia[i], 1.1 * m0.body_inertia[i])
    # 3. a body with a row not weighed keeps its model mass
    s = models.build(design)
    out = apply(s, [{"body": "leg_left_knee_pitch", "measured_g": "300"}, {"body": "leg_left_knee_pitch", "measured_g": ""}])
    assert out[0][2] is None and abs(s.compile().body_subtreemass[1] - m0.body_subtreemass[1]) < 1e-9
    # 4. the template has a row for every model body
    assert {r["body"] for r in template(design)} >= {"base", "whole_robot", "leg_right_ankle_roll", "relay_aep18024"}
    print("self-check measured masses: passed")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--design", default="F3", help="F3 (default), F3_1pack, F3_tether, F2, ...")
    ap.add_argument("--csv", default=CSV)
    ap.add_argument("--template", metavar="OUT", help="write a blank sheet for --design and stop")
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    if a.check:
        return self_check()
    if a.template:
        if os.path.exists(a.template):
            raise SystemExit(f"{a.template} exists. Choose another file name, so no weighed value is lost.")
        with open(a.template, "w", newline="") as f:
            w = csv.DictWriter(f, FIELDS)
            w.writeheader()
            w.writerows(template(a.design))
        return print(f"wrote {a.template}")
    rows = read(a.csv)
    s, out = build(a.design, rows)
    variant = f"{a.design}_measured"
    models.save(s, variant)
    m0, m1 = models.load(a.design), models.load(variant)
    text = report(a.design, out, rows, m0, m1)
    os.makedirs(os.path.join(HERE, "results"), exist_ok=True)
    with open(os.path.join(HERE, "results", f"measured_{a.design}.md"), "w") as f:
        f.write(text)
    print(text, f"wrote {os.path.relpath(models.path(variant), HERE)}", sep="")


if __name__ == "__main__":
    main()
