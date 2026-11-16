"""Print the gap evaluation results as text tables (fall rate and tracking error per test condition).

Usage (from the repository root):
    python3 scripts/isaaclab/s2r/summarize_eval.py logs/s2r_study/eval DR000 DR100 ...
"""

import csv
import os
import sys
from collections import defaultdict

PARAMS = ["friction", "torso_mass_add", "leg_mass_scale", "torso_com_x", "torso_com_z"]
NOMINAL = {"friction": 0.8, "torso_mass_add": 0.0, "leg_mass_scale": 1.0, "torso_com_x": 0.0, "torso_com_z": 0.0}


def load(directory, name):
    with open(os.path.join(directory, f"{name}.csv")) as f:
        return list(csv.DictReader(f))


def stats(rows):
    n = len(rows)
    mean = lambda key: sum(float(r[key]) for r in rows) / n
    return (f"fall {100 * mean('fell'):5.1f}%  lin {mean('lin_err'):.3f}  ang {mean('ang_err'):.3f}  "
            f"ret {mean('return'):6.2f}  P {mean('power_W'):5.1f}  da {mean('action_change'):.3f}  (n={n})")


def main():
    directory, names = sys.argv[1], sys.argv[2:]
    for name in names:
        rows = load(directory, name)
        print(f"===== {name}")
        nominal = [r for r in rows if r["group"].startswith("sweep")
                   and all(abs(float(r[p]) - NOMINAL[p]) < 1e-6 for p in PARAMS)]
        print(f"  nominal            {stats(nominal)}")
        for p in PARAMS:
            groups = defaultdict(list)
            for r in rows:
                if r["group"] == f"sweep:{p}":
                    groups[float(r[p])].append(r)
            for value in sorted(groups):
                print(f"  {p:15s} {value:+7.3f} {stats(groups[value])}")
        groups = defaultdict(list)
        for r in rows:
            if not r["group"].startswith("sweep"):
                groups[r["group"]].append(r)
        order = lambda g: (g.split(":")[0], float(g.split(":")[1]) if ":" in g else -1.0)
        for group in sorted(groups, key=order):
            print(f"  {group:23s} {stats(groups[group])}")


if __name__ == "__main__":
    main()
