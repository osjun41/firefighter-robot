# Payload check (ASMR robot, F3 parts) and the BHL payload models

`final_check.py` checks that the ASMR robot can carry the F3 parts: statics, walking and pushes, with the policy chosen by name
(default `baseline`) and the payload options `none`, `box_1kg` and `f3` of `source/robot_models/robots/asmr2026_payload.py`.
It runs the ASMR runner `sim/sim2sim/run.py`.

`models.py` builds the older MuJoCo models of the Berkeley Humanoid Lite (BHL) biped with the payload of the designs F2 and F3.
No script uses them any more (the mission moved to the ASMR robot); they are deleted in the last step of the model unification.

**Status:** tested in simulation (MuJoCo 3.15.0, CPU only). Nothing here is measured on board. **The check has not been run on the ASMR
robot yet** (only its quick self-check): the old F2 results of the BHL model are in `archive/payload_f2/`.

## How to run

Run these commands from the repository root.

1. Run the quick check (about 10 s):
   ```
   uv run python sim/payload/final_check.py check
   ```
2. Run the full check (about 10-20 min on 8 cores). It writes `results/final_f3.md` and `results/final_f3.json`.
   `--policy NAME` takes another folder of `policies/`:
   ```
   uv run python sim/payload/final_check.py
   uv run python sim/payload/final_check.py --policy NAME
   ```
3. `MUJOCO_GL=egl uv run python sim/payload/final_check.py video` films one walk and one push (needs ffmpeg). The videos stay local.

No BHL model is needed for the checks above. To build the old BHL models anyway, fetch the meshes with `scripts/fetch_bhl_assets.sh` and run
`uv run python sim/payload/models.py` (about 5 s).

## What `models.py` builds

`models.py` reads the parts of `cad/final/out_f2/layout.json` and `cad/final/out_f3/layout.json` (made by `cad/final/layout.py`).
Each part is a rigid body on the BHL torso, with a solid-box inertia. The water of both tanks is one body with the same inertia.
The BHL battery (603 g) and PC (280 g) are not on our robot, so `models.py` takes their mass out of the BHL torso.

| Model file (`models/`) | Content | Mass (kg) |
|---|---|---|
| `bhl_biped.xml` | The bare BHL biped | 11.34 |
| `bhl_F2.xml` | F2 with both tanks full (0.781 L) | 15.99 |
| `bhl_F2_half.xml` | F2 with both tanks half full | 15.60 |
| `bhl_F2_empty.xml` | F2 with empty tanks | 15.21 |
| `bhl_F3.xml` | F3, the trolley version: no water system on the robot, the end of the hose (anchor, signal plug, hose) on the deck | 14.60 |
| `bhl_F3_1pack.xml` | F3 with one 6S pack instead of two | 14.00 |
| `bhl_F3_tether.xml` | F3 with no 6S pack: motor power by cable from the trolley | 13.46 |

Each model has a "stand" keyframe: flat feet, 1 mm above the floor. The self-check of `models.py` compares every mass with the parts table.

**Assumed:** the hose from the trolley adds no load other than the 20 g of hose and water in the F3 layout. The gap study tests a hose pull (`sim/sim2sim/gap_study.py`).

## Weighed masses (`measured.py`)

The CAD model guesses many masses. The team weighs the real parts, and `measured.py` puts them into the model.

1. Weigh each part with a scale of 1 g resolution. Write the mass in grams in the `measured_g` column of [`masses.csv`](masses.csv).
2. Weigh each actuator module with its drive board before you mount it. It counts in the body that holds its housing. For example, the knee actuator counts in the thigh (`leg_left_hip_pitch`).
3. Weigh the printed parts, bearings and screws of each link together, in the row of that link.
4. Weigh the whole robot at the end, in the `whole_robot` row. This row is only a check.
5. When a body has a row with no mass, the model keeps its old mass for that body. You can fill the sheet step by step.
6. Build the model with the weighed masses:
   ```
   uv run python sim/payload/measured.py
   ```
   It writes `models/bhl_F3_measured.xml` and `results/measured_F3.md` (body, model mass, weighed mass, change).

- A torso or leg body keeps the centre of mass of the BHL model. Its inertia scales with its mass.
- A row that is more than 20 % off the model gets a note. Check it before you trust it: a part may be in the wrong row.
- `--design F3_1pack` uses another design. `--template new.csv` writes a blank sheet. `--check` runs the self-check.

## Results

**Not run yet on the ASMR robot.** `final_check.py` writes the tables (statics, walking at the MJCF torque limit and at 12 Nm,
push test) for the bare robot, the robot with the 1 kg box and the robot with the F3 parts to `results/final_f3.md`.
Read them and write the findings into its last section and here.

The old F2 check (BHL biped with F2 parts, shipped BHL policy, 6 / 9 Nm) is in `archive/payload_f2/final_f2.md`. It does not describe this robot:
the ASMR model has other masses, gains and torque limits, and F3 has no water.

## Files

| File | What it is |
|---|---|
| `models.py` | Builds `models/bhl_biped.xml` and `models/bhl_F*.xml` (BHL biped, used by `sim/mission` only), then runs its self-check |
| `poses.py` | Leg poses (ASMR: the right pitch joints turn the other way) and flat-foot placement on the floor |
| `analyze.py` | Static joint torques and centre of mass of the ASMR robot (stand, crouch, one leg) |
| `torso_tank.py` | Walking and push test helpers on the ASMR runner (`run.simulate` with a push). `final_check.py` uses them. |
| `report.py` | Plot palette and style of the simulation figures |
| `final_check.py` | The check of the ASMR robot with the F3 parts: statics, walking and push test (the drive cases are in `sim/sim2sim/hw_risks.py`) |
| `measured.py`, `masses.csv` | The sheet of parts to weigh, and the script that puts the weighed masses into the model |
| `results/final_f3.md`, `results/final_f3.json` | Results of `final_check.py` (written by the run) |

## Open points

- **Open:** the model uses the 130 g DWP-385 pump of the CAD layout. The KLP02 weighs about 250 g, so the robot will be about 0.12 kg heavier, plus the DMC-101.
- **Not verified:** the part masses are CAD and datasheet values. Weigh the robot when it is built.
- **Not verified:** the model has no motor dynamics, no CAN jitter and a flat floor only.
- **Assumed:** the motor phase resistance is 0.1 Ω for the power numbers.

## Licence

Code: MIT (root `LICENSE`). This README and the result files: CC BY-SA 4.0.
The BHL meshes and URDF are CC BY-SA 4.0. `scripts/fetch_bhl_assets.sh` fetches them. They are not in git.
