# Payload models and the F2 payload check

This folder builds the MuJoCo models of the Berkeley Humanoid Lite (BHL) biped with the payload of the final design F2.
It also checks that the robot can carry this payload: statics, walking, pushes and the motor drive model, with the shipped BHL policy.
`sim/sim2sim`, `sim/mission` and `docs/hardware/power_calc.py` load these models.

**Status:** tested in simulation (MuJoCo 3.15.0, CPU only). Nothing here is measured on board.

## How to run

Run these commands from the repository root.

1. Fetch the BHL meshes and URDF (not in git):
   ```
   scripts/fetch_bhl_assets.sh
   ```
2. Build the models. The XML files are not in git, so every other simulation needs this step first (about 5 s):
   ```
   uv run python sim/payload/models.py
   ```
3. Run the quick check (about 10 s):
   ```
   uv run python sim/payload/final_check.py check
   ```
4. Run the full F2 check (about 10 min on 8 cores). It writes `results/final_f2.md` and `results/final_f2.json`:
   ```
   uv run python sim/payload/final_check.py
   ```

`MUJOCO_GL=egl uv run python sim/payload/final_check.py video` films one walk and one push (needs ffmpeg). The videos stay local.

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

## Results of the F2 check

Status: tested in simulation. Policy: the shipped BHL policy `sim/sim2sim/policy_biped_25hz_a.onnx`. Full tables: [results/final_f2.md](results/final_f2.md).

Statics (stand pose):

| Model | Mass (kg) | CoM height (m) | CoM x / y (mm) | Peak torque, stand (Nm) | Peak pitch torque on one leg (Nm) |
|---|---|---|---|---|---|
| bare biped | 11.34 | 0.362 | −2 / 0 | 1.5 | 4.3 |
| F2, tanks full | 15.99 | 0.448 | +1 / 0 | 2.5 | 6.6 |
| F2, tanks half | 15.60 | 0.443 | 0 / 0 | 2.4 | 6.3 |
| F2, tanks empty | 15.21 | 0.439 | −1 / 0 | 2.3 | 6.1 |

Walking, 20 s with the training noise (speed reached at each command):

| Model | Limit | vx 0.3 m/s | vx 0.2 m/s + yaw 0.5 rad/s |
|---|---|---|---|
| F2, tanks full | 6 Nm | 0.21 m/s | 0.11 m/s, 0.44 rad/s |
| F2, tanks full | 9 Nm | 0.28 m/s | 0.13 m/s, 0.47 rad/s |
| F2, tanks half | 6 Nm | 0.21 m/s | falls at 10.2 s |
| F2, tanks half | 9 Nm | 0.29 m/s | 0.15 m/s, 0.51 rad/s |

Largest push with no fall, F2 with full tanks (worst of 3 push times):

| Robot state | Forward | Back | Left | Right |
|---|---|---|---|---|
| Standing | 3.8 N s | 5.6 N s | 4.9 N s | 4.5 N s |
| Walking at 0.3 m/s | 9.4 N s | 5.2 N s | 3.0 N s | 3.0 N s |

Drive model of `sim/sim2sim/hw_risks.py` (4 commands × 10 seeds of 20 s). **Old result:** `hw_risks.py` ran the BHL model with F2 when this table was made. It now runs the ASMR robot with the F3 parts, so these numbers are not reproduced by the script any more (the old report is in `archive/sim2sim_bhl/`).:

| Case | Falls | Mean motor bus power, walking |
|---|---|---|
| 1: K 20, D 2, 6 Nm (as trained) | 1 of 40 | 22 W |
| 7: as 1, with ±2° zero errors and 10 ms command latency | 1 of 40 | 22 W |
| 7b: as 7, with 10 ms observation latency | 7 of 40 | 22 W |

Findings:

1. The payload adds 4.64 kg and lifts the centre of mass from 0.362 m to 0.448 m. It stays within 1 mm of the middle.
2. On one leg, the pitch torque reaches 6.6 Nm. This is above the 6 Nm training limit. A 9 Nm limit gives margin.
3. At 6 Nm, the robot walks at 0.21 m/s for a 0.3 m/s command. At 9 Nm it walks at 0.28 m/s and does not fall.
4. The weakest push is 3.0 N s to the side while walking.
5. Observation latency matters: 10 ms more gives 7 falls in 40 runs instead of 1.
6. The walking power (22 W) has only the mechanical power and the copper loss. `docs/hardware/power_calc.md` adds the drive standby power.

## Files

| File | What it is |
|---|---|
| `models.py` | Builds `models/bhl_biped.xml` and `models/bhl_F2*.xml`, then runs its self-check |
| `poses.py` | Leg poses and flat-foot placement on the floor |
| `analyze.py` | Static joint torques and centre of mass (stand, crouch, one leg) |
| `torso_tank.py` | Walking and push test helpers. `final_check.py` uses them (`archive/rl_f1/f1_eval.py` did too). |
| `report.py` | Plot palette and style of the simulation figures |
| `final_check.py` | The F2 check: statics, walking and push test (the drive cases moved to `sim/sim2sim/hw_risks.py`) |
| `measured.py`, `masses.csv` | The sheet of parts to weigh, and the script that puts the weighed masses into the model |
| `results/final_f2.md`, `results/final_f2.json` | Results of `final_check.py` |

## Open points

- **Open:** the model uses the 130 g DWP-385 pump of the CAD layout. The KLP02 weighs about 250 g, so the robot will be about 0.12 kg heavier, plus the DMC-101.
- **Not verified:** the part masses are CAD and datasheet values. Weigh the robot when it is built.
- **Not verified:** the model has no motor dynamics, no CAN jitter and a flat floor only.
- **Assumed:** the motor phase resistance is 0.1 Ω for the power numbers.

## Licence

Code: MIT (root `LICENSE`). This README and the result files: CC BY-SA 4.0.
The BHL meshes and URDF are CC BY-SA 4.0. `scripts/fetch_bhl_assets.sh` fetches them. They are not in git.
