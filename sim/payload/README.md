# Payload check (ASMR robot, F3 parts)

`final_check.py` checks that the ASMR robot can carry the F3 parts: statics, walking and pushes, with the policy chosen by name
(default `baseline`) and the payload options `none`, `box_1kg` and `f3` of `source/robot_models/robots/asmr2026_payload.py`.
It runs the ASMR runner `sim/sim2sim/run.py`.

**Status:** tested in simulation (MuJoCo 3.15.0, CPU only). Nothing here is measured on board. **The check has not been run on the ASMR
robot yet** (only its quick self-check): the old F2 results of the BHL model are in `archive/payload_f2/`.
The BHL model builder (`models.py`), the weighed-mass script (`measured.py`, `masses.csv`) and `bhl_legacy.py` are in `archive/payload_bhl/`.

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

No BHL model or asset is needed.

## Files

| File | What it is |
|---|---|
| `poses.py` | Leg poses (ASMR: the right pitch joints turn the other way) and flat-foot placement on the floor |
| `analyze.py` | Static joint torques and centre of mass of the ASMR robot (stand, crouch, one leg) |
| `stability.py` | Walking and push test helpers on the ASMR runner (`run.simulate` with a push). `final_check.py` uses them. |
| `report.py` | Plot palette and style of the simulation figures |
| `final_check.py` | The check of the ASMR robot with the F3 parts: statics, walking and push test (the drive cases are in `sim/sim2sim/hw_risks.py`) |
| `results/final_f3.md`, `results/final_f3.json` | Results of `final_check.py` (written by the run) |

## Open points

- **Open:** the model uses the 130 g DWP-385 pump of the CAD layout. The KLP02 weighs about 250 g, so the robot will be about 0.12 kg heavier, plus the DMC-101.
- **Not verified:** the part masses are CAD and datasheet values. Weigh the robot when it is built.
- **Not verified:** the model has no motor dynamics, no CAN jitter and a flat floor only.
- **Assumed:** the motor phase resistance is 0.1 Ω for the power numbers.

## Licence

Code: MIT (root `LICENSE`). This README and the result files: CC BY-SA 4.0.
