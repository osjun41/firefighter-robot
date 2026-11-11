# Simulations

This folder holds the simulations of the design F3 (the robot with its water trolley): the robot model, walking, the sim-to-real gap, missions, the water jet and synthetic fire images.
All results here are **tested in simulation**. Nothing in this folder is measured on board.

## Setup (once)

Run these commands from the repository root.

1. Install the Python environment:
   ```
   uv sync
   ```
2. Fetch the Berkeley Humanoid Lite (BHL) meshes and URDF. They are not in git:
   ```
   scripts/fetch_bhl_assets.sh
   ```
3. Build the MuJoCo models of the robot. The XML files are not in git, and most simulations need them:
   ```
   uv run python sim/payload/models.py
   ```

## Folders

| Folder | What it simulates | Main command (from the repository root) |
|---|---|---|
| [payload/](payload/README.md) | The robot models (F2, F3 and the F3 options), the model with the weighed masses, and the payload check | `uv run python sim/payload/final_check.py` |
| [sim2sim/](sim2sim/README.md) | The shipped BHL walking policy loop (shared by `legs/`, `robot/` and the other folders), the motor drive risks and the sim-to-real gap study | `uv run python sim/sim2sim/gap_study.py --quick` |
| [mission/](mission/README.md) | Complete missions: walking, robot camera, fire detector, aiming, water droplets and fire model | `MUJOCO_GL=egl uv run python sim/mission/mission.py --seed 1055` |
| [rl/](rl/README.md) | Training and evaluation of walking policies (f1_v1, and f3_v1 for F3). Its own uv project (JAX, GPU) | `uv run python sim/rl/f1_eval.py sim/rl/policies/f3_v1.onnx --model=F3 --quick` |
| [water/](water/README.md) | Pump, hose, nozzle and jet range of the water trolley | `uv run python sim/water/water.py` |
| [firescenes/](firescenes/README.md) | Synthetic fire images in Blender, for the detector training set | `blender -b --factory-startup -P sim/firescenes/firescenes.py -- samples 0 200` |

## Quick checks

| Check | Command | Time |
|---|---|---|
| Robot models | `uv run python sim/payload/models.py` | 5 s |
| Payload check | `uv run python sim/payload/final_check.py check` | 10 s |
| Policy loop | `uv run python sim/sim2sim/run.py --check` | 10 s |
| Drive model | `uv run python sim/sim2sim/hw_risks.py --check` | 20 s |
| Gap study | `uv run python sim/sim2sim/gap_study.py --quick` | 20 s |
| Weighed masses | `uv run python sim/payload/measured.py --check` | 5 s |
| Mission tests | `MUJOCO_GL=egl uv run pytest sim/mission/test_mission.py -q` | 10 s |
| Water model | `uv run python sim/water/water.py check` | 5 s |
| Policy evaluation | `uv run python sim/rl/f1_eval.py sim/rl/policies/f3_v1.onnx --model=F3 --quick` | 15 s |

## Key numbers

- F3 weighs 14.60 kg. Its centre of mass is 0.432 m high. F2 weighed 15.99 kg with full tanks (`payload/`, `cad/final`).
- On F3, the shipped policy walks at 0.25 m/s for a 0.3 m/s command at 6 Nm. On F2 it walked at 0.22 m/s (`sim2sim/`).
- The real torque per amp, the torso centre of mass, a constant hose pull, the IMU mounting tilt and the latency cause most falls in the gap study. The motor inertia and the joint friction matter little (`sim2sim/`).
- Our policy f3_v1 trained with these errors. With all of them at their middle value and one at its worst, it falls once in 240 runs. The shipped policy falls 123 times (`rl/`, `sim2sim/`).
- From a quiet stand, the policies start to walk only after a short forward command of 0.45 m/s for 0.5 s (`rl/`).
- The trolley gives 0.41 L/min and 9.6 m/s at 22.2 V with the 1.0 mm nozzle. One 2 L bag lasts 4.8 min (`water/`).
- The mission simulation still runs F2. In 50 missions with real-fire flames, falls and an empty tank end most missions (`mission/`).

## Licence

Code: MIT (root `LICENSE`). READMEs and result files: CC BY-SA 4.0.
The BHL policy file is MIT (`sim2sim/BHL_LICENCE`). The BHL meshes and URDF are CC BY-SA 4.0 and stay out of git.
