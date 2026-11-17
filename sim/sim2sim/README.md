# Sim-to-sim: the ASMR walking policy loop, the motor drive risks and the gap study

This folder holds `run.py`, the loop that runs a walking policy of `policies/` (default `baseline`) on the ASMR 2026 robot in MuJoCo,
and two studies built on it: `hw_risks.py` (the course motor drives) and `gap_study.py` (which differences between the simulation
and the real robot stop the walk). Both use the F3 parts on the torso (`--payload`).

**Status:** tested in simulation (MuJoCo 3.15.0, Python 3.12, CPU only). Nothing here is measured on board.

## Setup

Run these commands once, from the repository root:

1. Install the Python environment: `uv sync`
2. Fetch the BHL meshes and URDF (not in git; `mission/`, `payload/` and `bhl_legacy.py` still use them): `scripts/fetch_bhl_assets.sh`
3. Build the MuJoCo models `sim/payload/models/bhl_*.xml`: `uv run python sim/payload/models.py`

`run.py`, `hw_risks.py` and `gap_study.py` need neither the BHL meshes nor these models.

## Files

| File | What it is |
|---|---|
| `run.py` | The ASMR runner: policy by name, joint order, standing pose, gains and limits from the MJCF, observation, payload option, one simulation run, video, self-check |
| `hw_risks.py` | Sim-to-real risks of the course motor drives on the ASMR robot with the F3 parts |
| `gap_study.py` | Sim-to-real gap study: payloads, one change at a time, combined cases |
| `bhl_legacy.py` | The old BHL loop (BHL gains, pose and policy). Still used by `sim/mission` and `sim/payload/torso_tank.py`; FIXME(policy): goes in the next steps of the model unification |
| `baseline_detector.py` | Colour-threshold fire "detector" for simulated cameras. Not a real detector |
| `policy_biped_25hz_a.onnx` | The shipped BHL walking policy, for `bhl_legacy.py` only |
| `BHL_LICENCE` | The MIT licence of the BHL code, which the policy copy requires |
| `results/hw_risks.md`, `.json`, `.png` | All tables, findings and the recommendation of `hw_risks.py` | (written by the run)
| `results/gap_study.md`, `.json`, `.png` | Tables and figure of `gap_study.py` | (written by the run)
| `results/results*.md` | Tables of `run.py` (`results_f3.md`: baseline with the F3 parts) |

The old BHL versions of the two studies, with their reports, are in `archive/sim2sim_bhl/`.

## The runner (`run.py`)

| Part | Value in `run.py` | Source |
|---|---|---|
| Policy | `policies/<name>/policy.onnx` (`--policy NAME`, default `baseline`), input `obs` (1 × 45), output `actions` (1 × 12) | `policies/policy_select.py` |
| Observation | command (3), angular velocity (3), projected gravity (3), joint position − default (12), joint velocity (12), last action (12), in the IMU-corrected base frame | `source/robot_models/robots/asmr2026_policy.py` |
| Joint target | default + 0.25 × action, clipped to the joint limits. Default: the ASMR standing pose | `asmr2026_policy.py`, `asmr2026_model.py` |
| PD control | the position actuators of the MJCF: K 90 / 22 Nm/rad, D 5.7 / 0.49 Nm s/rad, 18.1 / 22.6 Nm (hip and knee / ankle) | `asmr2026_humanoid.xml` |
| Rates | policy 25 Hz, physics 2000 Hz | |
| Start state | the standing pose, base at its standing height | `STANDING_ROOT_HEIGHT` |
| Payload | `--payload none`, `box_1kg` (default) or `f3` (47 parts, 4.14 kg) | `source/robot_models/robots/asmr2026_payload.py` |
| Observation noise | uniform, as in the training (`NOISE`) | BHL `biped/env_cfg.py:55-72` |

- A fall is a torso tilt above 0.78 rad (the training limit) or an IMU height below 60 % of its start height.
- `simulate()` returns the fall time, the mean forward speed and yaw rate from 5 s to 20 s, and torque numbers.

```bash
uv run python sim/sim2sim/run.py --check                      # the robot stands 5 s with a zero command
uv run python sim/sim2sim/run.py --policy NAME --payload f3   # four commands -> results/results_f3.md
```

Results of a non-default payload go to `results/results_<payload>.md`. Baseline with the F3 parts (16.35 kg in all) does not fall in 20 s in
any of the four commands and walks at 0.27 m/s for a 0.3 m/s command (`results/results_f3.md`). This is MuJoCo with the ideal MJCF
actuators and no drive effects, so it says the baseline is not thrown off by the F3 mass, not that it walks on the robot.

## Drive risks (`hw_risks.py`)

The robot uses XDrive MINI drives (an ODrive v3.6 clone, firmware v0.5.1). The drive does the PD control.
`hw_risks.py` runs the `run.py` loop on the ASMR robot with the F3 parts and adds a drive model.
The model has per-joint gains and torque limits, joint zero errors, command and observation latency, and the TRAP_TRAJ input mode.
Each case has 4 commands × 10 seeds of 20 s, and a push test while the robot walks at 0.3 m/s.
It also computes the regen energy on the shared 24 V bus behind the battery diode.
"As trained" (case 1) means the gains and torque limits of the ASMR MJCF. The "course" cases use the gains and the 18 A current limit
of the XDrive course config, which differ from the MJCF.

```bash
uv run python sim/sim2sim/hw_risks.py                 # all cases, policy baseline, payload f3 -> results/hw_risks.md, .json, .png
uv run python sim/sim2sim/hw_risks.py --policy NAME --payload none
uv run python sim/sim2sim/hw_risks.py --report        # tables and figure again from results/hw_risks.json (1 s)
uv run python sim/sim2sim/hw_risks.py --check         # self-check
```

**Not run yet on the ASMR robot:** the results files `results/hw_risks.*` are written by the command above (about 1 h on 2 cores, less on more).
The old BHL results (K 20, D 2, 6 Nm; shipped BHL policy on F2) are in `archive/sim2sim_bhl/hw_risks.md`. They do not describe this robot:
its MJCF gains are K 90 / 22 and D 5.7 / 0.49, and its torque limit is the 18 A of the course config. After the run, read the tables and write the findings
and the drive settings into the "Findings and recommendation" section of the report, and here.

Case ids: 1 baseline (MJCF gains), 2 / 2b course gains with PASSTHROUGH / TRAP_TRAJ, 3 course gains at 67 % current limit, 4a-c zero error ±1° / ±2° / ±4°,
5a-e latency, 6a / 6b torque limit 67 % / 50 %, 7 / 7b / 7c / 7d combined realistic cases.

## Gap study (`gap_study.py`)

`gap_study.py` asks which differences between the simulation and the real robot most likely stop the real robot from walking.
It runs the `hw_risks.py` loop and drive model on three payloads (bare, 1 kg box, F3), then changes one thing at a time on F3.
A realistic case R, a pessimistic case P and 12 cases "R with one pessimistic value" test the errors together.
Each case runs the policy with 4 commands × 10 seeds of 20 s and a standing push test.

| What | Command |
|---|---|
| Self-check | `uv run python sim/sim2sim/gap_study.py --quick` |
| All cases, baseline | `uv run python sim/sim2sim/gap_study.py` |
| Add another policy to the results | `uv run python sim/sim2sim/gap_study.py --policy NAME` |
| Only some cases (added to the results) | `uv run python sim/sim2sim/gap_study.py --cases F3,R,P` |
| Tables and figure again | `uv run python sim/sim2sim/gap_study.py --report` |

**Not run yet on the ASMR robot:** `results/gap_study.*` are written by the first command (several hours on 2 cores; the BHL version took 56 min on 8 cores with two policies).
The old BHL study (ranking, measurement order, randomization ranges for the training) is in `archive/sim2sim_bhl/gap_study.md`.
The ASMR model already holds measured leg masses and fitted armature and friction, so the cases move those values up and down. Parts of the old ranking
(torque per amp, torso CoM, hose pull, IMU tilt, latency) are likely to carry over, but only this run says so.

`gap_study.py` uses optional settings of `hw_risks.simulate()`: another model and policy, an IMU tilt, gear play, a hose pull,
and the time at the torque limit per joint. The cases of `hw_risks.py` do not use them.

## Detector interface and the baseline detector

Every detector module defines `detect(rgb, t) -> list[FireDetection]` (`aim/detection.py`).

- `rgb` is the H × W × 3 uint8 RGB image of the robot camera. `t` is the image time in seconds.
- Give `u, v` at the point where the fire touches the floor, for example the bottom centre of the box. `aim/` puts this pixel ray on the floor.
- Return an empty list when there is no fire.

`baseline_detector.py` marks bright orange-to-yellow pixels and returns one detection per blob.
The simulated camera of `robot/` (`robot/config.toml`, section `[sim]`) and the baseline runs of `sim/mission` use it.
The real detector is `vision/detector.py`. To test a detector in full F2 missions, use `sim/mission/mission.py --detector FILE`.

## Open points

- **Not verified:** the actuator model has no motor dynamics and no CAN jitter. Gearbox backlash has only the simple model of `gap_study.py`.
- **Assumed:** the motor phase resistance is 0.1 Ω. The bus capacitance (5 × 100 µF per drive) comes from photos.
- **Open:** check the drive trip limits on an XDrive MINI, and log the loop latency on the board.
- **Open:** after the first walks, run `dump_errors(odrv0)` on each drive and look for overspeed and regen errors.

## Licence

Code: MIT (root `LICENSE`). This README and the result files: CC BY-SA 4.0.
`policy_biped_25hz_a.onnx` is a copy of BHL `checkpoints/policy_biped_25hz_a.onnx` (commit `984741a`, SHA-256 `65769d21…2fd94e`). It is under the MIT licence in [`BHL_LICENCE`](BHL_LICENCE).
The BHL meshes and URDF are CC BY-SA 4.0. `scripts/fetch_bhl_assets.sh` fetches them. They are not in git.
