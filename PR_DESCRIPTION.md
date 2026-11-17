# Merge the SOBANG locomotion training code; one robot (ASMR) for all simulation

## Summary

This PR does two things.

1. **Training code.** It adds the walking-policy training and evaluation code of [SOBANG_Locomotion](https://github.com/GeunYoung-Heo/SOBANG_Locomotion) and puts all of it in one folder, `sim/locomotion/`. Nothing there runs on the robot: the robot only reads trained networks from `policies/`. The robot software (`robot/`, `legs/`, `actuators/`, `aim/`) is not changed.
2. **One robot in simulation.** Every simulation in the repository now runs the ASMR robot (`source/robot_models`) with a policy chosen by name (default `baseline`) and a payload chosen by name (`f3` = the 47 CAD parts of the F3 design). The Berkeley Humanoid Lite (BHL) robot is no longer used by any simulation.

Compared with the SOBANG repository: every training and evaluation script is here (192 files checked one by one). 23 scripts differ from SOBANG by path strings only, with no logic change. The only files left out are listed under "Not included".

## What changed

### Training and evaluation code (from SOBANG)

Details in [`sim/locomotion/README.md`](sim/locomotion/README.md).

| Group | Content |
|---|---|
| `scripts/rsl_rl/` | `export_onnx.py`, `expand_checkpoint.py` (new); `train.py`, `play.py`, `play_onnx.py` (were already here) |
| `scripts/sim2sim/` | MuJoCo evaluations: tracking, sustained push, timing, `play_mujoco.py` (new); Isaac Lab tracking reference (was here) |
| `scripts/push_test/` | Standing push tests in 8 directions, sustained push, radar plot, video |
| `scripts/s2r/` | Sim-to-real gaps: dynamics, actuator gain, timing, goal reaching, fall rate, videos, figures |
| `scripts/scenario/`, `scripts/sim2real/`, `scripts/list_envs.py` | Scenario videos, UDP visualiser, task list |
| `packages/berkeley_humanoid_lite/`, `packages/berkeley_humanoid_lite_lowlevel/` | Isaac Lab tasks and MuJoCo environment; policy loop only (`RlController`, `Cfg`, `Se2Gamepad`) that the MuJoCo scripts import |
| `configs/` | Deployment config templates |

Moved into `sim/locomotion/` (same content, new place): `scripts/isaaclab/*` → `sim/locomotion/scripts/`, `source/berkeley_humanoid_lite*` → `sim/locomotion/packages/`, `source/UPSTREAM.md` → `sim/locomotion/UPSTREAM.md`. The Isaac Lab uv project is `sim/locomotion/pyproject.toml` (Python 3.10, Isaac Sim 4.5); the root project (Python 3.12) is unchanged. `source/robot_models/` and `policies/` stay where they are because the robot software imports them.

### Policies and payloads are chosen by name

- A policy is a folder in `policies/`, chosen by `--policy NAME` everywhere; no name means `baseline`. An unknown name fails with the list of names; paths are rejected (`policies/policy_select.py`, standard library only).
- Robot: `robot/config.toml` `[legs] policy = "baseline"` is a name now (was a file path); `legs/run_policy.py --policy NAME` overrides it for one run.
- `--payload NAME` (`none`, `box_1kg` = default, `f3`) for `train.py`, `play.py`, `play_onnx.py`, `export_onnx.py` and `sim/sim2sim/run.py`. `f3` is the 47 parts of `cad/final/out_f3/layout.json` (4.14 kg), listed in `source/robot_models/robots/asmr2026_payload.py`. The parts are rigid and have no collision: MuJoCo welds each as a body, Isaac Lab merges them into the base link (`mdp.add_body_payload` takes an `inertia`). Nothing is removed from the course torso, so F3 is an upper bound.
- `export_onnx.py --name NAME [--payload f3]` creates `policies/NAME/` (refuses to overwrite without `--force`). `--checkpoint` takes a path, a file name in the run folder (with `--load_run`), a run folder, or nothing (the last model of the newest run), like `play.py`.
- `f1_v1` and `f3_v1` (trained on the BHL model) moved from `archive/rl_f1/policies/` to `policies/`, with `info.json` (`trained_on`). The ASMR robot code warns about a policy trained on another model and still loads it. The robot keeps `baseline`.

### Simulation moved to the ASMR robot (4 stages)

| Stage | Code | State |
|---|---|---|
| 1 | `sim/sim2sim/hw_risks.py`, `gap_study.py` | ASMR robot, named policy, selectable payload (default `f3`). `hw_risks` results regenerated with `baseline`. `gap_study` results **not regenerated yet** (long run). |
| 2 | `sim/payload/final_check.py` (statics, walking, push test), `stability.py` (was `torso_tank.py`) | ASMR robot, `--policy NAME`, payload options. `results/final_f3.*` hold a run with `baseline`; the Findings text is **not written**. |
| 3 | `sim/mission` | ASMR robot with the F3 parts (pan-tilt, camera, nozzle from `aim/geometry.py` F3), policy by name, jet of F3 (0.41 L/min, 9.6 m/s at 22.2 V, water on the trolley, not on the robot). Design `f2` is gone. **No mission campaign has been run** on the ASMR robot. |
| 4 | clean-up | `bhl_legacy.py`, the BHL model builder `models.py`, `measured.py`, `masses.csv` moved to `archive/payload_bhl/`; the shipped BHL onnx policy and the generated BHL models deleted; `docs/hardware/trolley.py` uses the ASMR + F3 robot mass (16.35 kg, CoM 0.49 m); `cad/final/README.md` gives the ASMR numbers. |

`third_party/` and `scripts/fetch_bhl_assets.sh` stay for the CAD scripts in `cad/`, which draw on the BHL meshes. No simulation and no test needs them. `sim/sim2sim/BHL_LICENCE` stays because the MIT licence of the BHL code and policy requires its notice. The old BHL results are in `archive/sim2sim_bhl/`, `archive/payload_f2/` and `archive/payload_bhl/`.

### Removal of the original BHL robot from the training code

- Deleted `packages/berkeley_humanoid_lite_assets` (BHL robot config, no model files) and the BHL `humanoid` task (`Velocity-Berkeley-Humanoid-Lite-v0`); the task package imports every config folder, so the package could not go without the task. `MujocoEnv` raises an error for a config without `mjcf_path` instead of falling back to a BHL scene whose files are not in the repository.
- Stopped registering the 13 `Velocity-Berkeley-Humanoid-Lite-Biped-*` tasks. 16 `Velocity-ASMR-Biped-*` tasks remain (`config/biped*` stays as the base classes). Script default tasks are `Velocity-ASMR-Biped-*`, which includes the 1 kg payload box.
- Removed the duplicate `asmr2026_humanoid.py` from the assets package.
- `list_envs.py` listed nothing (filter was `Template-`); it now lists `Velocity-*`.
- JAX/MJX training (`sim/rl`) and the mission campaign (`campaign.py`) moved to `archive/` (nothing imports them). Walking training is Isaac Lab only.

### Fixes

- 16 `policies/*/policy.yaml`: `mjcf_path` pointed at the deleted `source/berkeley_humanoid_lite_assets/data/...`. Now `source/robot_models/data/...`.
- `rsl_rl/play.py` imported the robot from the assets package. Now `source.robot_models`.
- Paths that depend on folder depth (`s2r/make_result_figures.py`, `s2r/make_gap_videos.sh`) and defaults (`play.py` writes `sim/locomotion/configs/policy_latest.yaml`, git-ignored).
- `legs/sysid.py`: CSV times are rounded to 5 decimals but the release time was not (a test failed on Windows).
- Text files are read and written as UTF-8 (the Windows default is cp949); `final_check.py` and `stability.py` create `results/` when it is missing; `gap_study.py` writes its JSON when a case holds numpy arrays.
- Exec bits restored for `scripts/fetch_bhl_assets.sh` and `robot/setup/install.sh`; stale `sim/rl` paths in the policy cards fixed.

### Docs, tests and CI

- `README.md`: "Laptop or robot?" table and "Put a new policy on the robot" (5 steps). `sim/locomotion/README.md`: layout, setup, every script, which `model_*.pt` each script takes, local verification (levels A, B, C), known limits. `policies/README.md`, `sim/README.md`, `sim/payload/README.md`, `sim/sim2sim/README.md`, `archive/README.md`, `CONTRIBUTING.md` updated.
- `sim/locomotion/tests/test_layout.py` (19 tests, no GPU): scripts compile, our own imports resolve, `policy.yaml` files and their `mjcf_path` exist, no old path is used, one robot definition, policy selection by name, no old task registered, no script uses the BHL model. `sim/mission/test_mission.py` (12 tests).
- CI no longer fetches the BHL assets or builds BHL models; a "Locomotion layout" step runs the layout test.

## Not included

- From SOBANG: `scripts/teleop` (BHL upper body), `scripts/hardware` (compares with the BHL model), and the C++/CAN/IMU drivers of the BHL lowlevel repo. They do not create policies, and the robot's drives are in `legs/` and `actuators/`.
- Hardware test menu: `robot/test_menu.py` is unchanged. The tests of the `actuators-asmr-actuator-test` branch are not merged here (separate change).
- Isaac Lab tasks (except the removed BHL `humanoid` task), rewards and policies: no change.

## How it was checked

| Check | Where | Result |
|---|---|---|
| `sim/locomotion/tests/test_layout.py`, `sim/mission` tests, `final_check.py check`, `trolley.py`, with `third_party/` moved away | Linux, mujoco 3.15.0 | 19 passed, 12 passed, OK, OK |
| Repository tests, `hw_risks.py`, `final_check.py`, missions (`baseline` and oracle detector, with `--video`) | Windows 11 (author) | passed, except `robot/test_robot.py::test_launch_bench_sim...`, which needs Linux (`launch.py` uses `prctl`) |
| `train.py --payload f3 --headless` (short run), `export_onnx.py --payload f3`, `sim/sim2sim/run.py` with the exported policy | Windows 11, RTX 4070 Ti SUPER, Isaac Sim 4.5 (author) | ran |
| `legs/run_policy.py --check`, `sim/sim2sim/run.py --check` (`--policy NAME`) | Linux | passed; unknown name lists the policies; a 46-observation policy is refused on the robot path |
| All 16 `policy.yaml`: network and `mjcf_path` exist | Linux | 16 of 16 |

`hw_risks.py` with `baseline` on the ASMR robot with the F3 parts (16.35 kg): 0 falls in 40 runs with the MJCF gains; with the course gains and the course TRAP_TRAJ file all 40 fall (time to fall 1.5 s); a 67 % current limit does not cause falls. `sim/sim2sim/run.py --payload f3`: `baseline` does not fall in 20 s in any of 4 commands (MuJoCo).

**Not verified**

- `play.py` and `play_onnx.py` (also `--video`): on the author's Windows PC Isaac Sim crashed when it started the RTX renderer (access violation in `omni.kit.widget.viewport`, before any of our code runs), with and without `--headless`. They are not tested anywhere.
- The Isaac evaluation scripts, and a training of full length.
- A policy trained on the ASMR robot with the F3 parts has not been checked with `final_check.py`, `gap_study.py` or the missions.
- `mission.py --sil --detector oracle` (the default detector failed on the author's PC because the real-time simulation lagged).

Please run levels B and C of "Verify locally" in `sim/locomotion/README.md` once on a machine where Isaac Sim opens its window before relying on the Isaac scripts.

## Next (not in this PR)

- Train a policy on the ASMR robot with the F3 parts, then rerun `final_check.py`, `gap_study.py` and `power_calc.py` with it, run the mission campaign, and write the Findings (the result files hold no Findings until then).
- Move `f1_v1` and `f3_v1` to `archive/` when a policy trained on the ASMR robot replaces them (the layout test, `legs/README.md`, `policies/README.md` and `docs/plan/walking.md` mention them).

## Notes for reviewers

- The robot test `test_install_renders_valid_files` fails when `$USER` is not set. This also happens on `main`.
- `legs/test_legs.py::test_fake_policy_run_stays_up` fails with mujoco 4.0.1 and passes with the pinned 3.15.0. Use `uv sync --locked`.
- `policies/` (28 MB) holds `.pt` checkpoints, which CONTRIBUTING says to keep out of git. They were already on `main` and are not changed here.
- `s2r/eval_timing_gap.py` has one scenario that copies the BHL deployment loop, not the loop of `legs/run_policy.py`.
- The robot software runs 45-observation policies only. The 46-observation push-signal policies are for simulation.
- Isaac Sim needs a working RTX renderer for `play.py`, `play_onnx.py` and videos. Training works with `--headless`.

🤖 Generated with [Claude Code](https://claude.com/claude-code)

https://claude.ai/code/session_01JSaaHkyjgzws9CiksTpUTm
