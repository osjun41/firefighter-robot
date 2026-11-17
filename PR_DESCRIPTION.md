# Merge the SOBANG locomotion training code; gather it in `sim/locomotion`

## Summary

Adds the walking-policy training and evaluation code of [SOBANG_Locomotion](https://github.com/GeunYoung-Heo/SOBANG_Locomotion) and puts all of it in one folder, `sim/locomotion/`. Nothing there runs on the robot: the robot only reads trained networks from `policies/`. The robot software (`robot/`, `legs/`, `actuators/`, `aim/`) is not changed.

Compared with the SOBANG repository: every training and evaluation script is here (192 files checked one by one). 23 scripts differ from SOBANG by path strings only, with no logic change. The only files left out are listed under "Not included".

## What changed

**New: training and evaluation (from SOBANG)** — details in [`sim/locomotion/README.md`](sim/locomotion/README.md)

| Group | Content |
|---|---|
| `scripts/rsl_rl/` | `export_onnx.py`, `expand_checkpoint.py` (new); `train.py`, `play.py`, `play_onnx.py` (were already here) |
| `scripts/sim2sim/` | MuJoCo evaluations: tracking, sustained push, timing, `play_mujoco.py` (new); Isaac Lab tracking reference (was here) |
| `scripts/push_test/` | Standing push tests in 8 directions, sustained push, radar plot, video |
| `scripts/s2r/` | Sim-to-real gaps: dynamics, actuator gain, timing, goal reaching, fall rate, videos, figures |
| `scripts/scenario/`, `scripts/sim2real/`, `scripts/list_envs.py` | Scenario videos, UDP visualiser, task list |
| `packages/berkeley_humanoid_lite/`, `packages/berkeley_humanoid_lite_lowlevel/` | Isaac Lab tasks and MuJoCo environment; policy loop only (`RlController`, `Cfg`, `Se2Gamepad`) that the MuJoCo scripts import |
| `configs/` | Deployment config templates |

**Moved into `sim/locomotion/`** (same content, new place)

- `scripts/isaaclab/*` → `sim/locomotion/scripts/`
- `source/berkeley_humanoid_lite*` → `sim/locomotion/packages/`
- `source/UPSTREAM.md` → `sim/locomotion/UPSTREAM.md`
- The Isaac Lab uv project is `sim/locomotion/pyproject.toml` (Python 3.10, Isaac Sim 4.5). The root project (Python 3.12) is unchanged.
- `source/robot_models/` and `policies/` stay where they are: the robot software imports them.

**Fixes to paths that did not work**

- 16 `policies/*/policy.yaml`: `mjcf_path` pointed at the deleted `source/berkeley_humanoid_lite_assets/data/...`. Now `source/robot_models/data/...`.
- `rsl_rl/play.py` imported the robot from the assets package. Now `source.robot_models` (the one robot definition).
- Removed the duplicate `asmr2026_humanoid.py` from the assets package (differed only in the USD path).
- **Removed the original BHL robot.** Deleted `packages/berkeley_humanoid_lite_assets` (BHL robot config, no model files) and the BHL `humanoid` task (`Velocity-Berkeley-Humanoid-Lite-v0`); the task package imports every config folder, so the package could not go without the task. `MujocoEnv` now raises an error for a config without `mjcf_path` instead of falling back to a BHL scene whose files are not in the repository. 
- `UPSTREAM.md` rewritten: what came from which upstream commit, what is not here, and the commands to get the originals (`scripts/fetch_bhl_assets.sh`, `git clone` + `checkout` of the pinned commits).
- `berkeley_humanoid_lite/__init__.py` puts the repository root on `sys.path`, so `source.robot_models` resolves for scripts run from `sim/locomotion/scripts/`.
- Paths that depend on folder depth: `s2r/make_result_figures.py`, `s2r/make_gap_videos.sh`.
- Commands and defaults now use `sim/locomotion/...` (for example `play.py` writes `sim/locomotion/configs/policy_latest.yaml`, git-ignored).

**Policies are selected by name** (`policies/policy_select.py`, standard library only)

- One rule everywhere: a policy is a folder in `policies/`, chosen by `--policy NAME`; no name means `baseline`. An unknown name fails with the list of available names; paths are rejected.
- Robot: `robot/config.toml` `[legs] policy = "baseline"` is now a name (was a file path); `legs/run_policy.py --policy NAME` overrides it for one run.
- `sim/sim2sim/run.py`, and every `sim/locomotion` script that took `--config <policy.yaml>`, now take `--policy NAME`. The `s2r/` scripts read `policies/<name>/model_*.pt` instead of `NAME=CHECKPOINT`. `Cfg.from_arguments()` takes `--policy` too (its old default file did not exist).
- `export_onnx.py --checkpoint <model.pt> --name NAME` creates `policies/NAME/` (refuses to overwrite without `--force`).
- Stage 1 of the model unification: `sim/sim2sim/hw_risks.py` and `gap_study.py` now run the ASMR robot with the named policy (default `baseline`) and the selectable payload (default `f3`). Their results are **not regenerated yet** (long runs); the old BHL results are in `archive/sim2sim_bhl/`.
- Stage 2: `sim/payload/final_check.py` (statics, walking, push test) runs the ASMR robot with `--policy NAME` and the payload options; its F2 results are in `archive/payload_f2/` and the new results are not generated yet.
- Stage 3: `sim/mission` runs the ASMR robot with the F3 parts (pan-tilt, camera and nozzle from `aim/geometry.py` F3), the policy by name (`--policy NAME`, default `baseline`) and the jet of F3 (0.41 L/min, 9.6 m/s at 22.2 V, water on the trolley, not on the robot). Design `f2` is gone. No mission campaign has been run on the ASMR robot yet.
- Stage 4: no simulation uses the BHL robot any more. `bhl_legacy.py`, the BHL model builder `models.py`, `measured.py`/`masses.csv` and the BHL policy copy are in `archive/payload_bhl/` (or deleted); `torso_tank.py` is now `sim/payload/stability.py`; `docs/hardware/trolley.py` uses the ASMR+F3 robot mass. `third_party/` stays only for the CAD scripts (`cad/`), which draw on the BHL meshes.

**Tasks and archive**

- Stopped registering the 13 `Velocity-Berkeley-Humanoid-Lite-Biped-*` tasks. 16 `Velocity-ASMR-Biped-*` tasks remain (a task is an environment recipe: robot, rewards, randomisation, pushes; training runs write to `logs/rsl_rl/<experiment>/<time>/`). `config/biped*` stays as the base classes of the `config/asmr_biped*` tasks.
- Script default tasks are now `Velocity-ASMR-Biped-*`, which includes the 1 kg payload box.
- `list_envs.py` listed nothing (filter was `Template-`); it now lists `Velocity-*`.
- JAX/MJX training (`sim/rl`) and the mission campaign (`campaign.py`) moved to `archive/` (nothing imports it; see `archive/README.md`). Walking training is Isaac Lab only. No script now evaluates the whole mission.

**Payload choice and policies in `policies/`**

- `--payload NAME` (`none`, `box_1kg` = default, `f3`) for `train.py`, `play.py`, `play_onnx.py`, `export_onnx.py` and `sim/sim2sim/run.py`. `f3` is the 47 CAD parts of the F3 design (4.14 kg), listed in `source/robot_models/robots/asmr2026_payload.py` from `cad/final/out_f3/layout.json`. The parts are rigid and have no collision: MuJoCo welds each as a body, Isaac Lab merges them into the base link (`mdp.add_body_payload` takes an `inertia`). Nothing is removed from the course torso, so F3 is an upper bound.
- `f1_v1` and `f3_v1` (BHL model) moved from `archive/rl_f1/policies/` to `policies/`, with `info.json` (`trained_on`). The ASMR robot code warns about a policy trained on another model and still loads it. The robot keeps `baseline`.
- Exec bits restored for `scripts/fetch_bhl_assets.sh` and `robot/setup/install.sh`; stale `sim/rl` paths in the policy cards fixed.

**Docs**

- `README.md`: new "Laptop or robot?" table and "Put a new policy on the robot" (5 steps; the network is named only by `[legs] policy`, a folder name).
- `sim/locomotion/README.md`: layout, setup, every script with what it does and where it runs, local verification (levels A, B, C), known limits.
- `policies/README.md`: the 16 trained networks. `sim/README.md`: new row. `CONTRIBUTING.md`: `sim/locomotion/` added to the walking workstream, layout test added to the test list.

**Tests and CI**

- New `sim/locomotion/tests/test_layout.py` (16 tests, no GPU): scripts compile, our own imports resolve to files, `policy.yaml` files and their `mjcf_path` exist, no old path is used, one robot definition, policy selection by name (default, error text, every folder resolves, no `--config`), no old task registered. Checked by breaking an import on purpose: the test fails.
- New CI step "Locomotion layout".

## Not included

- From SOBANG: `scripts/teleop` (BHL upper body), `scripts/hardware` (compares with the BHL model), and the C++/CAN/IMU drivers of the BHL lowlevel repo. They do not create policies, and the robot's drives are in `legs/` and `actuators/`.
- Hardware test menu: `robot/test_menu.py` is unchanged. The tests of the `actuators-asmr-actuator-test` branch are not merged here (separate change).
- Isaac Lab tasks (except the removed BHL `humanoid` task), rewards and policies: no change.

## How it was checked

Fresh clone of this branch, mujoco 3.15.0 (the pinned version), models built first as in CONTRIBUTING:

| Check | Result |
|---|---|
| `pytest sim/locomotion/tests` | 16 passed (includes the F3 merge against MuJoCo's own fusion) |
| `pytest` (whole repository) | 69 passed, 1 failed (`$USER`, also on `main`) |
| `legs/run_policy.py --check [--policy NAME]`, `sim/sim2sim/run.py --check [--policy NAME]` | passed; unknown name lists the policies; 46-observation policy refused on the robot path |
| Windows, Isaac Sim 4.5 / Isaac Lab 2.1 headless | tasks register (before the task cleanup: 29, now 16); `eval_tracking_mujoco.py` on `baseline`: 0 falls, tracking close to the commands |
| All 16 `policy.yaml`: network and `mjcf_path` exist | 16 of 16 |

`sim/sim2sim/run.py --payload f3`: baseline does not fall in 20 s with the F3 parts (16.35 kg) in any of 4 commands (MuJoCo, ideal actuators).

**Not run (needs Isaac Lab and a GPU)**: the Isaac side of `--payload` (`select_payload`, `add_body_payload(inertia=...)`; compiled and the maths checked against MuJoCo). **Not run after the `--policy` change** (needs Isaac Lab and a GPU; the scripts were compiled and the resolver tested): training, `export_onnx.py`, and the Isaac evaluation scripts. Please run levels B and C of "Verify locally" in `sim/locomotion/README.md` once before relying on them.

## Next (not in this PR)

- Run a mission campaign on the ASMR robot and the named policy, rerun `hw_risks`, `gap_study`, `power_calc.py` and `final_check.py`, and write their findings (their result files still hold the old text until then).
- Payload: water tank removed, spray device kept (water by IV drip, so no water payload). Measured leg masses (+677 g) and torso/IMU masses are applied with F3.

## Notes for reviewers

- The robot test `test_install_renders_valid_files` fails when `$USER` is not set. This also happens on `main`.
- `legs/test_legs.py::test_fake_policy_run_stays_up` fails with mujoco 4.0.1 and passes with the pinned 3.15.0. Use `uv sync --locked`.
- `policies/` (28 MB) holds `.pt` checkpoints, which CONTRIBUTING says to keep out of git. They were already on `main` and are not changed here.
- `s2r/eval_timing_gap.py` has one scenario that copies the BHL deployment loop, not the loop of `legs/run_policy.py`.
- The robot software runs 45-observation policies only. The 46-observation push-signal policies are for simulation.

🤖 Generated with [Claude Code](https://claude.com/claude-code)

https://claude.ai/code/session_01JSaaHkyjgzws9CiksTpUTm
