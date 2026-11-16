# Locomotion policy: training and evaluation

Everything that creates and checks the walking policy of the robot: Isaac Lab tasks (in `source/berkeley_humanoid_lite`), RSL-RL training, ONNX export, and the sim2sim and robustness evaluations. The robot model is `source/robot_models` (asmr2026-humanoid-v1); the trained networks are in [`policies/`](../../policies/README.md). The code comes from the SOBANG locomotion repository (see [`source/UPSTREAM.md`](../../source/UPSTREAM.md)).

This is its own uv project (Python 3.10, Isaac Sim 4.5, Isaac Lab 2.1), like `sim/rl` and `vision`. The root project (Python 3.12) does not install Isaac Sim. An NVIDIA GPU is needed for training. Run every command from the repository root; the first run downloads several GB of Isaac Sim packages.

```bash
uv sync --project scripts/isaaclab
```

## Layout

| Path | Purpose |
|---|---|
| `rsl_rl/` | `train.py`, `play.py` (also exports ONNX and writes `configs/policy_latest.yaml`), `play_onnx.py`, `export_onnx.py`, `expand_checkpoint.py` |
| `sim2sim/` | Isaac Lab and MuJoCo evaluations: command tracking, sustained push, timing gaps; `play_mujoco.py` |
| `push_test/` | 8-direction impulse and sustained push tests, plots |
| `s2r/` | Sim-to-real gaps: dynamics, actuator gain, timing, goal reaching, falls |
| `scenario/` | Scenario recordings (Isaac Lab and MuJoCo) |
| `sim2real/` | Visualisation of sim2real logs |
| `list_envs.py` | List the registered tasks |
| `../../configs/` | Deployment config templates (`policy_asmr_biped.yaml` is the course robot template) |

## Train and play

```bash
uv run --project scripts/isaaclab scripts/isaaclab/rsl_rl/train.py --task Velocity-ASMR-Biped-v0 --headless
uv run --project scripts/isaaclab scripts/isaaclab/rsl_rl/play_onnx.py --config policies/baseline/policy.yaml --task Velocity-ASMR-Biped-v0
uv run --project scripts/isaaclab scripts/isaaclab/sim2sim/play_mujoco.py --config policies/baseline/policy.yaml
uv run --project scripts/isaaclab scripts/isaaclab/sim2sim/eval_tracking_mujoco.py --config policies/baseline/policy.yaml --name baseline
```

Logs go to `logs/rsl_rl/<experiment>/<date>/` (not in git). S2R, gain, timing and goal-reaching evaluations must run one policy per process: from the second policy on, results in one process were wrong.

To use a newly trained network on the robot, copy `model_*.pt`, `policy.onnx`, `policy.yaml` and `params/` into `policies/<name>/` and point `policy` in `robot/config.toml` at the new `policy.onnx`. The observation size, action size, joint order and `POLICY_DT` must match `source/robot_models/robots/asmr2026_policy.py`; `legs/run_policy.py --check` verifies it.

## Not carried over

`scripts/teleop` and `scripts/hardware` of the SOBANG repository (Berkeley Humanoid Lite upper body, model comparison against the BHL MJCF) and the C++ and CAN parts of the BHL lowlevel repository. This robot's drivers are in `legs/` and `actuators/`.
