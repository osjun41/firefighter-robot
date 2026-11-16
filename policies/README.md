# Trained walking policies

Each folder holds `model_<iteration>.pt` (RSL-RL checkpoint), `policy.onnx` (deployment network), `policy.yaml` (deployment config, paths relative to the repository root) and `params/` (`env.yaml`, `agent.yaml` of the training run). Train and evaluate with [scripts/isaaclab](../scripts/isaaclab/README.md). `…` stands for `Velocity-ASMR-Biped`.

| Folder | Training task | Description | Obs |
|---|---|---|---|
| `baseline` | `…-v0` | Baseline walking. Upstream DR (mass, friction, joint offsets, gains x0.8-1.2). The robot runs this one. | 45 |
| `s2r_dr000` ... `s2r_dr200` | `…-S2R-DR{000,050,100,200}-v0` | Dynamics randomisation width alpha 0, 0.5, 1, 2 | 45 |
| `s2r_friction100` | `…-S2R-Friction100-v0` | alpha 1, friction only | 45 |
| `s2r_inertia100` | `…-S2R-Inertia100-v0` | alpha 1, mass and centre of mass only | 45 |
| `push_m2` | `…-Push-v0` | Push DR, no signal | 45 |
| `push_m3` | `…-PushSignal-v0` | Push signal added to the observation | 46 |
| `push_m3h` | `…-PushSignalHold-v0` | M3 + hold-position reward | 46 |
| `push_m3h2` | `…-PushSignalHold2-v0` | M3h improved (best single policy) | 46 |
| `brace_m4` | `…-Brace-v0` | Bracing-only policy | 46 |
| `brace_m4b` | `…-Brace2-v0` | M4 + strict hold reward, signal leads, actions limited to +-20 | 46 |
| `act_gain125` / `150` / `200` | `…-Actuator-Gain{125,150,200}-v0` | Per-joint Kp, Kd randomisation x0.8-1.25 / x0.67-1.5 / x0.5-2 | 45 |

alpha 1 means torso mass +-1.5 kg, leg mass +-20 %, centre of mass +-3 cm, foot friction 0.8 +- 0.4.

- Only 45-observation policies fit the robot software today. The push-signal policies (46) take the push flag (0/1) as the last observation, which `legs/run_policy.py`, `RlController` and `play_mujoco.py` do not provide. Test them in MuJoCo with `scripts/isaaclab/sim2sim/eval_sustained_push_mujoco.py`.
- M4 and M4b are meant to be switched in from `baseline` when a push signal arrives. That switch is not implemented in the deployment code.
