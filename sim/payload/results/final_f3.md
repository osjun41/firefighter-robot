# ASMR robot with the F3 parts: payload check (tested in simulation)

Robot: `asmr2026_humanoid_v1` (MJCF of `source/robot_models`) with the payload options of `source/robot_models/robots/asmr2026_payload.py`. F3: the 47 parts of `source/robot_models/data/f3_layout.json` (4.14 kg). Policy: `baseline` (`policy.onnx`), the PD gains of the MJCF.

## Statics

Joint torques that hold the pose, feet flat. The single-leg pose sways the pelvis over the stance foot as far as the hip roll range allows (the CoM must stay on the sole: `ok`).

| Model | Mass (kg) | Added (kg) | CoM x / y (mm) | CoM height (m) | Peak torque, stand (Nm) | Peak torque, crouch (Nm) | Peak pitch torque, one leg (Nm) | Peak torque, one leg, any joint (Nm) | One leg ok |
|---|---|---|---|---|---|---|---|---|---|
| bare robot | 12.21 | +0.00 | -34 / -1 | 0.432 | 3.7 | 4.9 | 9.2 | 9.2 | yes |
| robot with a 1 kg box (training default) | 13.21 | +1.00 | -35 / -1 | 0.464 | 4.1 | 5.5 | 10.1 | 10.1 | yes |
| robot with the F3 parts | 16.35 | +4.14 | -43 / -0 | 0.492 | 5.8 | 7.7 | 13.9 | 13.9 | yes |

Effort limits of the MJCF: 18.1 Nm (hip, knee), 22.6 Nm (ankle).

## Walking, 20 s with observation noise

| Model | Torque cap | stand | vx 0.3 | vx 0.2 + yaw 0.5 |
|---|---|---|---|---|
| bare robot | MJCF | vx -0.00, wz +0.00 | vx +0.29, wz -0.03 | vx +0.19, wz +0.52 |
| bare robot | 12 Nm | vx -0.00, wz +0.00 | vx +0.29, wz -0.03 | vx +0.19, wz +0.52 |
| robot with a 1 kg box (training default) | MJCF | vx -0.00, wz +0.00 | vx +0.29, wz -0.04 | vx +0.20, wz +0.52 |
| robot with a 1 kg box (training default) | 12 Nm | vx -0.00, wz +0.00 | vx +0.29, wz -0.04 | vx +0.20, wz +0.52 |
| robot with the F3 parts | MJCF | vx -0.00, wz +0.00 | vx +0.27, wz -0.02 | vx +0.17, wz +0.54 |
| robot with the F3 parts | 12 Nm | vx -0.00, wz +0.00 | vx +0.27, wz -0.02 | vx +0.17, wz +0.53 |

## Push test: largest push with no fall (N s, worst of 3 push times)

| Model | standing, forward | standing, back | standing, left | standing, right | walking 0.3 m/s, forward | walking 0.3 m/s, back | walking 0.3 m/s, left | walking 0.3 m/s, right |
|---|---|---|---|---|---|---|---|---|
| bare robot | 11.6 | 7.5 | 6.0 | 5.2 | 9.0 | 12.4 | 4.9 | 4.9 |
| robot with a 1 kg box (training default) | 13.1 | 7.9 | 6.4 | 6.0 | 11.6 | 14.6 | 5.6 | 5.6 |
| robot with the F3 parts | 15.8 | 8.6 | 7.9 | 7.1 | 13.1 | 11.6 | 6.4 | 5.6 |

## Findings and recommendation

Not written yet: this check has not been run on the ASMR robot. The F2 check of the Berkeley model (4 models, shipped Berkeley policy) is in `archive/payload_f2/final_f2.md`.
