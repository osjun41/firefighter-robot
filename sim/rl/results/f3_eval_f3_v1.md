Model `sim/payload/models/bhl_F3.xml`: 14.58 kg, file time 2026-10-29 23:58.

### Commands on bhl_F3 (6 Nm, training noise, BHL start drop, 20 s)

5 seeds per test. Speeds and stepping metrics are means from 5 s to 20 s of the runs without a fall.
Air: share of time with a foot off the ground. Steps: touchdowns per second. Slip: mean speed of a foot on the ground. Drift: distance moved from 5 s to 20 s.

| Test | Policy | Falls | vx (m/s) | vy (m/s) | yaw rate (rad/s) | Drift (m) | Yaw drift (rad) | Air (%) | Steps (1/s) | Slip (m/s) | Max knee (rad) | Peak torque (Nm) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| stand | shipped | 0 of 5 | -0.00 | +0.00 | -0.00 | 0.01 | -0.01 | 0 | 0.0 | 0.002 | 0.21 | 6.0 |
| stand | f1_v1 | 0 of 5 | -0.00 | +0.00 | +0.00 | 0.01 | +0.05 | 0 | 0.0 | 0.001 | 0.28 | 6.0 |
| stand | f3_v1 | 0 of 5 | +0.00 | +0.00 | -0.00 | 0.01 | -0.01 | 0 | 0.0 | 0.001 | 0.21 | 6.0 |
| vx 0.1 | shipped | 0 of 5 | +0.00 | -0.00 | -0.00 | 0.01 | -0.02 | 0 | 0.0 | 0.003 | 0.25 | 6.0 |
| vx 0.1 | f1_v1 | 0 of 5 | +0.01 | +0.00 | +0.00 | 0.13 | +0.01 | 3 | 0.3 | 0.005 | 0.83 | 6.0 |
| vx 0.1 | f3_v1 | 0 of 5 | +0.02 | -0.00 | -0.03 | 0.25 | -0.47 | 6 | 1.1 | 0.011 | 0.55 | 6.0 |
| vx 0.2 | shipped | 0 of 5 | +0.09 | -0.01 | -0.02 | 1.29 | -0.32 | 26 | 1.9 | 0.026 | 1.33 | 6.0 |
| vx 0.2 | f1_v1 | 0 of 5 | +0.11 | -0.00 | +0.02 | 1.68 | +0.31 | 46 | 3.7 | 0.029 | 1.00 | 6.0 |
| vx 0.2 | f3_v1 | 0 of 5 | +0.13 | -0.01 | -0.01 | 2.01 | -0.25 | 39 | 4.6 | 0.046 | 0.69 | 6.0 |
| vx 0.3 | shipped | 0 of 5 | +0.25 | -0.01 | -0.05 | 3.62 | -0.66 | 58 | 3.1 | 0.057 | 1.36 | 6.0 |
| vx 0.3 | f1_v1 | 0 of 5 | +0.17 | -0.00 | +0.03 | 2.50 | +0.42 | 58 | 3.6 | 0.039 | 1.16 | 6.0 |
| vx 0.3 | f3_v1 | 0 of 5 | +0.20 | -0.01 | +0.03 | 3.02 | +0.48 | 51 | 4.4 | 0.056 | 0.84 | 6.0 |
| turn on the spot 0.3 | shipped | 0 of 5 | +0.00 | +0.00 | -0.00 | 0.01 | -0.00 | 0 | 0.0 | 0.003 | 0.31 | 6.0 |
| turn on the spot 0.3 | f1_v1 | 0 of 5 | +0.00 | +0.00 | +0.00 | 0.03 | +0.07 | 1 | 0.1 | 0.003 | 0.63 | 6.0 |
| turn on the spot 0.3 | f3_v1 | 0 of 5 | +0.00 | +0.00 | +0.03 | 0.06 | +0.39 | 3 | 0.4 | 0.005 | 0.49 | 6.0 |
| turn on the spot 0.5 | shipped | 0 of 5 | +0.00 | +0.00 | +0.02 | 0.01 | +0.30 | 0 | 0.0 | 0.009 | 0.77 | 6.0 |
| turn on the spot 0.5 | f1_v1 | 0 of 5 | +0.03 | +0.04 | +0.44 | 0.05 | +6.49 | 40 | 4.1 | 0.032 | 0.81 | 6.0 |
| turn on the spot 0.5 | f3_v1 | 0 of 5 | +0.06 | +0.05 | +0.45 | 0.07 | +6.58 | 38 | 5.2 | 0.048 | 0.62 | 6.0 |
| turn on the spot 0.8 | shipped | 0 of 5 | -0.02 | +0.04 | +0.59 | 0.06 | +8.73 | 35 | 3.1 | 0.038 | 1.10 | 6.0 |
| turn on the spot 0.8 | f1_v1 | 0 of 5 | +0.04 | +0.07 | +0.74 | 0.09 | +10.96 | 60 | 4.6 | 0.036 | 0.89 | 6.0 |
| turn on the spot 0.8 | f3_v1 | 0 of 5 | +0.08 | +0.06 | +0.65 | 0.30 | +9.53 | 43 | 5.1 | 0.068 | 1.23 | 6.0 |
| vx 0.2 + yaw 0.5 | shipped | 0 of 5 | +0.12 | +0.01 | +0.46 | 0.18 | +6.82 | 47 | 3.3 | 0.046 | 1.35 | 6.0 |
| vx 0.2 + yaw 0.5 | f1_v1 | 0 of 5 | +0.12 | +0.01 | +0.43 | 0.09 | +6.47 | 54 | 4.3 | 0.036 | 1.05 | 6.0 |
| vx 0.2 + yaw 0.5 | f3_v1 | 0 of 5 | +0.14 | +0.01 | +0.47 | 0.21 | +6.93 | 48 | 5.3 | 0.050 | 0.74 | 6.0 |
| stand, pan-tilt moving | shipped | 0 of 5 | -0.00 | +0.00 | -0.00 | 0.01 | -0.01 | 0 | 0.0 | 0.002 | 0.21 | 6.0 |
| stand, pan-tilt moving | f1_v1 | 0 of 5 | -0.00 | +0.00 | +0.00 | 0.01 | +0.05 | 0 | 0.0 | 0.001 | 0.28 | 6.0 |
| stand, pan-tilt moving | f3_v1 | 0 of 5 | +0.00 | +0.00 | -0.00 | 0.01 | -0.01 | 0 | 0.0 | 0.001 | 0.21 | 6.0 |
| stand, pan-tilt + recoil 0.3 N | shipped | 0 of 5 | -0.00 | -0.00 | -0.00 | 0.01 | -0.01 | 0 | 0.0 | 0.002 | 0.21 | 6.0 |
| stand, pan-tilt + recoil 0.3 N | f1_v1 | 0 of 5 | -0.00 | +0.00 | +0.00 | 0.01 | +0.05 | 0 | 0.0 | 0.001 | 0.29 | 6.0 |
| stand, pan-tilt + recoil 0.3 N | f3_v1 | 0 of 5 | +0.00 | +0.00 | -0.00 | 0.01 | -0.01 | 0 | 0.0 | 0.001 | 0.21 | 6.0 |
| stand, pan-tilt + recoil 1.3 N | shipped | 0 of 5 | -0.00 | -0.00 | -0.00 | 0.01 | -0.01 | 0 | 0.0 | 0.002 | 0.22 | 6.0 |
| stand, pan-tilt + recoil 1.3 N | f1_v1 | 0 of 5 | -0.00 | +0.00 | +0.00 | 0.01 | +0.05 | 0 | 0.0 | 0.002 | 0.33 | 6.0 |
| stand, pan-tilt + recoil 1.3 N | f3_v1 | 0 of 5 | +0.00 | +0.00 | -0.00 | 0.01 | -0.01 | 0 | 0.0 | 0.001 | 0.23 | 5.7 |
| one 6S pack: vx 0.3 | shipped | 0 of 5 | +0.26 | -0.01 | -0.05 | 3.86 | -0.69 | 62 | 2.9 | 0.058 | 1.36 | 6.0 |
| one 6S pack: vx 0.3 | f1_v1 | 0 of 5 | +0.18 | -0.00 | +0.03 | 2.73 | +0.39 | 64 | 3.6 | 0.039 | 1.12 | 6.0 |
| one 6S pack: vx 0.3 | f3_v1 | 0 of 5 | +0.20 | -0.02 | +0.04 | 3.03 | +0.55 | 50 | 4.3 | 0.062 | 0.83 | 6.0 |
| one 6S pack: turn on the spot 0.5 | shipped | 0 of 5 | +0.00 | +0.00 | +0.02 | 0.01 | +0.35 | 0 | 0.1 | 0.009 | 0.86 | 6.0 |
| one 6S pack: turn on the spot 0.5 | f1_v1 | 0 of 5 | +0.03 | +0.05 | +0.47 | 0.06 | +6.94 | 49 | 4.4 | 0.031 | 0.80 | 6.0 |
| one 6S pack: turn on the spot 0.5 | f3_v1 | 0 of 5 | +0.07 | +0.05 | +0.45 | 0.07 | +6.63 | 40 | 5.3 | 0.047 | 0.59 | 6.0 |
| power by cable: vx 0.3 | shipped | 0 of 5 | +0.27 | -0.01 | -0.05 | 3.86 | -0.78 | 63 | 2.8 | 0.057 | 1.36 | 6.0 |
| power by cable: vx 0.3 | f1_v1 | 0 of 5 | +0.19 | -0.01 | +0.03 | 2.81 | +0.33 | 67 | 3.6 | 0.040 | 1.10 | 6.0 |
| power by cable: vx 0.3 | f3_v1 | 0 of 5 | +0.21 | -0.02 | +0.02 | 3.12 | +0.35 | 52 | 4.2 | 0.059 | 0.87 | 6.0 |
| power by cable: turn on the spot 0.5 | shipped | 0 of 5 | -0.00 | +0.00 | +0.04 | 0.02 | +0.57 | 1 | 0.2 | 0.010 | 0.84 | 6.0 |
| power by cable: turn on the spot 0.5 | f1_v1 | 0 of 5 | +0.03 | +0.05 | +0.49 | 0.08 | +7.19 | 55 | 4.6 | 0.031 | 0.79 | 6.0 |
| power by cable: turn on the spot 0.5 | f3_v1 | 0 of 5 | +0.07 | +0.05 | +0.45 | 0.07 | +6.61 | 43 | 5.3 | 0.048 | 0.57 | 6.0 |

### Start walking from a stand (bhl_F3, 6 Nm)

Zero command for 2.0 to 3.75 s (by seed), then the walk command for 9.5 s. Distance: forward distance in these 9.5 s. Started: the distance is more than half of command x 9.5 s.

| Command (m/s) | Observation noise | Policy | Falls | Started | Distance, mean (min to max) (m) | Distance at the command (m) |
|---|---|---|---|---|---|---|
| 0.10 | training | shipped | 0 of 8 | 0 of 8 | 0.00 (-0.01 to 0.01) | 0.95 |
| 0.10 | training | f1_v1 | 0 of 8 | 0 of 8 | -0.00 (-0.02 to 0.02) | 0.95 |
| 0.10 | training | f3_v1 | 0 of 8 | 0 of 8 | 0.05 (-0.06 to 0.15) | 0.95 |
| 0.15 | training | shipped | 0 of 8 | 0 of 8 | 0.00 (-0.02 to 0.04) | 1.43 |
| 0.15 | training | f1_v1 | 0 of 8 | 0 of 8 | 0.48 (0.37 to 0.65) | 1.43 |
| 0.15 | training | f3_v1 | 0 of 8 | 0 of 8 | 0.61 (0.47 to 0.69) | 1.43 |
| 0.20 | training | shipped | 0 of 8 | 1 of 8 | 0.75 (0.26 to 1.24) | 1.90 |
| 0.20 | training | f1_v1 | 0 of 8 | 3 of 8 | 0.93 (0.80 to 1.05) | 1.90 |
| 0.20 | training | f3_v1 | 0 of 8 | 6 of 8 | 1.01 (0.91 to 1.08) | 1.90 |
| 0.25 | training | shipped | 0 of 8 | 8 of 8 | 1.56 (1.26 to 1.72) | 2.38 |
| 0.25 | training | f1_v1 | 0 of 8 | 6 of 8 | 1.25 (1.17 to 1.32) | 2.38 |
| 0.25 | training | f3_v1 | 0 of 8 | 8 of 8 | 1.39 (1.32 to 1.47) | 2.38 |
| 0.10 | no | shipped | 0 of 8 | 0 of 8 | 0.01 (0.01 to 0.01) | 0.95 |
| 0.10 | no | f1_v1 | 0 of 8 | 0 of 8 | 0.00 (0.00 to 0.00) | 0.95 |
| 0.10 | no | f3_v1 | 0 of 8 | 0 of 8 | 0.01 (0.00 to 0.01) | 0.95 |
| 0.15 | no | shipped | 0 of 8 | 0 of 8 | 0.00 (-0.00 to 0.00) | 1.43 |
| 0.15 | no | f1_v1 | 0 of 8 | 0 of 8 | -0.10 (-0.10 to -0.09) | 1.43 |
| 0.15 | no | f3_v1 | 0 of 8 | 8 of 8 | 0.85 (0.84 to 0.86) | 1.43 |
| 0.20 | no | shipped | 0 of 8 | 0 of 8 | -0.01 (-0.02 to -0.01) | 1.90 |
| 0.20 | no | f1_v1 | 0 of 8 | 8 of 8 | 1.16 (1.15 to 1.17) | 1.90 |
| 0.20 | no | f3_v1 | 0 of 8 | 8 of 8 | 1.25 (1.24 to 1.25) | 1.90 |
| 0.25 | no | shipped | 0 of 8 | 8 of 8 | 1.54 (1.43 to 1.62) | 2.38 |
| 0.25 | no | f1_v1 | 0 of 8 | 8 of 8 | 1.36 (1.34 to 1.37) | 2.38 |
| 0.25 | no | f3_v1 | 0 of 8 | 8 of 8 | 1.52 (1.44 to 1.54) | 2.38 |

### Push test (`sim/payload/torso_tank.py`, bhl_F3, 6 Nm)

Largest push with no fall in the next 3 s (N s, 0.1 s on the torso, worst of 3 gait phases).

| Policy | standing, forward | standing, back | standing, left | standing, right | walking 0.3 m/s, forward | walking 0.3 m/s, back | walking 0.3 m/s, left | walking 0.3 m/s, right |
|---|---|---|---|---|---|---|---|---|
| shipped | 9.8 | 5.2 | 4.5 | 4.5 | 7.9 | 6.8 | 3.8 | 2.6 |
| f1_v1 | 12.0 | 3.0 | 5.2 | 6.8 | 8.2 | 6.4 | 5.2 | 5.6 |
| f3_v1 | 10.1 | 4.1 | 5.6 | 7.5 | 7.5 | 7.1 | 6.4 | 7.5 |

### Drive cases of `sim/sim2sim/hw_risks.py` on bhl_F3

Each case: 4 commands (stand, vx 0.3, turn on the spot 0.5, vx 0.2 + yaw 0.5) x 10 seeds of 20 s. Push: walking at 0.3 m/s, 5 push times, worst / mean (N s).

| Case | Policy | Falls | vx at 0.3 | yaw rate at turn on the spot 0.5 | vx, yaw at 0.2 + 0.5 | Peak torque (Nm) | Action rate (rad) | Push back (N s) | Push left (N s) |
|---|---|---|---|---|---|---|---|---|---|
| 1: baseline: K 20, D 2, 6 Nm (as trained) | shipped | 0 of 40 | 0.25 | 0.03 | 0.13, 0.45 | 6.0 | 0.122 | 6.8 / 7.7 | 4.1 / 5.2 |
| 1: baseline: K 20, D 2, 6 Nm (as trained) | f1_v1 | 0 of 40 | 0.17 | 0.44 | 0.12, 0.43 | 6.0 | 0.114 | 5.2 / 6.1 | 4.9 / 7.0 |
| 1: baseline: K 20, D 2, 6 Nm (as trained) | f3_v1 | 0 of 40 | 0.21 | 0.45 | 0.14, 0.48 | 6.0 | 0.128 | 6.8 / 7.5 | 7.5 / 8.7 |
| 7: realistic: K 20, D 2, 6 Nm, ±2°, 10 ms command latency | shipped | 2 of 40 | 0.25 | 0.05 | 0.14, 0.48 | 6.0 | 0.125 | 5.6 / 8.1 | 1.9 / 2.9 |
| 7: realistic: K 20, D 2, 6 Nm, ±2°, 10 ms command latency | f1_v1 | 0 of 40 | 0.18 | 0.46 | 0.13, 0.45 | 6.0 | 0.113 | 4.1 / 5.3 | 4.1 / 4.9 |
| 7: realistic: K 20, D 2, 6 Nm, ±2°, 10 ms command latency | f3_v1 | 0 of 40 | 0.21 | 0.47 | 0.14, 0.48 | 6.0 | 0.123 | 6.4 / 7.1 | 6.4 / 7.7 |
| 7b: realistic + 10 ms observation latency | shipped | 4 of 40 | 0.26 | 0.14 | 0.15, 0.46 | 6.0 | 0.132 | 4.9 / 7.4 | 1.5 / 2.2 |
| 7b: realistic + 10 ms observation latency | f1_v1 | 0 of 40 | 0.18 | 0.48 | 0.14, 0.45 | 6.0 | 0.113 | 3.4 / 6.2 | 3.4 / 3.9 |
| 7b: realistic + 10 ms observation latency | f3_v1 | 0 of 40 | 0.22 | 0.49 | 0.15, 0.49 | 6.0 | 0.120 | 5.6 / 6.2 | 5.2 / 6.8 |

### Gap cases on bhl_F3 (`hw_risks.py` loop with a changed plant and drive)

Each case: 4 commands x 10 seeds of 20 s, training noise, BHL start drop. The model has armature 0.005 kg m² and friction loss 0.1 Nm on each leg joint. Friction loss 0.3-0.5 Nm: a random value per joint and seed. Torque scale: K, D and the 6 Nm limit together. This loop has no knee target clip.

| Case | Policy | Falls | Falls: stand, vx 0.3, turn 0.5, vx 0.2 + yaw 0.5 | vx at 0.3 | yaw rate at turn on the spot 0.5 | vx, yaw at 0.2 + 0.5 | Peak torque (Nm) | Action rate (rad) |
|---|---|---|---|---|---|---|---|---|
| armature x3 | shipped | 0 of 40 | 0, 0, 0, 0 | 0.25 | 0.02 | 0.13, 0.47 | 6.0 | 0.123 |
| armature x3 | f1_v1 | 0 of 40 | 0, 0, 0, 0 | 0.18 | 0.45 | 0.13, 0.45 | 6.0 | 0.115 |
| armature x3 | f3_v1 | 0 of 40 | 0, 0, 0, 0 | 0.21 | 0.47 | 0.14, 0.49 | 6.0 | 0.128 |
| friction loss 0.3-0.5 Nm | shipped | 0 of 40 | 0, 0, 0, 0 | 0.25 | 0.02 | 0.13, 0.43 | 6.0 | 0.120 |
| friction loss 0.3-0.5 Nm | f1_v1 | 0 of 40 | 0, 0, 0, 0 | 0.17 | 0.36 | 0.12, 0.37 | 6.0 | 0.113 |
| friction loss 0.3-0.5 Nm | f3_v1 | 0 of 40 | 0, 0, 0, 0 | 0.19 | 0.40 | 0.13, 0.43 | 6.0 | 0.127 |
| torque scale 0.85 | shipped | 7 of 40 | 0, 2, 0, 5 | 0.17 | 0.01 | 0.10, 0.39 | 5.1 | 0.132 |
| torque scale 0.85 | f1_v1 | 1 of 40 | 0, 1, 0, 0 | 0.10 | 0.32 | 0.08, 0.34 | 5.1 | 0.119 |
| torque scale 0.85 | f3_v1 | 0 of 40 | 0, 0, 0, 0 | 0.17 | 0.39 | 0.09, 0.41 | 5.1 | 0.131 |
| latency 20 ms obs, 40 ms cmd | shipped | 30 of 40 | 0, 10, 10, 10 | - | - | -, - | 6.0 | 0.223 |
| latency 20 ms obs, 40 ms cmd | f1_v1 | 2 of 40 | 0, 1, 0, 1 | 0.18 | 0.45 | 0.14, 0.40 | 6.0 | 0.114 |
| latency 20 ms obs, 40 ms cmd | f3_v1 | 2 of 40 | 0, 1, 0, 1 | 0.22 | 0.53 | 0.16, 0.51 | 6.0 | 0.112 |
| all four together | shipped | 29 of 40 | 0, 10, 9, 10 | - | 0.01 | -, - | 5.1 | 0.209 |
| all four together | f1_v1 | 21 of 40 | 0, 9, 3, 9 | 0.11 | 0.31 | 0.09, 0.30 | 5.1 | 0.135 |
| all four together | f3_v1 | 1 of 40 | 0, 0, 0, 1 | 0.17 | 0.41 | 0.10, 0.44 | 5.1 | 0.112 |
