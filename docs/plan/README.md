# Project plan

Goal: a legs-only humanoid (the course Berkeley Humanoid Lite biped, design F3) finds a small fire with its camera, walks to it, aims a water jet with its pan-tilt and puts the fire out. A teammate pushes the water trolley behind it. A person at the e-stop can stop it at any time.

Final demo: **10 December 2026**. Plan of record: the team schedule board below. Review this page at every weekly meeting.

**Assumed:** the demo fire is a small tray fire (paper or wood, 10 to 30 cm) or a group of candles, on a fireproof tray, with an extinguisher and a person at the e-stop. Ask the TA which fire is allowed.

## Schedule and milestones

![Team schedule board](gantt.svg)

| Milestone | Week | Date | Pass criterion |
|---|---|---|---|
| M1 Robot assembly complete | W3 | 31 Oct | All 12 actuators are bench-tested. Legs and torso are assembled. All 12 drives answer on CAN with the robot hanging. |
| M2 Robot stands on gantry | W6 | 21 Nov | The robot stands for 60 s under the gantry with the walking policy, then walks on the gantry. Bus voltage stays under 26 V. |
| M3 Walks to fire and aims | W8 | 5 Dec | The robot sees a real fire, walks to it, aims and sprays. Three missions out of five succeed. |
| M4 Final demo | W9 | 10 Dec | |

`gantt.py` draws the chart: `uv run python docs/plan/gantt.py`.

## Workstreams and next tasks

Each workstream has one owner (see the team proposal). The links lead to the instructions.

| Workstream | State on 30 Oct | Next tasks |
|---|---|---|
| 3D printing and assembly | Pilot actuator assembled. Printing the 12 actuator modules (8 × 6512, 4 × 5010). | Weigh each printed part and each actuator before assembly ([masses.csv](../../sim/payload/masses.csv)). Print the F3 torso kit: 26 pieces, about 45 h ([print sheet](../../archive/cad/final/kit/print_sheet.md)). Do not print the F2 tanks. Assemble it ([kit guide](../../archive/cad/final/kit/README.md)). |
| Gearboxes and legs | Waiting for fasteners and parts. | Assemble and bench-test each actuator. Drive setup, CAN scan, zeroing, direction check, hanging joint test, joint identification ([legs/README.md](../../legs/README.md), parts A to G). |
| Torso | F3 layout and print kit built: no tanks, a hose anchor on the deck. | Fix the pan-tilt fastening and the rear deck screws in the CAD ([design.md](../design.md#open-design-points)). Measure the handles and the shelf on the robot before printing the deck. |
| Water module | Water trolley designed: IV stand, 2 L bag, pump box, boom, hose ([trolley.md](../hardware/trolley.md)). Pump: Kamoer KLP02 24 V with a DMC-101 controller. | Choose a heavy stand or ballast. Order the stand, the bag, the check valves, the pump and the controller. Run the bench tests ([water.md](../hardware/water.md)) and the trolley tests ([trolley.md](../hardware/trolley.md)). |
| Circuit | Wiring drawings SOB-E rev C: the pump circuit on the trolley with its own pack, a fourth e-stop contact for the pump signal ([electrical.md](../hardware/electrical.md)). | Get the DMC-101 datasheet and confirm the pump control (U8). Check that a fourth contact block fits the e-stop. Order the missing parts ([order list](../hardware/order_list.md)). Build the harness ([harness.csv](../hardware/harness.csv)) and the trolley circuit. Run the power-on tests before the first drive test. |
| Vision | Detector trained and exported for the RK3566 NPU. Screen bench and range test built. The B0201 arrives on 5 Nov. | Tests V1 to V11 of the [vision plan](vision.md): camera check, screen bench, calibration, range test with candles, then the board. |
| Simulation | F3 robot models from the CAD, the sim-to-real gap study and the weighing sheet. Our policy f3_v1 (in `policies/f3_v1`) trained for F3 with wide ranges. The mission simulation still runs F2. | Build the model with the weighed masses, repeat the gap study with the measured values, then train f3_v2. Mission campaign on F3 with the KLP02 jet ([sim/README.md](../../sim/README.md), [walking plan](walking.md)). |
| Control (sim-to-real) | Robot software tested in simulation with fake hardware. Joint identification tool `legs/sysid.py` and IMU level check built. f3_v1 starts to walk from a stand on the fake drives with the start command of the walk bridge. | Install the ROCK 3C, run the software-in-the-loop test on the board, then the test stages T0 to T2 of the [walking plan](walking.md). |
| Targeting | Aiming code tested in simulation, with the F3 jet (9.6 m/s). | Jet bench test to measure the jet speed and drag, camera-to-nozzle offset, then bench mode with the real pan-tilt ([aim/README.md](../../aim/README.md)). |
| Integration | — | From week 4: one integration test per week. Software-in-the-loop, then bench mode, then full mode ([robot/README.md](../../robot/README.md)). |

## Requirements and verification

Status words: built, tested in simulation, measured on laptop, measured on board, planned.

| Id | Requirement | Verification | Status |
|---|---|---|---|
| R1 | Detect a 10 cm flame at 3 m or more, recall 90 % or more | Recall by fire size on the test images, then a range test with our camera | measured on laptop: **not met yet**, recall 50 % at 1.8 m ([vision](../../vision/README.md)) |
| R2 | Fewer than 1 false fire per 100 images of fire-like objects | Hard-negative test set, then our room | measured on laptop |
| R3 | Walk at 0.2 m/s or more with the trolley hose, no fall in 10 runs | Simulation, then the test stages of the [walking plan](walking.md) | tested in simulation |
| R4 | Turn on the spot toward the fire | Simulation of our policy, then the robot | tested in simulation (archived: [archive/rl_f1](../../archive/rl_f1/README.md)) |
| R5 | The jet hits a 20 cm target at 1.5 m, nozzle 0.9 m high | Ballistics, then the bench jet test | tested in simulation ([water.md](../hardware/water.md)) |
| R6 | Spray for 60 s or more per fill | Pump and nozzle curves, then a timed bench test | tested in simulation: 4.8 min per 2 L bag |
| R7 | Put out the demo fire within 60 s of spraying | Mission simulation, then a static spray test | tested in simulation ([sim/mission](../../sim/mission/README.md)) |
| R8 | The e-stop cuts the drives and the pump in under 100 ms | Relay data, then a bench test with a scope or a slow-motion video | planned |
| R9 | No water reaches the electronics | Leak test of the hose and its joints, two check valves, boots or heat-shrink on the K1 terminals | planned |
| R10 | Any software fault stops the robot safely (legs damping, pump off) | Fault injection in the software-in-the-loop test, then on the board | tested in simulation ([robot](../../robot/README.md)) |
| R11 | The drives never trip on over-voltage while walking | Regen simulation, brake resistors, bus voltage log | tested in simulation |
| R12 | 20 minutes of operation per charge | Power budget, then a battery log | tested in simulation ([electrical.md](../hardware/electrical.md)) |

## Risks

| Risk | Likelihood | Effect | What we do |
|---|---|---|---|
| The robot cannot walk on the real legs. Earlier course years did not make it walk. | high | no walking demo | The [walking plan](walking.md): weigh the parts, identify the joints, train for the measured robot, test in stages. Demo mode B or Plan B as a fallback. |
| Drives trip on regen over-voltage | high without a fix | the robot collapses while walking | Brake resistors before the first walk ([electrical.md](../hardware/electrical.md)) |
| The DMC-101 input does not fit the proposed opto-isolator | medium | pump control redesign | Get the datasheet before the order is final |
| A wrong 5 V supply destroys the ROCK 3C | medium | lost computer | 5 V buck, check with a meter before the board goes on |
| A joint zero is wrong by 24° | medium | violent motion at start | Zero check in `legs/run_policy.py`, start hanging |
| A hose joint on the robot leaks onto the electronics | low | short circuit | Two check valves, a zip tie on every barb, boots on the K1 terminals, a leak test of the hose |
| The foot touches the battery ends or the pod at full knee range | high without the limit | broken parts | Software knee limit 1.4 rad (`robot/config.toml`) |
| The hose pulls the robot when the trolley lags | medium | fall | Breakaway clip at about 3 N, the person who pushes keeps the hose slack, a hose force in the training |
| The trolley tips over | medium with a light stand | pump and water on the floor | A heavy stand or ballast, the bag on the rear hook ([trolley.md](../hardware/trolley.md)) |
| The drive node ids differ from `legs/robot.py` | medium | wrong joint moves | Set the ids during the bring-up, then check each joint by hand |
| False fire on a lamp or a red object | medium | spray in the wrong place | Hard negatives in training, confirmation over several frames, operator at the e-stop |
| Printed gearboxes wear at 9 N·m | medium | backlash, falls | Stay at 6 N·m until a gearbox check |
| A fall damages the robot | medium | lost week | Gantry for all early tests, then a person with a strap on the handles, damping on any fault |

## Decisions

| Date | Decision | Why |
|---|---|---|
| 2026-10-08 | Legs-only robot, water from a pump | Concept review |
| 2026-10-15 | RGB camera only (Arducam B0201), no thermal | Cost and range. A wheeled backup can use the same camera. |
| 2026-10-15 | Drive settings: PASSTHROUGH, K 20, D 2, 6 N·m | Matched to the policy training |
| 2026-10-21 | Tank R2: two side tanks outside the torso (final design F2) | Team decision |
| 2026-10-22 | Pump: Kamoer KLP02 24 V with a DMC-101 controller | Team decision |
| 2026-10-29 | Design F3: the water bag and the pump on a trolley that a person pushes, a hose to the robot | A lighter robot (14.6 kg instead of 16.0 kg). Walking on the real robot is the main risk. |

## Plan B

The [walking plan](walking.md#decisions-and-dates) sets the decision dates. In mode B, the robot stands and sprays without walking. If the legs cannot stand reliably either, the team keeps the camera, detector, pan-tilt, pump and software on a wheeled base. Only the walking bridge changes: the mission software sends only a forward speed and a turn rate.
