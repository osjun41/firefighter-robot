# Upstream

The training code started from the open-source [Berkeley Humanoid Lite](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite) (BHL) project, through the SOBANG locomotion repository. The upstream code was copied and changed for the course robot; it is not linked as a submodule.

| Upstream repository | Commit | Date | Here |
|:--|:--|:--|:--|
| [HybridRobotics/Berkeley-Humanoid-Lite](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite) | `edc4738` | 2025-09-10 | `packages/berkeley_humanoid_lite/` (Isaac Lab tasks, MuJoCo environment), changed |
| [HybridRobotics/Berkeley-Humanoid-Lite-Lowlevel](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite-Lowlevel) | `652777c` | 2025-09-07 | `packages/berkeley_humanoid_lite_lowlevel/`, only `policy/` |
| [HybridRobotics/Berkeley-Humanoid-Lite-Assets](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite-Assets) | `fc90fed` | 2025-09-07 | **not copied** (see below) |

`edc4738` is the last upstream commit on Isaac Sim 4.5.0 / Isaac Lab 2.1.0 / Python 3.10, the versions in the upstream documentation. Later upstream commits move to Isaac Sim 5.1.

## What is not here

- **The BHL robot** (its URDF, MJCF, USD, meshes and Isaac Lab config) is not needed to train or evaluate the course robot, so it is not in this repository. The BHL `humanoid` task (`Velocity-Berkeley-Humanoid-Lite-v0`) and the fallback of `MujocoEnv` to a BHL scene were removed with it.
- **The course robot** is `source/robot_models/` at the repository root: the course model (asmr2026-humanoid-v1) with the changes for our hardware. Training, sim2sim and the robot software all read it from there.
- The tasks named `Velocity-Berkeley-Humanoid-Lite-Biped-*` are no longer registered. Their code (`config/biped*`) stays as the base configuration of the `config/asmr_biped*` tasks, and uses the course robot.

## Get the originals

```bash
scripts/fetch_bhl_assets.sh        # BHL robot model and meshes (commit fc90fed) into third_party/; CC BY-SA 4.0
                                   # only cad/ reads it (the CAD scripts draw parts on the BHL meshes); no simulation and no locomotion script does

git clone https://github.com/HybridRobotics/Berkeley-Humanoid-Lite.git && git -C Berkeley-Humanoid-Lite checkout edc4738
git clone https://github.com/HybridRobotics/Berkeley-Humanoid-Lite-Lowlevel.git && git -C Berkeley-Humanoid-Lite-Lowlevel checkout 652777c
```

Compare a file with the upstream one by `diff` against these clones (`source/berkeley_humanoid_lite/...` of the first repository is `packages/berkeley_humanoid_lite/...` here).

Upstream code is under the MIT License; other upstream assets are under CC BY-SA 4.0.
