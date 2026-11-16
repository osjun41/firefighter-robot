# Upstream

This repository started from the open-source Berkeley Humanoid Lite project.
The upstream code was copied into this repository (not linked as submodules)
so that it can be modified for the course robot.

| Upstream repository | Commit | Date | Location in this repo |
|:--|:--|:--|:--|
| [HybridRobotics/Berkeley-Humanoid-Lite](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite) | `edc4738` | 2025-09-10 | `/` |
| [HybridRobotics/Berkeley-Humanoid-Lite-Assets](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite-Assets) | `fc90fed` | 2025-09-07 | `source/berkeley_humanoid_lite_assets/` |
| [HybridRobotics/Berkeley-Humanoid-Lite-Lowlevel](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite-Lowlevel) | `652777c` | 2025-09-07 | `source/berkeley_humanoid_lite_lowlevel/` |

`edc4738` is the last upstream commit on Isaac Sim 4.5.0 / Isaac Lab 2.1.0 / Python 3.10,
the versions in the upstream documentation. Later upstream commits move to Isaac Sim 5.1.

The first commit of this repository is the verbatim import. Run
`git diff <first commit> -- <path>` to see what has changed since.

Upstream code is under the MIT License (`LICENCE`); other upstream assets are under CC BY-SA 4.0.
