# Archive

Code that is **not maintained and not imported** by anything else. It is kept to look up how an earlier result was made. It is not run by the tests or by CI, and its paths are those of the repository before the move, so it does not run from here without changes.

| Folder | What it is | Why it is here |
|---|---|---|
| [`rl_f1/`](rl_f1/README.md) | JAX/MJX training of the walking policy `f1_v1` (BHL legs with water tanks), its evaluation against the shipped BHL policy on the F2 design, and `mjx_check.py` (MJX against CPU MuJoCo cross-check) | The walking policy is trained in `sim/locomotion` (Isaac Lab) for the course robot. The water tanks of F1/F2 are removed |
| [`mission_campaign/`](mission_campaign/campaign.py) | Monte Carlo campaign of whole fire missions (50 missions per configuration) with its tables and figures | It used the BHL policy and the F2 model with water tanks. `sim/mission/` still runs single missions |
| [`sim2sim_bhl/`](sim2sim_bhl/) | Reports, figures and scripts of the drive-risk study (`hw_risks`) and the F3 gap study (`gap_study`) on the BHL biped with the shipped BHL policy, f1_v1 and f3_v1 | The two studies now run the ASMR robot with the F3 parts and the baseline policy (`sim/sim2sim/`). The old numbers do not describe that robot. The `.py` files here are the old versions; they need `bhl_legacy.py` and the BHL models |
| [`payload_bhl/`](payload_bhl/) | `bhl_legacy.py` (the old BHL loop with the BHL policy), `models.py` (builds the BHL biped and F1/F2 models), `measured.py` and `masses.csv` (weighed-mass sheet for the BHL biped) | The simulation now runs only on the ASMR robot (`source/robot_models`). They need `third_party/` (`scripts/fetch_bhl_assets.sh`) and the BHL policy, which was deleted from git here (use git history) |
| [`payload_f2/`](payload_f2/final_f2.md) | Results of the F2 payload check (statics, walking, push test) on the BHL biped with the shipped BHL policy | `sim/payload/final_check.py` now checks the ASMR robot with the F3 parts and the policy by name; these numbers describe another robot |
| [`cad/`](cad/README.md) | Parametric CAD scripts (build123d), the print kit and the renders of the designs F1 to F3 on the BHL legs | The hardware team keeps its own CAD files in `cad/`. The files that the simulation reads moved to `source/robot_models/data/f3_layout.json`, `sim/mission/meshes/` and `sim/firescenes/` |

Do not import from this folder. To use something again, move it back and bring it up to date first.
