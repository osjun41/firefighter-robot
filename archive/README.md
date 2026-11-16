# Archive

Code that is **not maintained and not imported** by anything else. It is kept to look up how an earlier result was made. It is not run by the tests or by CI, and its paths are those of the repository before the move, so it does not run from here without changes.

| Folder | What it is | Why it is here |
|---|---|---|
| [`rl_f1/`](rl_f1/README.md) | JAX/MJX training of the walking policy `f1_v1` (BHL legs with water tanks), its evaluation against the shipped BHL policy on the F2 design, and `mjx_check.py` (MJX against CPU MuJoCo cross-check) | The walking policy is trained in `sim/locomotion` (Isaac Lab) for the course robot. The water tanks of F1/F2 are removed |
| [`mission_campaign/`](mission_campaign/campaign.py) | Monte Carlo campaign of whole fire missions (50 missions per configuration) with its tables and figures | It used the BHL policy and the F2 model with water tanks. `sim/mission/` still runs single missions |

Do not import from this folder. To use something again, move it back and bring it up to date first.
