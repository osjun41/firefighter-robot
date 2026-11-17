"""Export an RSL-RL checkpoint's actor to ONNX and write its deployment config, without Isaac Sim.

The actor is rebuilt from the checkpoint (linear layers with ELU in between, as in RslRlPpoActorCriticCfg
with activation "elu"), exported with the input name "obs" and a batch size of 1 like play.py, and a
deployment config is written next to it by copying a template config (same robot and task) and pointing it
at the new ONNX file. The result goes to policies/<name>/ (policy.onnx, policy.yaml and a copy of the checkpoint),
so every script and the robot select it by NAME afterwards (--policy NAME).

Usage (from the repository root):
    uv run --project sim/locomotion sim/locomotion/scripts/rsl_rl/export_onnx.py \
        --checkpoint logs/rsl_rl/asmr_biped/<run>/model_5999.pt --name my_policy [--template baseline] [--payload f3]

--payload NAME says which payload the policy was trained with (none, box_1kg or f3, as train.py --payload). It is
written to info.json and to the sim_payload of policy.yaml, so MuJoCo scripts weld the same parts. Without it,
the template's sim_payload stays and info.json does not name a payload.
"""

import argparse
import json
import shutil
import sys
from pathlib import Path

import torch
from omegaconf import OmegaConf

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))  # repository root: policies/policy_select.py
from policies import policy_select  # noqa: E402
from source.robot_models.robots import asmr2026_payload  # noqa: E402

parser = argparse.ArgumentParser(description="Export an RSL-RL actor to ONNX with a deployment config.")
parser.add_argument("--checkpoint", type=str, required=True, help="RSL-RL checkpoint (model_*.pt).")
parser.add_argument("--name", type=str, required=True, help="Name of the new policy: written to policies/<name>/.")
parser.add_argument("--force", action="store_true", help="Overwrite an existing policies/<name>/policy.onnx.")
parser.add_argument("--template", type=str, default=policy_select.DEFAULT,
                    help="Name of the policy whose policy.yaml is copied (same robot and task). Default baseline.")
parser.add_argument("--action_limit", type=float, default=None,
                    help="Action bound the policy was trained with (the runner's clip_actions), for the config.")
asmr2026_payload.add_argument(parser)
args = parser.parse_args()


def build_actor(state_dict):
    layers = []
    index = 0
    while f"actor.{index}.weight" in state_dict:
        weight = state_dict[f"actor.{index}.weight"]
        linear = torch.nn.Linear(weight.shape[1], weight.shape[0])
        linear.weight.data.copy_(weight)
        linear.bias.data.copy_(state_dict[f"actor.{index}.bias"])
        layers.append(linear)
        index += 2
        if f"actor.{index}.weight" in state_dict:
            layers.append(torch.nn.ELU())
    return torch.nn.Sequential(*layers).eval()


checkpoint = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
actor = build_actor(checkpoint["model_state_dict"])
num_obs = actor[0].in_features
if "/" in args.name or "\\" in args.name:
    raise ValueError(f"--name is a folder name in policies/, not a path: {args.name!r}")
out_dir = policy_select.POLICIES / args.name
if (out_dir / "policy.onnx").exists() and not args.force:
    raise FileExistsError(f"policies/{args.name}/ already has a policy.onnx; pick another --name or pass --force")
out_dir.mkdir(parents=True, exist_ok=True)
onnx_path = str(out_dir / "policy.onnx")
torch.onnx.export(actor, torch.zeros(1, num_obs), onnx_path, input_names=["obs"], output_names=["actions"],
                  opset_version=11)

config = OmegaConf.load(policy_select.config(args.template))
# repository-relative like the other policy.yaml files
config.policy_checkpoint_path = f"policies/{args.name}/policy.onnx"
if args.action_limit is not None:
    config.action_limit_lower, config.action_limit_upper = -args.action_limit, args.action_limit
if args.payload is not None:
    config.pop("sim_payload", None)
    if (payload := asmr2026_payload.sim_payload(args.payload)) is not None:
        config.sim_payload = payload
config_path = out_dir / "policy.yaml"
OmegaConf.save(config, config_path)
shutil.copy2(args.checkpoint, out_dir / Path(args.checkpoint).name)
info = {"trained_on": "asmr2026", "observation_size": 45}
if args.payload is not None:
    info["payload"] = args.payload
(out_dir / "info.json").write_text(json.dumps(info, indent=1) + "\n")
print(f"[INFO] {args.checkpoint} (iteration {checkpoint.get('iter')}, {num_obs} observations) -> {onnx_path}")
print(f"[INFO] Deployment config: {config_path}")
