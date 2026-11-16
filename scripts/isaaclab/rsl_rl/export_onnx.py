"""Export an RSL-RL checkpoint's actor to ONNX and write its deployment config, without Isaac Sim.

The actor is rebuilt from the checkpoint (linear layers with ELU in between, as in RslRlPpoActorCriticCfg
with activation "elu"), exported with the input name "obs" and a batch size of 1 like play.py, and a
deployment config is written next to it by copying a template config (same robot and task) and pointing it
at the new ONNX file. Unlike play.py this leaves configs/policy_latest.yaml alone.

Usage (from the repository root):
    uv run --project scripts/isaaclab scripts/isaaclab/rsl_rl/export_onnx.py --checkpoint logs/rsl_rl/biped_push/<run>/model_7499.pt \
        --template configs/policy_latest.yaml --out_dir logs/rsl_rl/biped_push/<run>/exported_7499
"""

import argparse
import os

import torch
from omegaconf import OmegaConf

parser = argparse.ArgumentParser(description="Export an RSL-RL actor to ONNX with a deployment config.")
parser.add_argument("--checkpoint", type=str, required=True, help="RSL-RL checkpoint (model_*.pt).")
parser.add_argument("--template", type=str, required=True, help="Deployment config to copy (same robot and task).")
parser.add_argument("--out_dir", type=str, required=True, help="Directory for policy.onnx and policy.yaml.")
parser.add_argument("--action_limit", type=float, default=None,
                    help="Action bound the policy was trained with (the runner's clip_actions), for the config.")
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
os.makedirs(args.out_dir, exist_ok=True)
onnx_path = os.path.abspath(os.path.join(args.out_dir, "policy.onnx"))
torch.onnx.export(actor, torch.zeros(1, num_obs), onnx_path, input_names=["obs"], output_names=["actions"],
                  opset_version=11)

config = OmegaConf.load(args.template)
config.policy_checkpoint_path = onnx_path
if args.action_limit is not None:
    config.action_limit_lower, config.action_limit_upper = -args.action_limit, args.action_limit
config_path = os.path.join(args.out_dir, "policy.yaml")
OmegaConf.save(config, config_path)
print(f"[INFO] {args.checkpoint} (iteration {checkpoint.get('iter')}, {num_obs} observations) -> {onnx_path}")
print(f"[INFO] Deployment config: {config_path}")
