"""Warm-start a policy whose observations gained new terms at the end.

Appends zero-weight input columns to the first layer of the actor and of the critic (and to their Adam
moments), so the expanded network computes exactly what the original did until training changes the new
weights. Use it to fine-tune a trained policy on a task that observes more, e.g. a push signal.

With --log_std the learned action noise "std" is stored as "log_std" for noise_std_type="log".

Usage (from the repository root):
    uv run --project scripts/isaaclab scripts/isaaclab/rsl_rl/expand_checkpoint.py --input logs/rsl_rl/biped/<run>/model_5999.pt \
        --output logs/rsl_rl/biped_push_signal/init/model_5999.pt --actor_inputs 1 --critic_inputs 2
"""

import argparse
import os

import torch

parser = argparse.ArgumentParser(description="Append zero-weight inputs to an RSL-RL actor-critic checkpoint.")
parser.add_argument("--input", type=str, required=True, help="Checkpoint to expand.")
parser.add_argument("--output", type=str, required=True, help="Expanded checkpoint.")
parser.add_argument("--actor_inputs", type=int, default=0, help="Observations appended for the actor.")
parser.add_argument("--critic_inputs", type=int, default=0, help="Observations appended for the critic.")
parser.add_argument(
    "--log_std", action="store_true", help="Switch the action noise to the log parameterization (noise_std_type='log')."
)
args = parser.parse_args()


def pad_columns(tensor: torch.Tensor, extra: int) -> torch.Tensor:
    return torch.cat([tensor, torch.zeros(tensor.shape[0], extra, dtype=tensor.dtype)], dim=1)


checkpoint = torch.load(args.input, map_location="cpu", weights_only=False)
model = checkpoint["model_state_dict"]
expanded = {}
for name, extra in (("actor.0.weight", args.actor_inputs), ("critic.0.weight", args.critic_inputs)):
    if extra > 0:
        old_shape = tuple(model[name].shape)
        model[name] = pad_columns(model[name], extra)
        expanded[old_shape] = extra
        print(f"{name}: {old_shape} -> {tuple(model[name].shape)}")

# the Adam moments of the expanded weights get zero columns too (matched by shape)
for state in checkpoint.get("optimizer_state_dict", {}).get("state", {}).values():
    for key in ("exp_avg", "exp_avg_sq"):
        if key in state and tuple(state[key].shape) in expanded:
            state[key] = pad_columns(state[key], expanded[tuple(state[key].shape)])

if args.log_std and "std" in model:
    # a learned std can step below zero ("normal expects all elements of std >= 0"); log_std cannot
    std = model.pop("std")
    model["log_std"] = torch.log(std.clamp(min=1e-3))
    # the policy's own parameter comes first in parameters(), so its Adam state is entry 0: start it fresh
    state = checkpoint.get("optimizer_state_dict", {}).get("state", {})
    if 0 in state and tuple(state[0]["exp_avg"].shape) == tuple(std.shape):
        del state[0]
    print(f"std {std.numpy().round(3).tolist()} -> log_std")

os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
torch.save(checkpoint, args.output)
print(f"Saved {args.output} (iteration {checkpoint.get('iter')})")
