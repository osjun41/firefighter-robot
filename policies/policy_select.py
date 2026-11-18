"""Select a walking policy by name from policies/.

Every policy lives in `policies/<name>/` (policy.onnx, policy.yaml, checkpoints, params/).
Code and scripts choose one by NAME only, never by file path. With no name, `baseline` is used.

    from policies import policy_select     # repo root on sys.path
    policy_select.onnx("brace_m4")         # policies/brace_m4/policy.onnx
    policy_select.config()                 # policies/baseline/policy.yaml

CLI scripts: `policy_select.add_argument(parser)` adds `--policy NAME` (default baseline).
To use a new policy, export or copy it to `policies/<name>/` first (see policies/README.md).
Each policy also says which robot model it was trained for (info.json: "trained_on": "asmr2026" or "bhl"; a folder with
a policy.yaml and no info.json is an ASMR policy). Code for the ASMR robot passes robot="asmr2026"; for a policy trained on
another robot model it WARNS (joint convention, default pose and gains differ, so it may behave oddly) and still loads it.
Pass strict=True to refuse instead.
Standard library only, so it also runs on the robot board.
"""
import json
import warnings
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
POLICIES = ROOT / "policies"
DEFAULT = "baseline"


def names():
    """Sorted names of all policy folders that hold a policy.onnx or policy.yaml."""
    return sorted(p.name for p in POLICIES.iterdir()
                  if p.is_dir() and ((p / "policy.onnx").exists() or (p / "policy.yaml").exists()))


def folder(name=None):
    """Folder of policy `name` (default baseline). Raises with the available names if unknown."""
    name = (name or DEFAULT).strip()
    if "/" in name or "\\" in name or name.endswith((".onnx", ".yaml", ".pt")) or name in (".", ".."):
        raise ValueError(f"give a policy NAME from policies/, not a path: {name!r}. Available: {', '.join(names())}")
    path = POLICIES / name
    if not path.is_dir():
        raise FileNotFoundError(f"no policy {name!r} in policies/. Available: {', '.join(names())}")
    return path


def _file(name, filename):
    path = folder(name) / filename
    if not path.is_file():
        raise FileNotFoundError(f"policy {(name or DEFAULT)!r} has no {filename} ({path})")
    return path


def info(name=None):
    """What the policy was trained for: info.json, or {"trained_on": "asmr2026"} for a folder with a policy.yaml."""
    path = folder(name) / "info.json"
    if path.is_file():
        return json.loads(path.read_text())
    if (folder(name) / "policy.yaml").is_file():
        return {"trained_on": "asmr2026"}
    raise FileNotFoundError(f"policy {(name or DEFAULT)!r} has neither info.json nor policy.yaml, so its robot model is unknown")


def mismatch(name=None, robot=None):
    """Text describing the robot-model mismatch of policy `name` against `robot`, or "" when it fits (or robot is None)."""
    trained = info(name)["trained_on"]
    if robot is None or trained == robot:
        return ""
    return (f"policy {(name or DEFAULT)!r} was trained for {trained!r}, not {robot!r}: its joint convention, default pose "
            f"and gains differ, so it may behave oddly. Test it in the air first.")


def onnx(name=None, robot=None, strict=False):
    """policies/<name>/policy.onnx. With robot="asmr2026", warn when the policy was trained for another robot model.

    The policy is still returned (the point is to try it). strict=True raises ValueError instead of warning.
    """
    if text := mismatch(name, robot):
        if strict:
            raise ValueError(text)
        warnings.warn(text, stacklevel=2)
    return _file(name, "policy.onnx")


def config(name=None):
    """policies/<name>/policy.yaml (deployment config: joints, gains, scales)."""
    return _file(name, "policy.yaml")


def checkpoint(name=None):
    """Training checkpoint policies/<name>/model_*.pt (the highest iteration), for scripts that rebuild the network."""
    found = sorted(folder(name).glob("model_*.pt"), key=lambda p: int(p.stem.split("_")[1]))
    if not found:
        raise FileNotFoundError(f"policy {(name or DEFAULT)!r} has no model_*.pt checkpoint in {folder(name)}")
    return found[-1]


def add_argument(parser, multiple=False):
    """Add `--policy NAME` (default baseline) to an argparse parser.

    multiple=True lets the option repeat (`--policy a --policy b`); read the result with chosen(args.policy).
    """
    help_text = f"policy folder name in policies/ (default {DEFAULT}). Available: {', '.join(names())}"
    if multiple:
        parser.add_argument("--policy", action="append", default=None, metavar="NAME", help=help_text)
    else:
        parser.add_argument("--policy", default=DEFAULT, metavar="NAME", help=help_text)


def chosen(value):
    """List of policy names from a `--policy` option made with multiple=True (baseline when none was given)."""
    return list(value) if value else [DEFAULT]
