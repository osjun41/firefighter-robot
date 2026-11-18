"""Payload choice of the ASMR biped tasks: none, box_1kg (the default of the tasks) or f3 (the F3 parts).

The tasks have the payload as the startup event `add_payload` (mdp.add_body_payload). `select_payload` changes or
removes that event of a task config, so a script can train or play with another payload:

    train.py --task Velocity-ASMR-Biped-v0 --payload f3

The parts and the option names are in source/robot_models/robots/asmr2026_payload.py. In Isaac Lab the parts of
`f3` are merged into the base link (their rigid sum: mass, centre of mass and inertia by the parallel-axis theorem).
The deployment config (policy.yaml) keeps them as separate bodies for MuJoCo (`sim_payload`).
"""

import numpy as np
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg

import berkeley_humanoid_lite.tasks.locomotion.velocity.mdp as mdp
from source.robot_models.robots import asmr2026_payload as payloads


def payload_params(name):
    """Parameters of mdp.add_body_payload for a payload option (not for "none")."""
    items = payloads.parts(name)
    params = {"asset_cfg": SceneEntityCfg("robot", body_names="base")}
    if len(items) == 1:  # one box: mass, centre, size
        item = items[0]
        params.update(mass=item["mass"], position=tuple(item["position"]), size=tuple(item["size"]))
        return params
    mass, centre, inertia = payloads.combined(name)
    low = np.min([np.array(p["position"]) - np.array(p["size"]) / 2 for p in items], axis=0)
    high = np.max([np.array(p["position"]) + np.array(p["size"]) / 2 for p in items], axis=0)
    params.update(
        mass=float(mass),
        position=tuple(float(v) for v in centre),
        size=tuple(float(v) for v in high - low),  # bounding box, for information (the inertia is given)
        inertia=tuple(tuple(float(v) for v in row) for row in inertia),
        payload=name,
    )
    return params


def select_payload(env_cfg, name):
    """Set the payload of a task config. name None keeps the task's own, "none" removes it."""
    if name is None:
        return
    payloads.check(name)
    if name == "none":
        env_cfg.events.add_payload = None
    elif getattr(env_cfg.events, "add_payload", None) is None:
        env_cfg.events.add_payload = EventTerm(func=mdp.add_body_payload, mode="startup", params=payload_params(name))
    else:
        env_cfg.events.add_payload.params = payload_params(name)
    print(f"[INFO] payload: {payloads.describe(name)}")


def sim_payload(env_cfg):
    """The `sim_payload` entry of the deployment config for the payload of a task config, or None."""
    term = getattr(env_cfg.events, "add_payload", None)
    if term is None:
        return None
    params = term.params
    if "payload" in params:  # a named option with several parts: MuJoCo keeps them as separate bodies
        return payloads.sim_payload(params["payload"])
    return {key: (list(params[key]) if key != "mass" else params[key]) for key in ("mass", "position", "size")}
