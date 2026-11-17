"""Leg poses and placement on the floor with flat feet, for the ASMR robot (sim/sim2sim/run.py)."""
import mujoco
import numpy as np

# Pitch joints of the right leg turn the other way than the left ones (mirrored joint axes): the right leg gets -value.
STAND = {"hip_pitch": 0.585, "knee_pitch": -0.952, "ankle_pitch": 0.367}  # left leg of the ASMR default pose
CROUCH = {"hip_pitch": 0.85, "knee_pitch": -1.35, "ankle_pitch": 0.5}  # same hip+knee+ankle sum as STAND (0), inside the limits
SIDE = {"left": 1.0, "right": -1.0}
SAME = {"left": 1.0, "right": 1.0}  # the BHL biped (models.py): both legs take the same pitch values


def leg_q(pose, sway=0.0, side=SIDE):
    """Joint dict for both legs. sway < 0 moves the pelvis over the left foot, sway > 0 over the right foot
    (hip roll and ankle roll of both legs turn by the same angle, so the legs form a parallelogram)."""
    q = {f"leg_{s}_{k}_joint": side[s] * v for s in ("left", "right") for k, v in pose.items()}
    for s in ("left", "right"):
        q[f"leg_{s}_hip_roll_joint"], q[f"leg_{s}_ankle_roll_joint"] = sway, -sway
    return q


def foot_geom(m, side):
    return next(i for i in range(m.ngeom) if m.geom_bodyid[i] == m.body(f"leg_{side}_ankle_roll").id
                and m.geom_type[i] == mujoco.mjtGeom.mjGEOM_BOX)


def sole(m, d, side):
    """Centre of the sole (bottom face of the foot collision box), world frame. Valid when the foot is flat."""
    g = foot_geom(m, side)
    R, c, h = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g], m.geom_size[g]
    up = np.argmax(np.abs(R[2]))  # box axis that points up
    return c - R[:, up] * h[up] * np.sign(R[2, up])


def sole_half(m, side):
    """Half length (x) and half width (y) of the sole when the foot is flat (the base at identity, all joints zero)."""
    d = mujoco.MjData(m)
    d.qpos[3] = 1
    mujoco.mj_kinematics(m, d)
    g = foot_geom(m, side)
    R = np.abs(d.geom_xmat[g].reshape(3, 3))
    return float(R[0] @ m.geom_size[g]), float(R[1] @ m.geom_size[g])


def place(m, q, stance="left", gap=0.0):
    """qpos with joints q, base turned so the stance foot is flat, and base height so that the stance sole is at z = gap."""
    d = mujoco.MjData(m)
    d.qpos[3] = 1
    mujoco.mj_kinematics(m, d)
    b = d.body(f"leg_{stance}_ankle_roll")
    flat = b.xmat.reshape(3, 3).copy()  # foot orientation at zero joints: flat on the floor
    for j, v in q.items():
        d.joint(j).qpos = v
    mujoco.mj_kinematics(m, d)
    R = flat @ b.xmat.reshape(3, 3).T  # turns the current foot orientation back to flat
    mujoco.mju_mat2Quat(d.qpos[3:7], R.flatten())
    mujoco.mj_kinematics(m, d)
    d.qpos[2] += gap - sole(m, d, stance)[2]
    return d.qpos.copy()


def stand_qpos(m, pose, gap=0.001, side=SIDE):
    return place(m, leg_q(pose, side=side), gap=gap)
