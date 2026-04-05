"""
forward_kinematics.py — 5-DOF Forward Kinematics for the Letter Organizer Arm

v6: Added collision_check() to detect arm joints inside shelf bounding box.

Joint chain (from .wbt):
  Base → trans(0,0,0.025) → RotZ(q1) →           [J1 shoulder_yaw]
         trans(0,0,0.04)  → RotY(q2) →            [J2 shoulder_pitch]
         trans(0,0,0.2)   → RotY(q3) →            [J3 elbow_pitch]
         trans(0,0,0.17)  → RotY(q4) →            [J4 wrist_pitch]
         trans(0,0,0.02)  → RotZ(q5) →            [J5 wrist_roll]
         trans(0,0,0.06)                           [Gripper tip]

Zero-config tip position: (0, 0, 0.515)
"""

import numpy as np
import os
import yaml



def _rotz(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, 0, 0],
        [s,  c, 0, 0],
        [0,  0, 1, 0],
        [0,  0, 0, 1],
    ])


def _roty(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [ c, 0, s, 0],
        [ 0, 1, 0, 0],
        [-s, 0, c, 0],
        [ 0, 0, 0, 1],
    ])


def _trans(x, y, z):
    T = np.eye(4)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T



LINK_DEFS = [
    {"dz": 0.025, "rot": _rotz, "name": "shoulder_yaw"},
    {"dz": 0.04,  "rot": _roty, "name": "shoulder_pitch"},
    {"dz": 0.2,   "rot": _roty, "name": "elbow_pitch"},
    {"dz": 0.17,  "rot": _roty, "name": "wrist_pitch"},
    {"dz": 0.02,  "rot": _rotz, "name": "wrist_roll"},
]

GRIPPER_OFFSET = 0.06


class ForwardKinematics:
    N_JOINTS = 5

    def __init__(self, config_path=None):
        self.links = LINK_DEFS
        self.gripper_offset = GRIPPER_OFFSET
        self.joint_names = [l["name"] for l in self.links]

        if config_path and os.path.exists(config_path):
            with open(config_path, "r", encoding='utf-8') as f:
                cfg = yaml.safe_load(f)
            self.gripper_offset = cfg.get("gripper", {}).get(
                "tip_offset", GRIPPER_OFFSET
            )

    def fk(self, q):
        q = np.asarray(q, dtype=float)
        assert len(q) == self.N_JOINTS
        T = np.eye(4)
        for i, link in enumerate(self.links):
            T = T @ _trans(0, 0, link["dz"])
            T = T @ link["rot"](q[i])
        T = T @ _trans(0, 0, self.gripper_offset)
        return T

    def fk_all_transforms(self, q):
        q = np.asarray(q, dtype=float)
        assert len(q) == self.N_JOINTS
        transforms = []
        T = np.eye(4)
        for i, link in enumerate(self.links):
            T = T @ _trans(0, 0, link["dz"])
            T = T @ link["rot"](q[i])
            transforms.append(T.copy())
        transforms.append(T @ _trans(0, 0, self.gripper_offset))
        return transforms

    def get_position(self, q):
        return self.fk(q)[:3, 3]

    def compute_jacobian(self, q, delta=1e-6):
        q = np.asarray(q, dtype=float)
        J = np.zeros((3, self.N_JOINTS))
        p0 = self.get_position(q)
        for i in range(self.N_JOINTS):
            q_plus = q.copy()
            q_plus[i] += delta
            J[:, i] = (self.get_position(q_plus) - p0) / delta
        return J


    def collision_check(self, q, bbox):
        """
        Check whether any joint frame or the gripper tip is inside the
        shelf bounding box.

        bbox : dict with keys x_min, x_max, y_min, y_max, z_min, z_max
               (all in ROBOT BASE frame)

        Returns
        -------
        collides : bool   True if ANY joint is inside bbox
        depth    : float  max penetration depth (0 if no collision)
        details  : list   per-joint (name, pos, inside_flag)
        """
        transforms = self.fk_all_transforms(q)
        names = self.joint_names + ["gripper_tip"]

        collides = False
        max_depth = 0.0
        details = []

        for name, T in zip(names, transforms):
            p = T[:3, 3]
            inside = (bbox["x_min"] <= p[0] <= bbox["x_max"] and
                      bbox["y_min"] <= p[1] <= bbox["y_max"] and
                      bbox["z_min"] <= p[2] <= bbox["z_max"])
            if inside:
                dx = min(p[0] - bbox["x_min"], bbox["x_max"] - p[0])
                dy = min(p[1] - bbox["y_min"], bbox["y_max"] - p[1])
                dz = min(p[2] - bbox["z_min"], bbox["z_max"] - p[2])
                depth = min(dx, dy, dz)
                max_depth = max(max_depth, depth)
                collides = True
            else:
                depth = 0.0
            details.append((name, p.copy(), inside))

        return collides, max_depth, details

    def collision_cost(self, q, bbox, n_check=3):
        """
        Scalar penalty for arm joints intruding into shelf bbox.
        Only checks the first n_check joint frames (shoulder+elbow by default).
        Wrist and gripper are ALLOWED to enter shelf for placing letters.
        Returns 0.0 if no collision.
        """
        transforms = self.fk_all_transforms(q)
        cost = 0.0
        for T in transforms[:n_check]:
            p = T[:3, 3]
            if (bbox["x_min"] <= p[0] <= bbox["x_max"] and
                bbox["y_min"] <= p[1] <= bbox["y_max"] and
                bbox["z_min"] <= p[2] <= bbox["z_max"]):
                dx = min(p[0] - bbox["x_min"], bbox["x_max"] - p[0])
                dy = min(p[1] - bbox["y_min"], bbox["y_max"] - p[1])
                dz = min(p[2] - bbox["z_min"], bbox["z_max"] - p[2])
                cost += min(dx, dy, dz)
        return cost

