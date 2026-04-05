"""
inverse_kinematics.py — 5-DOF Inverse Kinematics for the Letter Organizer Arm
"""

import numpy as np
import os
import yaml
from forward_kinematics import ForwardKinematics


class InverseKinematics:

    DEFAULT_LIMITS = [
        (-3.14159, 3.14159),
        (-2.2,     2.2),
        (-2.5,     2.5),
        (-2.2,     2.2),
        (-3.14159, 3.14159),
    ]

    def __init__(self, config_path=None):
        self.fk = ForwardKinematics(config_path)
        if config_path and os.path.exists(config_path):
            self._load_limits(config_path)
        else:
            self.limits = list(self.DEFAULT_LIMITS)
        self.n_joints = self.fk.N_JOINTS

    def _load_limits(self, path):
        with open(path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        self.limits = []
        for j in cfg["joints"]:
            lim = j.get("limits", [-3.14159, 3.14159])
            self.limits.append(tuple(lim))

    def clamp_joints(self, q):
        q = np.array(q, dtype=float)
        for i in range(self.n_joints):
            lo, hi = self.limits[i]
            q[i] = np.clip(q[i], lo, hi)
        return q

    def solve(self, target_pos, q_init=None, max_iter=200, tol=1e-3,
              damping=0.05, n_restarts=10, require_downward=False,
              shelf_bbox=None):
        target = np.asarray(target_pos, dtype=float)

        if require_downward:
            pitch_target = np.pi if target[0] >= 0 else -np.pi
        else:
            pitch_target = 0.0

        best_q   = None
        best_err = float("inf")

        seeds = self._generate_seeds(q_init, n_restarts, require_downward,
                                     pitch_target, target)

        for seed in seeds:
            q, err = self._dls_iterate(target, seed, max_iter, tol, damping,
                                       require_downward, pitch_target,
                                       shelf_bbox)

            if shelf_bbox is not None:
                cc = self.fk.collision_cost(q, shelf_bbox)
                if cc > 0:
                    err += 10.0 + cc * 100.0

            if err < best_err:
                best_err = err
                best_q   = q.copy()
            if err < tol:
                return best_q, True, err

        return best_q, best_err < tol, best_err

    def _generate_seeds(self, q_init, n_restarts, require_downward,
                        pitch_target, target=None):
        seeds = []
        seeds.append(np.zeros(self.n_joints))
        if q_init is not None:
            seeds.append(np.asarray(q_init, dtype=float))

        sign = 1.0 if pitch_target > 0 else -1.0

        if require_downward:
            base_seeds = [
                [0,    0.6,  1.3,  1.24],
                [0,    0.5,  1.5,  1.14],
                [0,    0.8,  1.0,  1.34],
                [0,    0.7,  1.1,  1.34],
                [0.3,  0.6,  1.3,  1.24],
                [-0.3, 0.6,  1.3,  1.24],
                [0,    1.0,  1.2,  0.94],
                [0,    0.8,  1.5,  0.84],
                [0,    1.2,  1.0,  0.94],
                [1.0,  1.0,  1.2,  0.94],
                [-1.0, 1.0,  1.2,  0.94],
                [0.5,  0.9,  1.3,  0.94],
                [-0.5, 0.9,  1.3,  0.94],
                [0,    0.6,  1.8,  0.74],
                [0,    0.4,  0.8,  1.94],
                [0.3,  0.4,  0.8,  1.94],
                [-0.3, 0.4,  0.8,  1.94],
                [0,    0.3,  1.0,  1.84],
                [0.5,  0.3,  1.0,  1.84],
                [-0.5, 0.3,  1.0,  1.84],
            ]
            for cfg in base_seeds:
                seeds.append(np.array([cfg[0],
                                       sign * cfg[1],
                                       sign * cfg[2],
                                       sign * cfg[3],
                                       0.0]))
        else:
            heuristic_configs = [
                [0,    -0.5, -0.5, -0.5, 0],
                [0,    -0.8, -1.0,  0.3, 0],
                [1.57, -0.5, -0.5, -0.5, 0],
                [-1.57,-0.5, -0.5, -0.5, 0],
                [0,     0.3,  0.5, -0.8, 0],
                [0.78, -0.3, -0.8, -0.2, 0],
                [0,     0.5,  0.5,  0.5, 0],
                [0,     0.8,  1.0, -0.3, 0],
            ]
            for cfg in heuristic_configs:
                seeds.append(np.array(cfg))

        remaining = max(0, n_restarts - len(seeds))
        for _ in range(remaining):
            q_rand = np.array([np.random.uniform(lo, hi)
                               for lo, hi in self.limits])
            if require_downward:
                q_rand[3] = pitch_target - q_rand[1] - q_rand[2]
                q_rand = self.clamp_joints(q_rand)
            seeds.append(q_rand)

        return seeds[:n_restarts + 2]

    def _dls_iterate(self, target, q_init, max_iter, tol, damping,
                     require_downward, pitch_target, shelf_bbox=None):
        q = self.clamp_joints(q_init.copy())
        pitch_weight = 0.5 if require_downward else 0.0
        col_weight   = 2.0

        for _ in range(max_iter):
            pos       = self.fk.get_position(q)
            error_pos = target - pos
            err_norm  = np.linalg.norm(error_pos)

            if require_downward:
                pitch_err  = pitch_target - (q[1] + q[2] + q[3])
                error_aug  = np.append(error_pos, pitch_weight * pitch_err)
                J_pos      = self.fk.compute_jacobian(q)
                J_pitch    = pitch_weight * np.array([[0, 1, 1, 1, 0]])
                J          = np.vstack([J_pos, J_pitch])
                JJT        = J @ J.T
                dq = J.T @ np.linalg.solve(
                    JJT + damping**2 * np.eye(4), error_aug)
            else:
                J   = self.fk.compute_jacobian(q)
                JJT = J @ J.T
                dq  = J.T @ np.linalg.solve(
                    JJT + damping**2 * np.eye(3), error_pos)

            if shelf_bbox is not None:
                dq_col = self._collision_gradient(q, shelf_bbox)
                if np.linalg.norm(dq_col) > 0:
                    dq += col_weight * dq_col

            dq_norm = np.linalg.norm(dq)
            if dq_norm > 0.3:
                dq *= 0.3 / dq_norm

            q = self.clamp_joints(q + dq)
            if err_norm < tol:
                return q, err_norm

        pos = self.fk.get_position(q)
        return q, np.linalg.norm(target - pos)

    def _collision_gradient(self, q, bbox, delta=1e-4):
        c0 = self.fk.collision_cost(q, bbox)
        if c0 == 0:
            return np.zeros(self.n_joints)
        grad = np.zeros(self.n_joints)
        for i in range(self.n_joints):
            q_plus = q.copy()
            q_plus[i] += delta
            grad[i] = (self.fk.collision_cost(q_plus, bbox) - c0) / delta
        return -grad

    def interpolate_trajectory(self, q_start, q_end, n_steps=20):
        q_s = np.asarray(q_start, dtype=float)
        q_e = np.asarray(q_end,   dtype=float)
        traj = []
        for i in range(n_steps + 1):
            s = 0.5 * (1.0 - np.cos(np.pi * i / n_steps))
            traj.append(self.clamp_joints(q_s + s * (q_e - q_s)))
        return traj


