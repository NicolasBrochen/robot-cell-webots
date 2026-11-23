"""Kinematics of library robots (Webots UR5e, FANUC LR Mate 200iD) carrying a Robotiq 2F-85.

The chains are copied from the Webots PROTOs: for every joint, the child frame is
    T_child = Trans(anchor) . Rot(axis, q) . Trans(-anchor) . T_endpoint0
(axis, anchor and the zero-pose endPoint transform expressed in the parent frame).
The tool transform goes from the last link to the frame of the Robotiq2f85Gripper node
mounted in the robot toolSlot (its +z is the approach direction, fingers close along y),
then `tcp_offset` metres along that +z.

Validated against Webots R2025a (controllers/kin_calib): < 1 mm / < 0.1 deg.
"""
import math

import numpy as np


def rot(axis, angle):
    """Rotation matrix about a (not necessarily unit) axis."""
    w = np.asarray(axis, float)
    w = w / np.linalg.norm(w)
    K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


def hom(R=None, t=(0, 0, 0)):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    T[:3, 3] = t
    return T


def log_rot(R):
    c = max(-1.0, min(1.0, (np.trace(R) - 1) / 2))
    th = math.acos(c)
    if th < 1e-9:
        return np.zeros(3)
    if math.pi - th < 1e-6:                              # 180 deg: axis from the diagonal
        A = (R + np.eye(3)) / 2
        k = int(np.argmax(np.diag(A)))
        v = A[:, k] / math.sqrt(max(A[k, k], 1e-12))
        return v * th
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v * th / (2 * math.sin(th))


class Chain:
    """Serial chain of revolute joints in the Webots convention.

    joints : list of (axis, anchor, endpoint_translation, endpoint_rotation_axis_angle or None)
    tool   : 4x4 from the last link frame to the gripper frame
    names  : motor names (Webots devices)
    limits : list of (min, max) joint limits (rad)
    """

    def __init__(self, name, joints, tool, names, limits, tcp_offset=0.0):
        self.name = name
        self.names = names
        self.limits = limits
        self.tool = tool
        self.tcp_offset = tcp_offset
        self._A = []
        for axis, anchor, ep_t, ep_r in joints:
            E = hom(rot(ep_r[:3], ep_r[3]) if ep_r is not None else None, ep_t)
            self._A.append((np.asarray(axis, float), np.asarray(anchor, float), E))

    def fk(self, q, tcp_offset=None):
        """4x4 pose of the gripper frame (+tcp_offset along its z) in the robot base frame."""
        T = np.eye(4)
        for (axis, anchor, E), qi in zip(self._A, q):
            T = T @ hom(t=anchor) @ hom(rot(axis, qi)) @ hom(t=-anchor) @ E
        off = self.tcp_offset if tcp_offset is None else tcp_offset
        return T @ self.tool @ hom(t=(0, 0, off))

    def _err(self, q, p, R, off):
        T = self.fk(q, off)
        return np.concatenate([p - T[:3, 3], log_rot(R @ T[:3, :3].T)])

    def _dls(self, q, p, R, off, iters, lam):
        for _ in range(iters):
            e = self._err(q, p, R, off)
            if np.linalg.norm(e) < 1e-10:
                break
            J = np.zeros((6, len(q)))
            for i in range(len(q)):
                dq = np.zeros(len(q))
                dq[i] = 1e-7
                J[:, i] = (self._err(q, p, R, off) - self._err(q + dq, p, R, off)) / 1e-7
            q = q + J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), e)
        return q, float(np.linalg.norm(self._err(q, p, R, off)))

    def ik(self, p_target, R_target, q0, tcp_offset=None, iters=300, lam=0.02, tol=1e-6, restarts=8):
        """Damped least squares IK on position + orientation (robot base frame), starting from q0
        (keep q0 = previous solution along a path for continuity). If it does not converge, it retries
        with other damping factors and then from deterministically perturbed seeds around q0.
        Raises RuntimeError if the residual stays above `tol`."""
        p, R = np.asarray(p_target, float), np.asarray(R_target, float)
        q0 = np.array(q0, float)
        best_q, best_e = q0, float("inf")
        rng = np.random.default_rng(12345)
        seeds = [q0] + [q0 + rng.normal(0, 0.4, len(q0)) for _ in range(restarts)]
        for k, seed in enumerate(seeds):
            for l, n in ((lam, iters), (0.1, 3 * iters), (0.005, 3 * iters)) if k == 0 else ((lam, 2 * iters),):
                q, e = self._dls(seed.copy(), p, R, tcp_offset, n, l)
                if e < best_e:
                    best_q, best_e = q, e
                if best_e <= tol:
                    return best_q
        raise RuntimeError(f"{self.name}: IK did not converge (residual {best_e:.2e})")


def world(robot, base_translation=(0, 0, 0), base_yaw=0.0):
    """4x4 transform robot base -> world (robot placed at base_translation, yawed about z)."""
    return hom(rot((0, 0, 1), base_yaw), base_translation)


PI2 = math.pi / 2
TWO_PI = 2 * math.pi

# Webots UR5e (projects/robots/universal_robots/protos/UR5e.proto, R2025a)
UR5E = Chain(
    "UR5e",
    joints=[
        ((0, 0, 1), (0, 0, 0.163), (0, 0, 0.163), None),                     # shoulder_pan
        ((0, 1, 0), (0, 0.138, 0), (0, 0.138, 0), (0, 1, 0, PI2)),            # shoulder_lift
        ((0, 1, 0), (0, -0.131, 0.425), (0, -0.131, 0.425), None),            # elbow
        ((0, 1, 0), (0, 0, 0.392), (0, 0, 0.392), (0, 1, 0, PI2)),            # wrist_1
        ((0, 0, 1), (0, 0.127, 0), (0, 0.127, 0), None),                      # wrist_2
        ((0, 1, 0), (0, 0, 0.1), (0, 0, 0.1), None),                          # wrist_3
    ],
    # toolSlot Pose T(0, 0.1, 0) in wrist_3_link, Robotiq mounted with rotation (1 0 0 -pi/2): its z = +y of wrist_3_link
    tool=hom(t=(0, 0.1, 0)) @ hom(rot((1, 0, 0), -PI2)),
    names=["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
    limits=[(-TWO_PI, TWO_PI), (-TWO_PI, TWO_PI), (-math.pi, math.pi), (-TWO_PI, TWO_PI), (-TWO_PI, TWO_PI), (-TWO_PI, TWO_PI)],
)
UR5E_GRIPPER_MOUNT = "rotation 1 0 0 -1.5708"   # Robotiq2f85Gripper field to use in the UR5e toolSlot

# FANUC LR Mate 200iD (protos/FanucLrMate200iD.proto, urdf2webots from ROS-Industrial)
FANUC = Chain(
    "FANUC LR Mate 200iD",
    joints=[
        ((0, 0, 1), (0, 0, 0.33), (0, 0, 0.33), None),          # joint_1
        ((0, 1, 0), (0.05, 0, 0), (0.05, 0, 0), None),          # joint_2
        ((0, -1, 0), (0, 0, 0.33), (0, 0, 0.33), None),         # joint_3
        ((-1, 0, 0), (0, 0, 0.035), (0, 0, 0.035), None),       # joint_4
        ((0, -1, 0), (0.335, 0, 0), (0.335, 0, 0), None),       # joint_5
        ((-1, 0, 0), (0.08, 0, 0), (0.08, 0, 0), None),         # joint_6
    ],
    # toolSlot inside the "tool0" Solid, rotation (0.707107 0 0.707107 pi): its z = +x of link_6 (flange axis)
    tool=hom(rot((0.707107, 0, 0.707107), math.pi)),
    names=[f"joint_{i}" for i in range(1, 7)],
    limits=[(-2.96706, 2.96706), (-1.74533, 2.53073), (-1.22173, 3.57792), (-3.31613, 3.31613),
            (-2.18166, 2.18166), (-6.28319, 6.28319)],
)
FANUC_GRIPPER_MOUNT = ""                          # no rotation needed in the FANUC toolSlot
