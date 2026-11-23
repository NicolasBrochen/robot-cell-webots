"""Kinematics of the two robots of the cell (shared by the generators and the checks).

SCARA (4 axes, RRPR)
    J1, J2 : revolute about z, links L1 = 0.40 m, L2 = 0.30 m
    J3     : prismatic, -z (stroke 0..0.15 m)
    J4     : revolute about z (tool yaw), J4 = -(J1 + J2) keeps the gripper aligned with world x
    finger centre height = base_z + 0.175 - d3

6-axis arm (spherical wrist), product of exponentials, zero pose = upper arm vertical,
forearm horizontal along +x:
    J1 z @ (0,0,0)      J2 y @ (0,0,0.30)    J3 y @ (0,0,0.65)
    J4 x @ (0.15,0,0.65) J5 y @ (0.30,0,0.65) J6 x @ (0.34,0,0.65)
    finger-centre frame M: (0.405, 0, 0.65), tool rotated -90 deg about y.
Numeric IK: damped least squares on position + orientation of the finger-centre frame.
"""
import math

import numpy as np

# ---------------------------------------------------------------- 6-axis arm
AXES = [
    ((0, 0, 1), (0, 0, 0)),
    ((0, 1, 0), (0, 0, 0.30)),
    ((0, 1, 0), (0, 0, 0.65)),
    ((1, 0, 0), (0.15, 0, 0.65)),
    ((0, 1, 0), (0.30, 0, 0.65)),
    ((1, 0, 0), (0.34, 0, 0.65)),
]
M = np.eye(4)
M[:3, :3] = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]], float)
M[:3, 3] = (0.405, 0, 0.65)


def rot(w, th):
    w = np.asarray(w, float)
    K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def _exp(w, p, th):
    E = np.eye(4)
    R = rot(w, th)
    E[:3, :3] = R
    E[:3, 3] = (np.eye(3) - R) @ np.asarray(p, float)
    return E


def arm_fk(q):
    """4x4 pose of the finger-centre frame (robot base frame)."""
    T = np.eye(4)
    for (w, p), th in zip(AXES, q):
        T = T @ _exp(w, p, th)
    return T @ M


def arm_joint_points(q):
    """World points of the 6 joint axes + product of exponentials up to J6."""
    T = np.eye(4)
    pts = []
    for (w, p), th in zip(AXES, q):
        pts.append((T @ np.append(p, 1))[:3])
        T = T @ _exp(w, p, th)
    return pts, T


def log_rot(R):
    c = max(-1.0, min(1.0, (np.trace(R) - 1) / 2))
    th = math.acos(c)
    if th < 1e-9:
        return np.zeros(3)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v * th / (2 * math.sin(th))


def _err(q, p_t, R_t):
    T = arm_fk(q)
    return np.concatenate([p_t - T[:3, 3], log_rot(R_t @ T[:3, :3].T)])


def arm_ik(p_t, R_t, q0, iters=200, lam=0.02, tol=1e-4):
    q = np.array(q0, float)
    for _ in range(iters):
        e = _err(q, p_t, R_t)
        if np.linalg.norm(e) < 1e-7:
            break
        J = np.zeros((6, 6))
        for i in range(6):
            dq = np.zeros(6)
            dq[i] = 1e-6
            J[:, i] = (_err(q, p_t, R_t) - _err(q + dq, p_t, R_t)) / 1e-6
        q += J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), e)
    e = float(np.linalg.norm(_err(q, p_t, R_t)))
    if e > tol:
        raise RuntimeError(f"arm IK did not converge for {p_t} (error {e:.2e})")
    return q


def arm_spheres(q, holding=True, fingers=True):
    """Collision spheres (robot base frame): upper arm, forearm, wrist, gripper, carried cube."""
    pts, T = arm_joint_points(q)
    Tt = T @ M
    c, Rt = Tt[:3, 3], Tt[:3, :3]
    tool_o = c + Rt @ np.array([0, 0, 0.035])
    out = []

    def seg(a, b, r, n):
        for k in range(n + 1):
            out.append((a + (b - a) * k / n, r))

    seg(pts[1], pts[2], 0.06, 7)
    seg(pts[2], pts[4], 0.05, 6)
    seg(pts[4], tool_o, 0.045, 3)
    for dx in (-0.04, 0.0, 0.04):
        out.append((tool_o + Rt @ np.array([dx, 0, 0]), 0.025))
    if fingers:
        for sx in (-1, 1):
            for dz in (-0.015, -0.035, -0.055):
                out.append((tool_o + Rt @ np.array([sx * 0.03, 0, dz]), 0.013))
    if holding:
        out.append((c, 0.035))
    return out


# ---------------------------------------------------------------- SCARA
L1, L2 = 0.40, 0.30


def scara_ik(x, y, d3, base=(0.0, 0.0)):
    x, y = x - base[0], y - base[1]
    c2 = max(-1.0, min(1.0, (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2)))
    q2 = math.acos(c2)
    q1 = math.atan2(y, x) - math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
    return [q1, q2, d3, -(q1 + q2)]


def scara_fk(q, base=(0.0, 0.0, 0.0)):
    """Finger-centre position of the SCARA (world)."""
    q1, q2, d3, _ = q
    x = base[0] + L1 * math.cos(q1) + L2 * math.cos(q1 + q2)
    y = base[1] + L1 * math.sin(q1) + L2 * math.sin(q1 + q2)
    return np.array([x, y, base[2] + 0.175 - d3])


def scara_spheres(q, base=(0.0, 0.0, 0.0), fingers=True):
    q1, q2, d3, _ = q
    bx, by, zb = base
    E = np.array([bx + L1 * math.cos(q1), by + L1 * math.sin(q1)])
    Q = E + L2 * np.array([math.cos(q1 + q2), math.sin(q1 + q2)])
    B = np.array([bx, by])
    out = []
    for k in range(7):
        p = B + (E - B) * k / 6
        out.append((np.array([p[0], p[1], zb + 0.43]), 0.07))
    for k in range(6):
        p = E + (Q - E) * k / 5
        out.append((np.array([p[0], p[1], zb + 0.485]), 0.06))
    for z in np.linspace(zb + 0.59 - d3, zb + 0.23 - d3, 8):
        out.append((np.array([Q[0], Q[1], z]), 0.02))
    tz = zb + 0.21 - d3
    for dx in (-0.04, 0.0, 0.04):
        out.append((np.array([Q[0] + dx, Q[1], tz]), 0.025))
    if fingers:
        for sx in (-1, 1):
            for dz in (-0.015, -0.035, -0.055):
                out.append((np.array([Q[0] + sx * 0.03, Q[1], tz + dz]), 0.013))
    return out


# ---------------------------------------------------------------- helpers
def interp(keys, t):
    """Smooth-step interpolation of a key list [(t, v1, v2, ...), ...]."""
    for a, b in zip(keys[:-1], keys[1:]):
        if a[0] <= t <= b[0]:
            u = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 1.0
            s = u * u * (3 - 2 * u)
            return [a[i] + (b[i] - a[i]) * s for i in range(1, len(a))]
    return list(keys[-1][1:])


def box_dist(p, c, s):
    c, s = np.array(c), np.array(s)
    return float(np.linalg.norm(np.maximum(np.abs(p - c) - s / 2, 0)))


def ry(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def motion_stamp(t):
    ms = int(round(t * 1000))
    return f"{ms // 60000:02d}:{(ms // 1000) % 60:02d}:{ms % 1000:03d}"
