"""Generate the two synchronised robot cycles of the cell (Webots .motion files).

Cycle (14.4 s, started by the PLC when a cube sits at the end stop):
  FANUC : half-open gripper above the end stop -> grasp (top, pads on the +-x faces, upper 30 mm)
          -> presents the cube at the hand-over point H -> releases it once the UR5e holds it -> back.
  UR5e  : from under H, gripper pointing up, grasps the +-y faces (lower 30 mm) -> flips the cube
          180 deg about y in the air -> drops it into the output bin -> back.

Checks (non-zero exit if one fails): IK convergence, joint limits, joint speeds (< 80 % of the motor limits).
    python tools/gen_motions.py [motions_dir]
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from cell_layout import (CYCLE, DT, F_GRASP, F_HAND, F_HOVER, F_LEAVE, FANUC_BASE, FANUC_YAW, G_CLOSE,  # noqa: E402
                         G_HALF, G_OPEN, H, I, O, TCP, U_DROP, U_HAND, U_PRE, UR_BASE, UR_YAW)
from kinematics import interp, motion_stamp  # noqa: E402
from kinematics_lib import FANUC, UR5E  # noqa: E402

PI = math.pi
# gripper frame: z = approach, y = closing direction
R_F = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], float)        # FANUC: pointing down, closing along world x
R_U_DOWN = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]], float)  # UR5e: pointing down, closing along world y


def ry(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


F_KEYS = [  # (t, x, y, z_gripper_origin, fingers)
    (0.0, *I, F_HOVER, G_HALF), (1.0, *I, F_GRASP, G_HALF), (1.4, *I, F_GRASP, G_CLOSE), (1.8, *I, F_GRASP, G_CLOSE),
    (2.6, *I, F_HOVER, G_CLOSE), (4.6, *H, F_HAND, G_CLOSE), (6.8, *H, F_HAND, G_CLOSE), (7.2, *H, F_HAND, G_HALF),
    (8.0, *H, F_LEAVE, G_HALF), (10.0, *I, F_HOVER, G_HALF), (CYCLE, *I, F_HOVER, G_HALF),
]
U_KEYS = [  # (t, x, y, z_gripper_origin, theta, fingers)  theta: 0 = pointing down, pi = pointing up
    (0.0, *O, U_DROP, 0, G_OPEN), (1.0, *O, U_DROP, 0, G_OPEN), (4.6, *H, U_PRE, PI, G_OPEN),
    (5.2, *H, U_PRE, PI, G_OPEN), (6.0, *H, U_HAND, PI, G_OPEN), (6.4, *H, U_HAND, PI, G_CLOSE),
    (8.0, *H, U_HAND, PI, G_CLOSE), (8.8, *H, U_PRE, PI, G_CLOSE), (11.6, *O, U_DROP, 0, G_CLOSE),
    (12.2, *O, U_DROP, 0, G_OPEN), (CYCLE, *O, U_DROP, 0, G_OPEN),
]
F_SEED = [0.9, 0.3, -0.2, 0.0, -1.2, 0.0]
U_SEED = [0.963, -1.144, 1.402, 1.313, 1.571, 2.533]   # wrist_1 (J4) branch rotated by 180 deg
# motor speed limits of the PROTOs (rad/s); the cycles must stay below 80 % of them
VMAX = {"fanuc": [7.85, 6.63, 9.08, 9.60, 9.51, 17.45], "ur5e": [3.14, 3.14, 3.14, 6.28, 6.28, 6.28]}
GRIPPER = ["ROBOTIQ 2F-85 Gripper::left finger joint", "ROBOTIQ 2F-85 Gripper::right finger joint"]


def local(base, yaw, p, R):
    """World target -> robot base frame."""
    c, s = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return Rz.T @ (np.asarray(p) - np.asarray(base)), Rz.T @ R


def generate():
    n = int(round(CYCLE / DT))
    rows = {"fanuc": [], "ur5e": []}
    qf = qu = None
    traj = {"fanuc": [], "ur5e": []}
    for k in range(n + 1):
        t = k * DT
        fx, fy, fz, fg = interp(F_KEYS, t)
        p, R = local(FANUC_BASE, FANUC_YAW, (fx, fy, fz), R_F)
        qf = FANUC.ik(p, R, qf if qf is not None else F_SEED, tcp_offset=0.0)
        ux, uy, uz, uth, ug = interp(U_KEYS, t)
        p, R = local(UR_BASE, UR_YAW, (ux, uy, uz), ry(uth) @ R_U_DOWN)
        qu = UR5E.ik(p, R, qu if qu is not None else U_SEED, tcp_offset=0.0)
        for name, q, g in (("fanuc", qf, fg), ("ur5e", qu, ug)):
            rows[name].append(f"{motion_stamp(t)},Pose{k}," + ",".join(f"{v:.5f}" for v in q) + f",{g:.5f},{g:.5f}")
            traj[name].append(np.array(q))
    return rows, traj


def check(traj):
    ok = True
    for name, chain in (("fanuc", FANUC), ("ur5e", UR5E)):
        Q = np.array(traj[name])
        v = np.abs(np.diff(Q, axis=0)).max(axis=0) / DT
        lim = np.array(chain.limits)
        margin = np.minimum(Q.min(axis=0) - lim[:, 0], lim[:, 1] - Q.max(axis=0))
        print(f"{name:6s} max joint speed (rad/s): " + " ".join(f"{x:.2f}/{m:g}" for x, m in zip(v, VMAX[name])))
        print(f"{name:6s} min margin to joint limits (rad): " + " ".join(f"{x:.2f}" for x in margin))
        if np.any(margin < 0.02):
            ok = False
            print("  FAIL: joint limit")
        if np.any(v > 0.8 * np.array(VMAX[name])):
            ok = False
            print("  FAIL: joint speed above 80 % of the motor limit")
    return ok


def main(outdir):
    os.makedirs(outdir, exist_ok=True)
    rows, traj = generate()
    for name, chain in (("fanuc", FANUC), ("ur5e", UR5E)):
        with open(os.path.join(outdir, f"{name}_cell.motion"), "w") as f:
            f.write("#WEBOTS_MOTION,V1.0," + ",".join(chain.names + GRIPPER) + "\n" + "\n".join(rows[name]) + "\n")
    print(f"{len(rows['fanuc'])} poses per robot, cycle {CYCLE} s -> {outdir}")
    ok = check(traj)
    print("OK" if ok else "CHECKS FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "..", "motions")))
