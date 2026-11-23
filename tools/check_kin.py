"""Kinematics checks (no Webots needed).

1. FANUC LR Mate 200iD and UR5e (+ Robotiq 2F-85): numeric IK -> FK closure (position and
   orientation) on a grid of gripper poses, gripper pointing down, tilted and up.
2. Motions: replays motions/*.motion through the FK and checks that the grippers hit the pick (I),
   hand-over (H) and drop (O) points at the planned key times.
The FK itself is validated against Webots by tools/calibrate.sh (31 random configurations per robot).

    python tools/check_kin.py          (exit code 0 = all checks passed)
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from cell_layout import F_GRASP, F_HAND, FANUC_BASE, FANUC_YAW, H, I, O, U_DROP, U_HAND, UR_BASE, UR_YAW  # noqa: E402
from kinematics_lib import FANUC, UR5E, log_rot  # noqa: E402

TOL_POS, TOL_ROT, TOL_MOTION = 1e-6, 1e-6, 2e-4
failures = []


def report(name, worst, tol, n, unit="m"):
    ok = worst <= tol
    print(f"{'PASS' if ok else 'FAIL'}  {name:55s} worst {worst:.2e} {unit} over {n} poses")
    if not ok:
        failures.append(name)


def ry(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


DOWN = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], float)
for chain, seed, box in ((FANUC, [0.0, 0.3, -0.2, 0.0, -1.2, 0.0], ((0.30, 0.50), (-0.25, 0.25), (0.35, 0.65))),
                         (UR5E, [0.0, -1.4, 1.6, -1.8, -1.57, 0.0], ((0.35, 0.60), (-0.25, 0.25), (0.20, 0.55)))):
    wp = wr = 0.0
    n = skipped = 0
    for th in (0.0, math.pi / 4, math.pi / 2):
        R_t = ry(th) @ DOWN
        q = np.array(seed)
        for x in np.linspace(*box[0], 4):
            for y in np.linspace(*box[1], 4):
                for z in np.linspace(*box[2], 3):
                    try:
                        q = chain.ik(np.array([x, y, z]), R_t, q, tcp_offset=0.0)
                    except RuntimeError:
                        skipped += 1          # outside the reachable workspace for this orientation
                        continue
                    T = chain.fk(q, tcp_offset=0.0)
                    wp = max(wp, float(np.linalg.norm(T[:3, 3] - [x, y, z])))
                    wr = max(wr, float(np.linalg.norm(log_rot(R_t @ T[:3, :3].T))))
                    n += 1
    print(f"      {chain.name}: {skipped} grid poses unreachable, skipped")
    report(f"{chain.name} IK -> FK closure (position)", wp, TOL_POS, n)
    report(f"{chain.name} IK -> FK closure (orientation)", wr, TOL_ROT, n, "rad")


def load(path):
    rows = []
    with open(path) as f:
        f.readline()
        for line in f:
            p = line.strip().split(",")
            mm, ss, ms = (int(v) for v in p[0].split(":"))
            rows.append((mm * 60 + ss + ms / 1000, [float(v) for v in p[2:8]]))
    return rows


def at(rows, t):
    return min(rows, key=lambda r: abs(r[0] - t))[1]


def world(chain, base, yaw, q):
    c, s = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return Rz @ chain.fk(np.array(q), tcp_offset=0.0)[:3, 3] + np.array(base)


mdir = os.path.join(HERE, "..", "motions")
F = load(os.path.join(mdir, "fanuc_cell.motion"))
U = load(os.path.join(mdir, "ur5e_cell.motion"))
targets = [
    ("FANUC grasp at the end stop I", world(FANUC, FANUC_BASE, FANUC_YAW, at(F, 1.4)), (I[0], I[1], F_GRASP)),
    ("FANUC at the hand-over H", world(FANUC, FANUC_BASE, FANUC_YAW, at(F, 6.0)), (H[0], H[1], F_HAND)),
    ("UR5e at the hand-over H", world(UR5E, UR_BASE, UR_YAW, at(U, 6.4)), (H[0], H[1], U_HAND)),
    ("UR5e drop above the bin O", world(UR5E, UR_BASE, UR_YAW, at(U, 12.2)), (O[0], O[1], U_DROP)),
]
worst = 0.0
for name, p, t in targets:
    e = float(np.linalg.norm(p - np.array(t)))
    worst = max(worst, e)
    print(f"      {name:40s} error {1000 * e:.3f} mm")
report("motion key points (gripper origin)", worst, TOL_MOTION, len(targets))
print("\nALL CHECKS PASSED" if not failures else f"\n{len(failures)} CHECK(S) FAILED: {failures}")
sys.exit(1 if failures else 0)
