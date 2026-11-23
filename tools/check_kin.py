"""Kinematics checks (no Webots needed).

1. SCARA   : analytic IK -> FK closure on a grid covering the reachable annulus.
2. 6-axis  : numeric IK -> FK closure (position + orientation) on a grid of finger-centre
             targets, gripper pointing down, up and tilted.
3. Motions : replays motions/*.motion through the FK and checks that the finger centres hit
             the pick (I), hand-over (H) and drop (O) points at the planned key times.

    python tools/check_kin.py          (exit code 0 = all checks passed)
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from cell_layout import ARM_BASE, CUBE_AT_H, H, I, O, R_HAND, R_HOVER, S_GRASP, S_HAND, SCARA_BASE  # noqa: E402
from kinematics import L1, L2, arm_fk, arm_ik, log_rot, ry, scara_fk, scara_ik  # noqa: E402

TOL_POS = 1e-6      # m
TOL_ROT = 1e-6      # rad
TOL_MOTION = 2e-4   # m (motion files are written with 5 decimals)
failures = []


def report(name, worst, tol, n, unit="m"):
    ok = worst <= tol
    print(f"{'PASS' if ok else 'FAIL'}  {name:55s} worst {worst:.2e} {unit} over {n} poses")
    if not ok:
        failures.append(name)


# 1. SCARA closure
worst, n = 0.0, 0
for r in np.linspace(abs(L1 - L2) + 0.02, L1 + L2 - 0.02, 25):
    for a in np.linspace(-math.pi, math.pi, 37):
        for d3 in (0.0, 0.075, 0.15):
            x, y = r * math.cos(a), r * math.sin(a)
            q = scara_ik(x, y, d3)
            p = scara_fk(q)
            worst = max(worst, float(np.linalg.norm(p - [x, y, 0.175 - d3])))
            n += 1
report("SCARA IK -> FK closure", worst, TOL_POS, n)

# 2. 6-axis closure (robot base frame)
R_DOWN = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], float)
wp, wr, n, skipped = 0.0, 0.0, 0, 0
seed = [0.9, 0.8, 0.3, 0.0, 0.46, 0.9]
SHOULDER = np.array([0, 0, 0.30])
for th in (0.0, math.pi / 4, math.pi / 2, math.pi):
    R_t = ry(th) @ R_DOWN
    q = np.array(seed)
    for x in np.linspace(0.25, 0.55, 7):
        for y in np.linspace(-0.3, 0.3, 7):
            for z in (0.10, 0.25, 0.40):
                wrist = np.array([x, y, z]) + R_t[:, 2] * 0.105    # wrist centre, 105 mm behind the fingers
                if not 0.12 < np.linalg.norm(wrist - SHOULDER) < 0.64:
                    skipped += 1                                    # outside the reachable shell
                    continue
                try:
                    q = arm_ik(np.array([x, y, z]), R_t, q, tol=1e-6)
                except RuntimeError:
                    q = arm_ik(np.array([x, y, z]), R_t, seed, tol=1e-6)
                T = arm_fk(q)
                wp = max(wp, float(np.linalg.norm(T[:3, 3] - [x, y, z])))
                wr = max(wr, float(np.linalg.norm(log_rot(R_t @ T[:3, :3].T))))
                n += 1
print(f"      ({skipped} grid poses outside the reachable shell skipped)")
report("6-axis IK -> FK closure (position)", wp, TOL_POS, n)
report("6-axis IK -> FK closure (orientation)", wr, TOL_ROT, n, "rad")


# 3. motion replay
def load(path):
    rows = []
    with open(path) as f:
        f.readline()
        for line in f:
            p = line.strip().split(",")
            mm, ss, ms = (int(v) for v in p[0].split(":"))
            rows.append((mm * 60 + ss + ms / 1000, [float(v) for v in p[2:]]))
    return rows


def at(rows, t):
    return min(rows, key=lambda r: abs(r[0] - t))[1]


mdir = os.path.join(HERE, "..", "motions")
S = load(os.path.join(mdir, "scara_cell.motion"))
A = load(os.path.join(mdir, "arm6_cell.motion"))
RZ = np.diag([-1.0, -1.0, 1.0])


def arm_world(q):
    return RZ @ arm_fk(q)[:3, 3] + np.array(ARM_BASE)


targets = [
    ("SCARA grasp at end stop I", scara_fk(at(S, 1.6)[:4], SCARA_BASE), (I[0], I[1], 0.475 - S_GRASP)),
    ("SCARA at hand-over H", scara_fk(at(S, 6.0)[:4], SCARA_BASE), (H[0], H[1], 0.475 - S_HAND)),
    ("6-axis at hand-over H (full-height grasp)", arm_world(at(A, 6.6)[:6]), (H[0], H[1], R_HAND)),
    ("6-axis drop above output bin O", arm_world(at(A, 12.2)[:6]), (O[0], O[1], R_HOVER)),
]
worst = 0.0
for name, p, t in targets:
    e = float(np.linalg.norm(p - np.array(t)))
    worst = max(worst, e)
    print(f"      {name:45s} error {1000 * e:.3f} mm")
report("motion key points", worst, TOL_MOTION, len(targets))
gap = (0.475 - S_HAND) - R_HAND
print(f"      finger-centre offset SCARA - 6-axis at H: {1000 * gap:.1f} mm "
      f"(cube centre {1000 * CUBE_AT_H:.1f} mm, SCARA grips the upper half)")

print("\nALL CHECKS PASSED" if not failures else f"\n{len(failures)} CHECK(S) FAILED: {failures}")
sys.exit(1 if failures else 0)
