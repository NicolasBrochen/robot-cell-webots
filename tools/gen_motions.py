"""Generate the two synchronised robot cycles of the cell (Webots .motion files).

Cycle (14.4 s, started by the PLC when a cube sits at the end stop):
  SCARA : pick at the end stop I (top grasp, faces +-x, upper half of the cube)
          -> hand-over point H -> release -> back to I.
  6-axis: from under H, gripper pointing up, grasps faces +-y over the full cube height
          (palm just below the cube) -> flips the cube 180 deg about y in the air
          -> drops it into the output bin O -> back.

Checks (non-zero exit if one fails): IK convergence, structural clearance between the
two robots, clearance to the fixed obstacles, joint speeds within the motor limits.

    python tools/gen_motions.py [motions_dir]
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cell_layout import (ARM_BASE, CLOSE, CYCLE, DT, H, I, O, OPEN, R_HAND, R_HOVER,  # noqa: E402
                         R_HPRE, S_GRASP, S_HAND, S_UP, SCARA_BASE, STATIC)
from kinematics import (arm_ik, arm_spheres, box_dist, interp, motion_stamp, ry,  # noqa: E402
                        scara_ik, scara_spheres)

PI = math.pi
S_KEYS = [  # (t, x, y, d3, fingers)
    (0.0, *I, S_UP, OPEN), (1.0, *I, S_GRASP, OPEN), (1.6, *I, S_GRASP, CLOSE), (1.8, *I, S_GRASP, CLOSE),
    (2.6, *I, S_UP, CLOSE), (4.6, *H, S_UP, CLOSE), (5.2, *H, S_HAND, CLOSE), (6.8, *H, S_HAND, CLOSE),
    (7.4, *H, S_HAND, OPEN), (8.0, *H, S_UP, OPEN), (10.0, *I, S_UP, OPEN), (CYCLE, *I, S_UP, OPEN),
]
R_KEYS = [  # (t, x, y, z, theta, fingers)   theta: 0 = fingers down, pi = fingers up
    (0.0, *O, R_HOVER, 0, OPEN), (2.0, *O, R_HOVER, 0, OPEN), (4.6, *H, R_HPRE, PI, OPEN),
    (5.2, *H, R_HPRE, PI, OPEN), (6.0, *H, R_HAND, PI, OPEN), (6.6, *H, R_HAND, PI, CLOSE),
    (8.0, *H, R_HAND, PI, CLOSE), (8.8, *H, R_HPRE, PI, CLOSE), (11.6, *O, R_HOVER, 0, CLOSE),
    (12.2, *O, R_HOVER, 0, OPEN), (CYCLE, *O, R_HOVER, 0, OPEN),
]
SCARA_VMAX = [3.0, 4.0, 0.3, 5.0]
ARM_VMAX = [3.0, 3.0, 3.0, 8.0, 3.0, 8.0]
MIN_ROBOT_CLEARANCE = 0.010
MIN_STATIC_CLEARANCE = 0.020

RZ = np.diag([-1.0, -1.0, 1.0])                    # base yaw of the 6-axis arm (pi)
R_DOWN = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], float)
ARM_SEED = [0.9, 0.8, 0.3, 0.0, 0.46, 0.9]


def arm_target(x, y, z, th):
    p_local = RZ.T @ (np.array([x, y, z]) - np.array(ARM_BASE))
    R_local = RZ.T @ ry(th) @ R_DOWN
    return p_local, R_local


def generate():
    n = int(round(CYCLE / DT))
    s_rows, r_rows, qs_all, q6_all = [], [], [], []
    q6 = None
    for k in range(n + 1):
        t = k * DT
        sx, sy, sd3, sf = interp(S_KEYS, t)
        qs = scara_ik(sx, sy, sd3, base=SCARA_BASE[:2])
        rx, ry_, rz, rth, rf = interp(R_KEYS, t)
        q6 = arm_ik(*arm_target(rx, ry_, rz, rth), q6 if q6 is not None else ARM_SEED)
        s_rows.append(f"{motion_stamp(t)},Pose{k}," + ",".join(f"{v:.5f}" for v in qs) + f",{sf:.5f},{sf:.5f}")
        r_rows.append(f"{motion_stamp(t)},Pose{k}," + ",".join(f"{v:.5f}" for v in q6) + f",{rf:.5f},{rf:.5f}")
        qs_all.append((t, qs, rf))
        q6_all.append((t, q6.copy(), rf))
    return s_rows, r_rows, qs_all, q6_all


def check(qs_all, q6_all):
    ok = True
    min_rs, t_rs = 1e9, 0.0
    min_static = {k: (1e9, 0.0) for k in STATIC}
    for (t, qs, _), (_, q6, rf) in zip(qs_all, q6_all):
        world = lambda sph: [(RZ @ p + np.array(ARM_BASE), r) for p, r in sph]  # noqa: E731
        # robot <-> robot: structure only (fingers and the shared cube meet by design at the hand-over)
        for pa, ra in world(arm_spheres(q6, holding=False, fingers=False)):
            for pb, rb in scara_spheres(qs, base=SCARA_BASE, fingers=False):
                d = float(np.linalg.norm(pa - pb)) - ra - rb
                if d < min_rs:
                    min_rs, t_rs = d, t
        # 6-axis (+ carried cube) <-> fixed obstacles
        for pa, ra in world(arm_spheres(q6, holding=rf > 0.005, fingers=False)):
            for name, (c, s) in STATIC.items():
                d = box_dist(pa, c, s) - ra
                if d < min_static[name][0]:
                    min_static[name] = (d, t)
    print(f"clearance SCARA <-> 6-axis (structure, fingers excluded): {1000 * min_rs:6.1f} mm at t={t_rs:.2f} s")
    if min_rs < MIN_ROBOT_CLEARANCE:
        ok = False
        print("  FAIL: below", 1000 * MIN_ROBOT_CLEARANCE, "mm")
    for name, (d, t) in min_static.items():
        print(f"clearance 6-axis <-> {name:15s}: {1000 * d:6.1f} mm at t={t:.2f} s")
        if d < MIN_STATIC_CLEARANCE:
            ok = False
            print("  FAIL: below", 1000 * MIN_STATIC_CLEARANCE, "mm")
    for label, rows, vmax in (("SCARA", [r[1] for r in qs_all], SCARA_VMAX), ("6-axis", [r[1] for r in q6_all], ARM_VMAX)):
        Q = np.array(rows)
        v = np.abs(np.diff(Q, axis=0)).max(axis=0) / DT
        print(f"max joint speed {label:6s}: " + " ".join(f"{a:.2f}/{b:g}" for a, b in zip(v, vmax)))
        if np.any(v > np.array(vmax)):
            ok = False
            print("  FAIL: joint speed above motor limit")
    return ok


def main(outdir):
    os.makedirs(outdir, exist_ok=True)
    s_rows, r_rows, qs_all, q6_all = generate()
    hdr = "#WEBOTS_MOTION,V1.0,"
    with open(os.path.join(outdir, "scara_cell.motion"), "w") as f:
        f.write(hdr + "joint1,joint2,joint3,joint4,finger_left,finger_right\n" + "\n".join(s_rows) + "\n")
    with open(os.path.join(outdir, "arm6_cell.motion"), "w") as f:
        f.write(hdr + ",".join(f"joint{i}" for i in range(1, 7)) + ",finger_left,finger_right\n"
                + "\n".join(r_rows) + "\n")
    print(f"{len(s_rows)} poses per robot, cycle {CYCLE} s -> {outdir}")
    ok = check(qs_all, q6_all)
    print("OK" if ok else "CHECKS FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else
                  os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "motions")))
