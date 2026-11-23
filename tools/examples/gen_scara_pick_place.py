"""SCARA pick & place example: A -> B -> A loop (15.2 s) of a 50 g cube on a work table.

Same gripper strategy as the cell: the fingers close to 0.5 mm beyond contact, so the
two fingers stay symmetric and the cube remains centred (+-0.5 mm) in the gripper.
    python tools/examples/gen_scara_pick_place.py [out.motion]
"""
import math
import os
import sys

L1, L2 = 0.4, 0.3
A, B = (0.35, 0.15), (0.55, -0.15)
UP, DOWN, PLACE, OPEN, CLOSE = 0.0, 0.10, 0.098, 0.0, 0.0105   # J3 stroke, finger travel


def ik(x, y):
    c2 = max(-1, min(1, (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2)))
    q2 = math.acos(c2)
    q1 = math.atan2(y, x) - math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
    return q1, q2, -(q1 + q2)


def half(P, Q):
    return [((P[0], P[1], DOWN, OPEN), 1.0), ((P[0], P[1], DOWN, CLOSE), 0.6), ((P[0], P[1], DOWN, CLOSE), 0.2),
            ((P[0], P[1], UP, CLOSE), 0.8), ((Q[0], Q[1], UP, CLOSE), 2.2), ((Q[0], Q[1], PLACE, CLOSE), 1.0),
            ((Q[0], Q[1], PLACE, OPEN), 0.6), ((Q[0], Q[1], UP, OPEN), 0.8), ((Q[0], Q[1], UP, OPEN), 0.4)]


def main(out):
    state, t, dt, rows = (A[0], A[1], UP, OPEN), 0.0, 0.04, []

    def row(t, s):
        q1, q2, q4 = ik(s[0], s[1])
        ms = int(round(t * 1000))
        rows.append(f"{ms // 60000:02d}:{(ms // 1000) % 60:02d}:{ms % 1000:03d},Pose{len(rows)},"
                    f"{q1:.5f},{q2:.5f},{s[2]:.5f},{q4:.5f},{s[3]:.5f},{s[3]:.5f}")

    row(0, state)
    for tgt, dur in half(A, B) + half(B, A):
        n = max(1, int(round(dur / dt)))
        for k in range(1, n + 1):
            a = k / n
            s = a * a * (3 - 2 * a)
            row(t + dur * a, tuple(state[i] + (tgt[i] - state[i]) * s for i in range(4)))
        state, t = tgt, t + dur
    with open(out, "w") as f:
        f.write("#WEBOTS_MOTION,V1.0,joint1,joint2,joint3,joint4,finger_left,finger_right\n" + "\n".join(rows) + "\n")
    print(len(rows), "poses,", round(t, 1), "s ->", out)


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    main(sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "..", "..", "motions", "examples", "scara_pick_place.motion"))
