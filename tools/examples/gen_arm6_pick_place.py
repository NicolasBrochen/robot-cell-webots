"""Génère le mouvement pick & place du bras 6 axes (fichier .motion Webots).

Cinématique : produit d'exponentielles, configuration zéro = bras vertical,
avant-bras horizontal selon +x. Cinématique inverse numérique (moindres carrés
amortis) sur la position du centre des doigts + l'orientation de la pince
(doigts vers le bas, fermeture selon l'axe x du monde).
Usage : python gen_motion.py <sortie.motion>
"""
import math
import sys
import numpy as np

# axes (direction, point) en configuration zéro, repère monde
AXES = [
    ((0, 0, 1), (0, 0, 0)),        # J1 base (lacet)
    ((0, 1, 0), (0, 0, 0.30)),     # J2 épaule
    ((0, 1, 0), (0, 0, 0.65)),     # J3 coude
    ((1, 0, 0), (0.15, 0, 0.65)),  # J4 rotation avant-bras
    ((0, 1, 0), (0.30, 0, 0.65)),  # J5 tangage poignet
    ((1, 0, 0), (0.34, 0, 0.65)),  # J6 rotation bride
]
# repère « centre des doigts » en configuration zéro : outil tourné de -90° autour de y
R_TOOL0 = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]], float)
M = np.eye(4)
M[:3, :3] = R_TOOL0
M[:3, 3] = (0.405, 0, 0.65)


def rot(w, th):
    w = np.asarray(w, float)
    K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def fk(q):
    T = np.eye(4)
    for (w, p), th in zip(AXES, q):
        R = rot(w, th)
        E = np.eye(4)
        E[:3, :3] = R
        E[:3, 3] = (np.eye(3) - R) @ np.asarray(p, float)
        T = T @ E
    return T @ M


def log_rot(R):
    c = max(-1.0, min(1.0, (np.trace(R) - 1) / 2))
    th = math.acos(c)
    if th < 1e-9:
        return np.zeros(3)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v * th / (2 * math.sin(th))


def err(q, p_t, R_t):
    T = fk(q)
    return np.concatenate([p_t - T[:3, 3], log_rot(R_t @ T[:3, :3].T)])


def ik(p_t, R_t, q0, iters=200, lam=0.02):
    q = np.array(q0, float)
    for _ in range(iters):
        e = err(q, p_t, R_t)
        if np.linalg.norm(e) < 1e-7:
            break
        J = np.zeros((6, 6))
        for i in range(6):
            dq = np.zeros(6)
            dq[i] = 1e-6
            J[:, i] = (err(q, p_t, R_t) - err(q + dq, p_t, R_t)) / 1e-6
        q += J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(6), e)
    e = np.linalg.norm(err(q, p_t, R_t))
    if e > 1e-4:
        raise RuntimeError(f"IK non convergée pour {p_t} (erreur {e:.2e})")
    return q


A, B = (0.35, 0.15), (0.55, -0.15)
Z_UP, Z_GRASP, Z_PLACE = 0.175, 0.075, 0.077   # hauteur du centre des doigts
OPEN, CLOSE = 0.0, 0.0105                      # contact à 0.010 : 0.5 mm de serrage
R_DOWN = np.eye(3)                             # pince verticale, fermeture selon x


def half(P, Q):
    return [((P[0], P[1], Z_GRASP, OPEN), 1.0), ((P[0], P[1], Z_GRASP, CLOSE), 0.6),
            ((P[0], P[1], Z_GRASP, CLOSE), 0.2), ((P[0], P[1], Z_UP, CLOSE), 0.8),
            ((Q[0], Q[1], Z_UP, CLOSE), 2.2), ((Q[0], Q[1], Z_PLACE, CLOSE), 1.0),
            ((Q[0], Q[1], Z_PLACE, OPEN), 0.6), ((Q[0], Q[1], Z_UP, OPEN), 0.8),
            ((Q[0], Q[1], Z_UP, OPEN), 0.4)]


def main(out):
    seed = [math.atan2(A[1], A[0]), 0.3, 0.9, 0.0, math.pi / 2 - 1.2, 0.0]
    state = (A[0], A[1], Z_UP, OPEN)
    q = ik(np.array(state[:3]), R_DOWN, seed)
    rows, t, dt = [], 0.0, 0.04

    def row(t, q, f):
        ms = int(round(t * 1000))
        rows.append(f"{ms // 60000:02d}:{(ms // 1000) % 60:02d}:{ms % 1000:03d},Pose{len(rows)},"
                    + ",".join(f"{v:.5f}" for v in q) + f",{f:.5f},{f:.5f}")

    row(0, q, state[3])
    for tgt, dur in half(A, B) + half(B, A):
        n = max(1, int(round(dur / dt)))
        for k in range(1, n + 1):
            a = k / n
            s = a * a * (3 - 2 * a)
            cur = [state[i] + (tgt[i] - state[i]) * s for i in range(4)]
            q = ik(np.array(cur[:3]), R_DOWN, q)
            row(t + dur * a, q, cur[3])
        state, t = tgt, t + dur
    header = "#WEBOTS_MOTION,V1.0," + ",".join(f"joint{i}" for i in range(1, 7)) + ",finger_left,finger_right"
    with open(out, "w") as f:
        f.write(header + "\n" + "\n".join(rows) + "\n")
    qa = ik(np.array([A[0], A[1], Z_UP]), R_DOWN, seed)
    print(f"{len(rows)} poses, durée {t:.1f} s")
    print("pose de départ (au-dessus de A) :", [round(v, 5) for v in qa])


if __name__ == "__main__":
    main(sys.argv[1])
