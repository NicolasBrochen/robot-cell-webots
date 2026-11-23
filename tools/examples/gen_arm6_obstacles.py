"""Pick & place du bras 6 axes avec évitement d'obstacles.

- Collision : le bras entier (bras, avant-bras, poignet, pince, doigts) et le cube
  transporté sont modélisés par des sphères ; obstacles = boîtes (+ marge).
- Planification : A* sur une grille XY (1 cm) à hauteur de transit, chaque nœud
  validé par cinématique inverse + test de collision du bras complet ;
  puis raccourcis (shortcut) et lissage des coins (Chaikin), revalidés.
Usage : python gen_motion_obstacles.py <sortie.motion>
"""
import heapq
import math
import sys

import numpy as np

sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
from gen_arm6_pick_place import AXES, A, B, CLOSE, OPEN, R_DOWN, Z_GRASP, Z_PLACE, Z_UP, ik, rot

# obstacles : (centre, taille) — identiques à la scène Webots
OBSTACLES = [
    ((0.47, 0.0, 0.2), (0.06, 0.06, 0.3)),     # pilier
    ((0.51, -0.07, 0.085), (0.1, 0.03, 0.07)),  # muret
]
MARGIN = 0.015
GRID = 0.01
TRANSIT_TIME = 3.2


def joint_points(q):
    """Points des axes (monde) + repère outil, pour la configuration q."""
    T = np.eye(4)
    pts = []
    for (w, p), th in zip(AXES, q):
        pts.append((T @ np.append(p, 1))[:3])
        R = rot(w, th)
        E = np.eye(4)
        E[:3, :3] = R
        E[:3, 3] = (np.eye(3) - R) @ np.asarray(p, float)
        T = T @ E
    return pts, T


def spheres(q, holding=True):
    from gen_arm6_pick_place import M
    pts, T = joint_points(q)
    Tt = T @ M                       # repère « centre des doigts »
    c, Rt = Tt[:3, 3], Tt[:3, :3]
    tool_o = c + Rt @ np.array([0, 0, 0.035])
    out = []

    def seg(a, b, r, n):
        for k in range(n + 1):
            out.append((a + (b - a) * k / n, r))

    seg(pts[1], pts[2], 0.06, 7)       # bras
    seg(pts[2], pts[4], 0.05, 6)       # avant-bras
    seg(pts[4], tool_o, 0.045, 3)      # poignet -> outil
    for dx in (-0.04, 0.0, 0.04):      # corps de pince
        out.append((tool_o + Rt @ np.array([dx, 0, 0]), 0.025))
    for sx in (-1, 1):                 # doigts
        for dz in (-0.015, -0.035, -0.055):
            out.append((tool_o + Rt @ np.array([sx * 0.03, 0, dz]), 0.013))
    if holding:
        out.append((c, 0.035))         # cube transporté
    return out


def collides(q, holding=True):
    for p, r in spheres(q, holding):
        for (cx, cy, cz), (sx, sy, sz) in OBSTACLES:
            lo = np.array([cx - sx / 2, cy - sy / 2, cz - sz / 2])
            hi = np.array([cx + sx / 2, cy + sy / 2, cz + sz / 2])
            if np.linalg.norm(p - np.clip(p, lo, hi)) < r + MARGIN:
                return True
    return False


def clearance(q, holding=True):
    best = 1e9
    for p, r in spheres(q, holding):
        for (cx, cy, cz), (sx, sy, sz) in OBSTACLES:
            lo = np.array([cx - sx / 2, cy - sy / 2, cz - sz / 2])
            hi = np.array([cx + sx / 2, cy + sy / 2, cz + sz / 2])
            best = min(best, np.linalg.norm(p - np.clip(p, lo, hi)) - r)
    return best


def seed_for(x, y):
    a = math.atan2(y, x)
    return [a, 0.8, 0.3, 0.0, 0.46, a]


_cache = {}


def config_at(x, y, z=Z_UP):
    key = (round(x, 4), round(y, 4), round(z, 4))
    if key not in _cache:
        try:
            q = ik(np.array([x, y, z]), R_DOWN, seed_for(x, y))
            _cache[key] = None if collides(q) else q
        except RuntimeError:
            _cache[key] = None
    return _cache[key]


def astar(start, goal):
    to_cell = lambda p: (round(p[0] / GRID), round(p[1] / GRID))
    s, g = to_cell(start), to_cell(goal)
    h = lambda c: math.hypot(c[0] - g[0], c[1] - g[1])
    openq = [(h(s), 0.0, s)]
    came, cost = {s: None}, {s: 0.0}
    nbrs = [(dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy]
    while openq:
        _, gc, c = heapq.heappop(openq)
        if c == g:
            path = []
            while c is not None:
                path.append((c[0] * GRID, c[1] * GRID))
                c = came[c]
            return path[::-1]
        if gc > cost[c]:
            continue
        for dx, dy in nbrs:
            n = (c[0] + dx, c[1] + dy)
            if config_at(n[0] * GRID, n[1] * GRID) is None:
                continue
            nc = gc + math.hypot(dx, dy)
            if nc < cost.get(n, 1e9):
                cost[n], came[n] = nc, c
                heapq.heappush(openq, (nc + h(n), nc, n))
    raise RuntimeError("aucun chemin sans collision")


def segment_ok(a, b, step=0.005):
    n = max(1, int(math.dist(a, b) / step))
    return all(config_at(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) is not None
               for k in range(n + 1))


def shortcut(path):
    out, i = [path[0]], 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not segment_ok(path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def chaikin(path, it=3):
    for _ in range(it):
        new = [path[0]]
        for a, b in zip(path[:-1], path[1:]):
            new += [(0.75 * a[0] + 0.25 * b[0], 0.75 * a[1] + 0.25 * b[1]),
                    (0.25 * a[0] + 0.75 * b[0], 0.25 * a[1] + 0.75 * b[1])]
        new.append(path[-1])
        path = new
    return path


def resample(path, n):
    d = [0.0]
    for a, b in zip(path[:-1], path[1:]):
        d.append(d[-1] + math.dist(a, b))
    L, out, j = d[-1], [], 0
    for k in range(n + 1):
        a = k / n
        s = L * a * a * (3 - 2 * a)            # profil de vitesse doux
        while j < len(d) - 2 and d[j + 1] < s:
            j += 1
        u = (s - d[j]) / max(1e-12, d[j + 1] - d[j])
        out.append((path[j][0] + (path[j + 1][0] - path[j][0]) * u,
                    path[j][1] + (path[j + 1][1] - path[j][1]) * u))
    return out, L


def plan(P, Q, plan_margin=0.035):
    global MARGIN
    final = MARGIN
    MARGIN = plan_margin            # marge élargie pour laisser la place au lissage
    _cache.clear()
    raw = astar(P, Q)
    sc = shortcut(raw)
    MARGIN = final                  # validation avec la marge réelle
    _cache.clear()
    for it in (3, 2, 1):
        sm = chaikin(sc, it)
        if all(segment_ok(a, b) for a, b in zip(sm[:-1], sm[1:])):
            print(f"lissage Chaikin : {it} itération(s)")
            return sm
    print("lissage refusé (trop proche des obstacles) : polyligne conservée")
    return sc


def main(out):
    path_ab = plan(A, B)
    dt = 0.04
    rows = []
    q = config_at(*A)

    def row(t, q, f):
        ms = int(round(t * 1000))
        rows.append(f"{ms // 60000:02d}:{(ms // 1000) % 60:02d}:{ms % 1000:03d},Pose{len(rows)},"
                    + ",".join(f"{v:.5f}" for v in q) + f",{f:.5f},{f:.5f}")

    def vertical(P, z0, z1, f0, f1, dur, t, q):
        n = max(1, int(round(dur / dt)))
        for k in range(1, n + 1):
            a = k / n
            s = a * a * (3 - 2 * a)
            q = ik(np.array([P[0], P[1], z0 + (z1 - z0) * s]), R_DOWN, q)
            row(t + dur * a, q, f0 + (f1 - f0) * s)
        return t + dur, q

    def transit(path, t, q, f):
        n = int(round(TRANSIT_TIME / dt))
        pts, _ = resample(path, n)
        for k, (x, y) in enumerate(pts[1:], 1):
            q = ik(np.array([x, y, Z_UP]), R_DOWN, q)
            row(t + TRANSIT_TIME * k / n, q, f)
        return t + TRANSIT_TIME, q

    t = 0.0
    row(0, q, OPEN)
    min_clear = 1e9
    for P, Q, path in ((A, B, path_ab), (B, A, path_ab[::-1])):
        t, q = vertical(P, Z_UP, Z_GRASP, OPEN, OPEN, 1.0, t, q)
        t, q = vertical(P, Z_GRASP, Z_GRASP, OPEN, CLOSE, 0.6, t, q)
        t, q = vertical(P, Z_GRASP, Z_GRASP, CLOSE, CLOSE, 0.2, t, q)
        t, q = vertical(P, Z_GRASP, Z_UP, CLOSE, CLOSE, 0.8, t, q)
        t, q = transit(path, t, q, CLOSE)
        t, q = vertical(Q, Z_UP, Z_PLACE, CLOSE, CLOSE, 1.0, t, q)
        t, q = vertical(Q, Z_PLACE, Z_PLACE, CLOSE, OPEN, 0.6, t, q)
        t, q = vertical(Q, Z_PLACE, Z_UP, OPEN, OPEN, 0.8, t, q)
        t, q = vertical(Q, Z_UP, Z_UP, OPEN, OPEN, 0.4, t, q)

    # contrôle : dégagement minimal sur toutes les poses du transit
    for x, y in resample(path_ab, 200)[0]:
        min_clear = min(min_clear, clearance(config_at(x, y)))
    header = "#WEBOTS_MOTION,V1.0," + ",".join(f"joint{i}" for i in range(1, 7)) + ",finger_left,finger_right"
    with open(out, "w") as f:
        f.write(header + "\n" + "\n".join(rows) + "\n")
    L = resample(path_ab, 10)[1]
    print(f"{len(rows)} poses, durée {t:.1f} s")
    print(f"chemin A->B : {len(path_ab)} points, longueur {L:.3f} m (ligne droite {math.dist(A, B):.3f} m)")
    print(f"dégagement minimal bras+cube / obstacles : {1000 * min_clear:.1f} mm (marge imposée {1000 * MARGIN:.0f} mm)")
    print("waypoints (raccourcis) :", [(round(x, 3), round(y, 3)) for x, y in shortcut(astar(A, B))])
    with open(out.replace(".motion", "_path.txt"), "w") as f:
        f.write("\n".join(f"{x:.4f} {y:.4f} {Z_UP:.4f}" for x, y in path_ab))


if __name__ == "__main__":
    main(sys.argv[1])
