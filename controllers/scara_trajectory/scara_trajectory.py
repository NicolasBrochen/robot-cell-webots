"""Contrôleur SCARA : suit une trajectoire en 8 (3D) par cinématique inverse
et trace en temps réel la trajectoire réelle de l'outil (ligne rouge).
La trajectoire planifiée est affichée en vert."""
import math
from controller import Supervisor

L1, L2 = 0.4, 0.3          # longueurs des bras (m)
Z_TIP0 = 0.16              # hauteur de la pointe de l'outil quand l'axe 3 = 0
CENTER = (0.45, 0.0)       # centre de la trajectoire (au-dessus du plan de travail)
PERIOD = 8.0               # s par tour de 8
MAX_TRAIL_POINTS = 5000

robot = Supervisor()
dt = int(robot.getBasicTimeStep())

motors = [robot.getDevice(f"joint{i}") for i in range(1, 5)]
sensors = [robot.getDevice(f"joint{i}_sensor") for i in range(1, 5)]
for s in sensors:
    s.enable(dt)


def target(t):
    """Point cible (x, y, z) de la pointe de l'outil à l'instant t."""
    w = 2 * math.pi / PERIOD
    x = CENTER[0] + 0.15 * math.sin(w * t)
    y = CENTER[1] + 0.12 * math.sin(2 * w * t)
    z = 0.10 + 0.035 * math.sin(3 * w * t)
    return x, y, z


def ik(x, y, z):
    r2 = x * x + y * y
    c2 = (r2 - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    c2 = max(-1.0, min(1.0, c2))
    q2 = math.acos(c2)  # configuration « coude à gauche »
    q1 = math.atan2(y, x) - math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
    d3 = Z_TIP0 - z
    q4 = -(q1 + q2)     # outil à orientation constante dans le repère monde
    return q1, q2, d3, q4


def fk(q1, q2, d3):
    x = L1 * math.cos(q1) + L2 * math.cos(q1 + q2)
    y = L1 * math.sin(q1) + L2 * math.sin(q1 + q2)
    return x, y, Z_TIP0 - d3


# --- Trajectoire planifiée (verte) ---
planned = robot.getFromDef("PLANNED_PATH").getField("geometry").getSFNode()
p_pts = planned.getField("coord").getSFNode().getField("point")
p_idx = planned.getField("coordIndex")
n = 200
for i in range(n + 1):
    p = list(target(PERIOD * i / n))
    if i == 0:
        p_pts.setMFVec3f(0, p)
    else:
        p_pts.insertMFVec3f(-1, p)
        p_idx.insertMFInt32(-1, i)

# --- Trace réelle (rouge) ---
trail = robot.getFromDef("TRAIL").getField("geometry").getSFNode()
t_pts = trail.getField("coord").getSFNode().getField("point")
t_idx = trail.getField("coordIndex")
tool = robot.getFromDef("TOOL")  # recherche dans tout l'arbre de scène
n_trail = 0


def tool_tip():
    if tool is not None:
        px, py, pz = tool.getPosition()
        return [px, py, pz - 0.05]
    return list(fk(*(s.getValue() for s in sensors[:3])))


# Phase 1 : rejoindre doucement le point de départ
start = ik(*target(0.0))
APPROACH = 2.0
t0 = robot.getTime()
while robot.step(dt) != -1:
    a = min(1.0, (robot.getTime() - t0) / APPROACH)
    s = 0.5 - 0.5 * math.cos(math.pi * a)
    for m, q in zip(motors, start):
        m.setPosition(s * q)
    if a >= 1.0:
        break

# Phase 2 : suivi de trajectoire + traçage
t_start = robot.getTime()
step = 0
while robot.step(dt) != -1:
    t = robot.getTime() - t_start
    for m, q in zip(motors, ik(*target(t))):
        m.setPosition(q)

    step += 1
    if step % 2 == 0 and n_trail < MAX_TRAIL_POINTS:
        p = tool_tip()
        if n_trail == 0:
            t_pts.setMFVec3f(0, p)
        else:
            t_pts.insertMFVec3f(-1, p)
            t_idx.insertMFInt32(-1, n_trail)
        n_trail += 1
