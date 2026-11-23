"""Headless check of the pick & place examples (added to a copy of the world by tools/run_examples.sh).

    controllerArgs: <duration_s> <report.json>
Passes when, during the run, the cube was placed on mark B (+-5 mm), then back on mark A (+-5 mm),
never fell off the table and nothing touched the obstacles (PILLAR, LOW_WALL) if present.
"""
import json
import math
import sys

from controller import Supervisor

sup = Supervisor()
dt = int(sup.getBasicTimeStep())
duration, report = float(sys.argv[1]), sys.argv[2]
cube = sup.getFromDef("CUBE")
A, B = (0.35, 0.15), (0.55, -0.15)
obstacles = [n for n in (sup.getFromDef("PILLAR"), sup.getFromDef("LOW_WALL")) if n is not None]
for n in obstacles:
    n.enableContactPointsTracking(dt)
m = {"placed_on_B_mm": None, "back_on_A_mm": None, "lowest_cube_z": 1.0, "obstacle_contacts": 0}
seen_b = False
while sup.step(dt) != -1 and sup.getTime() < duration:
    p = cube.getPosition()
    v = math.sqrt(sum(x * x for x in cube.getVelocity()[:3]))
    m["lowest_cube_z"] = min(m["lowest_cube_z"], p[2])
    resting = abs(p[2] - 0.07) < 0.002 and v < 0.002
    if resting and math.dist(p[:2], B) < 0.02:
        seen_b = True
        d = 1000 * math.dist(p[:2], B)
        m["placed_on_B_mm"] = d if m["placed_on_B_mm"] is None else min(m["placed_on_B_mm"], d)
    if resting and seen_b and math.dist(p[:2], A) < 0.02:
        d = 1000 * math.dist(p[:2], A)
        m["back_on_A_mm"] = d if m["back_on_A_mm"] is None else min(m["back_on_A_mm"], d)
    for n in obstacles:
        m["obstacle_contacts"] += len(n.getContactPoints())
ok = (m["placed_on_B_mm"] is not None and m["placed_on_B_mm"] < 5 and m["back_on_A_mm"] is not None
      and m["back_on_A_mm"] < 5 and m["lowest_cube_z"] > 0.06 and m["obstacle_contacts"] == 0)
json.dump({"passed": ok, "metrics": m}, open(report, "w"), indent=2)
sup.simulationQuit(0 if ok else 1)
