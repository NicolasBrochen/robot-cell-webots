"""Test supervisor: runs one tests/*.test.json scenario on the cell and quits Webots
with exit status 0 (all checks passed) or 1. Added to a copy of the world by tools/run_tests.sh.

    controllerArgs: <spec.json> <report.json>

Spec
  {"name": ..., "duration": s,
   "actions": [{"at": s, "do": "starve"}],          # starve: from then on, every cube waiting on the belt
                                                    # or in reserve is taken out of the cell (infeed dry)
   "checks":  [{"metric": name, "min": v} | {"metric": name, "max": v}, ...]}

Metrics
  delivered              cubes dropped into the output bin
  starts                 START orders sent by the PLC (listened on channel 1)
  starts_without_cube    START orders sent while no cube was resting against the end stop
  starts_after_starve    START orders after the last "starve" action + 1 cycle
  cubes_on_floor         cube samples lying on the floor outside the bin (should be 0)
  robot_contacts         contacts between a robot and anything but a cube
  handover_offset_mm     max horizontal offset of the cube from either finger centre while both grippers hold it
  arm6_cube_slip_mm      max |cube - finger centre| along the 6-axis gripper axis while the 6-axis holds it alone
                         and the gripper does not point down (cube weight towards or across the palm)
  arm6_palm_gap_min_mm   min gap between the cube face and the 6-axis palm (negative = cube inside the palm)
"""
import json
import math
import sys

from controller import Supervisor

sup = Supervisor()
dt = int(sup.getBasicTimeStep())
spec = json.load(open(sys.argv[1]))
report_path = sys.argv[2]

rx = sup.getDevice("receiver")
rx.enable(dt)

cubes = []
while sup.getFromDef(f"CUBE_{len(cubes)}") is not None:
    cubes.append(sup.getFromDef(f"CUBE_{len(cubes)}"))
robots = {name: sup.getFromDef(d) for name, d in (("scara", "SCARA"), ("arm6", "ARM6"))}
tools = {name: r.getFromProtoDef("TOOL") for name, r in robots.items()}
fingers = {name: (r.getFromProtoDef("FINGER_L"), r.getFromProtoDef("FINGER_R")) for name, r in robots.items()}
for r in robots.values():
    r.enableContactPointsTracking(dt, True)
stop = sup.getFromDef("CONVEYOR").getPosition()
bin_pos = sup.getFromDef("OUTPUT_BIN").getPosition()

m = {"delivered": 0, "starts": 0, "starts_without_cube": 0, "starts_after_starve": 0, "cubes_on_floor": 0,
     "robot_contacts": 0, "handover_offset_mm": 0.0, "arm6_cube_slip_mm": 0.0, "arm6_palm_gap_min_mm": 99.0}
in_bin, starve_t, actions = set(), None, sorted(spec.get("actions", []), key=lambda a: a["at"])
events = []
worst_slip = {}


def local(tool, p):
    """p expressed in the frame of the gripper body (TOOL)."""
    o, R = tool.getPosition(), tool.getOrientation()
    d = [p[i] - o[i] for i in range(3)]
    return [sum(R[3 * j + i] * d[j] for j in range(3)) for i in range(3)]


def gap(name):
    t = tools[name]
    return local(t, fingers[name][1].getPosition())[0] - local(t, fingers[name][0].getPosition())[0]


def cube_at_stop():
    return any(math.dist(c.getPosition(), (stop[0], stop[1], 0.37)) < 0.010 for c in cubes)   # open-gripper capture range


GRIP_CLOSED = 0.0503     # finger gap when closed on the 40 mm cube = 50.0 mm (+0.3 mm margin)
k = 0
while sup.step(dt) != -1:
    t = sup.getTime()
    k += 1
    while actions and t >= actions[0]["at"]:
        a = actions.pop(0)
        if a["do"] == "starve":
            starve_t = t
            events.append((round(t, 2), "starve"))
    if starve_t is not None:          # infeed dry: any cube back in the reserve or on the belt leaves the cell
        for i, c in enumerate(cubes):
            p = c.getPosition()
            if p[0] > 5 or (abs(p[0] - stop[0]) < 0.05 and p[1] > stop[1] + 0.02 and p[2] > 0.35):
                c.getField("translation").setSFVec3f([-2.0, -1.0 + 0.1 * i, 0.0205])
                c.resetPhysics()
    while rx.getQueueLength() > 0:
        msg = rx.getString()
        rx.nextPacket()
        if msg == "START":
            m["starts"] += 1
            if not cube_at_stop():
                m["starts_without_cube"] += 1
            if starve_t is not None and t > starve_t + 15.0:
                m["starts_after_starve"] += 1
            near = min(cubes, key=lambda c: math.dist(c.getPosition(), (stop[0], stop[1], 0.37)))
            events.append((round(t, 2), "START", [round(v, 4) for v in near.getPosition()]))
    if k % 2:
        continue
    cps = [c.getPosition() for c in cubes]
    for i, p in enumerate(cps):
        inb = abs(p[0] - bin_pos[0]) < 0.125 and abs(p[1] - bin_pos[1]) < 0.125 and p[2] < 0.12
        if inb and i not in in_bin:
            in_bin.add(i)
            m["delivered"] += 1
            events.append((round(t, 2), f"delivered cube {i}"))
        elif not inb:
            in_bin.discard(i)
        if not inb and p[2] < 0.03 and -1.5 < p[0] < 5:     # off-cell parking (x = -2) and reserve (x = 10) excluded
            m["cubes_on_floor"] += 1
    for name, r in robots.items():
        for cp in r.getContactPoints(True):
            if min(math.dist(cp.getPoint(), p) for p in cps) > 0.035:
                m["robot_contacts"] += 1
    gs, ga = gap("scara"), gap("arm6")
    tr = spec.get("trace")
    if tr and tr[0] <= t <= tr[1] and k % 12 == 0:      # optional debug trace (every ~0.1 s)
        events.append((round(t, 2), "trace", {"scara_gap_mm": round(1000 * gs, 1),
                       "scara_fingers": [round(v, 3) for v in local(tools["scara"], tools["scara"].getPosition())],
                       "cubes": [[round(v, 4) for v in p] for p in cps],
                       "cube_in_scara": [[round(1000 * v, 1) for v in local(tools["scara"], p)] for p in cps]}))
    for p in cps:
        ls, la = local(tools["scara"], p), local(tools["arm6"], p)
        held_s = gs < GRIP_CLOSED and abs(ls[0]) < 0.03 and abs(ls[1]) < 0.03 and -0.07 < ls[2] < 0.0
        held_a = ga < GRIP_CLOSED and abs(la[0]) < 0.03 and abs(la[1]) < 0.03 and -0.07 < la[2] < 0.0
        if held_s and held_a:
            m["handover_offset_mm"] = max(m["handover_offset_mm"], 1000 * math.hypot(ls[0], ls[1]),
                                          1000 * math.hypot(la[0], la[1]))
        if held_a and not held_s and tools["arm6"].getOrientation()[8] < 0.5:   # gripper not pointing down
            slip = 1000 * abs(la[2] + 0.035)
            if slip > m["arm6_cube_slip_mm"]:
                worst_slip.update(t=round(t, 3), local_mm=[round(1000 * v, 1) for v in la], gap_mm=round(1000 * ga, 1))
            m["arm6_cube_slip_mm"] = max(m["arm6_cube_slip_mm"], slip)
            m["arm6_palm_gap_min_mm"] = min(m["arm6_palm_gap_min_mm"], 1000 * (-0.015 - (la[2] + 0.02)))
    if t >= spec["duration"]:
        break

results, ok = [], True
for c in spec["checks"]:
    v = m[c["metric"]]
    passed = ("min" not in c or v >= c["min"]) and ("max" not in c or v <= c["max"])
    ok &= passed
    results.append({**c, "value": round(v, 3), "passed": passed})
    bound = (f">= {c['min']}" if "min" in c else "") + (" and " if "min" in c and "max" in c else "") + \
            (f"<= {c['max']}" if "max" in c else "")
    print(f"  [{'PASS' if passed else 'FAIL'}] {c['metric']:22s} = {v:8.3f}   (expected {bound})", flush=True)
json.dump({"name": spec["name"], "passed": ok, "metrics": m, "checks": results, "events": events, "worst_slip": worst_slip,
           "final_cubes": [[round(v, 3) for v in c.getPosition()] for c in cubes]},
          open(report_path, "w"), indent=2)
print(f"[cell_test] {spec['name']}: {'PASSED' if ok else 'FAILED'}", flush=True)
sup.simulationQuit(0 if ok else 1)
