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
  handover_offset_mm     max offset of the cube along the closing axis of a gripper while that gripper holds it
                         alone (so it is closed on the cube): the cube must be centred between the two fingers
  handover_width_offset_mm  same along the pad width (22 mm pads on a 40 mm cube: < 9 mm keeps the pad on the face)
  arm6_cube_slip_mm      max |cube - nominal TCP| along the UR5e gripper axis while the UR5e holds it alone
                         and its gripper does not point down (weight towards or across the palm)
  fanuc_cube_slip_mm     same for the FANUC gripper while the FANUC holds the cube alone
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
robots = {name: sup.getFromDef(d) for name, d in (("fanuc", "FANUC"), ("ur5e", "UR5E"))}
tools = {"fanuc": sup.getFromDef("FANUC_GRIP"), "ur5e": sup.getFromDef("UR_GRIP")}   # Robotiq nodes (toolSlots)
TCP = 0.15                         # nominal cube centre along the gripper z axis
for r in robots.values():
    r.enableContactPointsTracking(dt, True)
stop = sup.getFromDef("CONVEYOR").getPosition()
bin_pos = sup.getFromDef("OUTPUT_BIN").getPosition()

m = {"delivered": 0, "starts": 0, "starts_without_cube": 0, "starts_after_starve": 0, "cubes_on_floor": 0,
     "robot_contacts": 0, "handover_offset_mm": 0.0, "handover_width_offset_mm": 0.0, "arm6_cube_slip_mm": 0.0, "fanuc_cube_slip_mm": 0.0}
in_bin, starve_t, actions = set(), None, sorted(spec.get("actions", []), key=lambda a: a["at"])
events = []
worst_slip = {}
worst_handover = {}
last_double = {}
dropped = set()


def local(tool, p):
    """p expressed in the frame of the gripper body (TOOL)."""
    o, R = tool.getPosition(), tool.getOrientation()
    d = [p[i] - o[i] for i in range(3)]
    return [sum(R[3 * j + i] * d[j] for j in range(3)) for i in range(3)]


def in_grip(name, p):
    """Cube centre inside the gripper capture zone (between the pads) and in the air."""
    q = local(tools[name], p)
    return abs(q[0]) < 0.015 and abs(q[1]) < 0.015 and abs(q[2] - TCP) < 0.015 and p[2] > 0.40, q


def cube_at_stop():
    return any(math.dist(c.getPosition(), (stop[0], stop[1], 0.37)) < 0.010 for c in cubes)   # open-gripper capture range


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
            if i not in dropped:
                dropped.add(i)
                events.append((round(t, 2), f"cube {i} on the floor", [round(v, 3) for v in p]))
    for name, r in robots.items():
        for cp in r.getContactPoints(True):
            if min(math.dist(cp.getPoint(), p) for p in cps) > 0.035:
                m["robot_contacts"] += 1
                if m["robot_contacts"] <= 5 or m["robot_contacts"] % 50 == 0:
                    events.append((round(t, 2), f"{name} contact", [round(v, 3) for v in cp.getPoint()]))
    tr = spec.get("trace")
    if tr and tr[0] <= t <= tr[1] and k % 12 == 0:      # optional debug trace (every ~0.1 s)
        R = tools["ur5e"].getOrientation()
        events.append((round(t, 2), "trace", {"cube0_in_ur_mm": [round(1000 * v, 1) for v in local(tools["ur5e"], cps[0])],
                       "cube0_in_fanuc_mm": [round(1000 * v, 1) for v in local(tools["fanuc"], cps[0])],
                       "cube0_world": [round(v, 3) for v in cps[0]], "ur_grip_z_axis": [round(R[2], 2), round(R[5], 2), round(R[8], 2)]}))
    for p in cps:
        held_f, lf = in_grip("fanuc", p)
        held_u, lu = in_grip("ur5e", p)
        if held_u and not held_f:                   # UR5e alone: its gripper is closed on the cube
            m["handover_offset_mm"] = max(m["handover_offset_mm"], 1000 * abs(lu[1]))
            m["handover_width_offset_mm"] = max(m["handover_width_offset_mm"], 1000 * abs(lu[0]))
        if held_f and not held_u:                   # FANUC alone: idem
            if 1000 * abs(lf[1]) > m["handover_offset_mm"]:
                worst_handover.update(t=round(t, 3), in_fanuc_mm=[round(1000 * v, 1) for v in lf])
            m["handover_offset_mm"] = max(m["handover_offset_mm"], 1000 * abs(lf[1]))
            m["handover_width_offset_mm"] = max(m["handover_width_offset_mm"], 1000 * abs(lf[0]))
        if held_u and not held_f and tools["ur5e"].getOrientation()[8] > -0.5:   # UR gripper not pointing down
            slip = 1000 * abs(lu[2] - TCP)
            if slip > m["arm6_cube_slip_mm"]:
                worst_slip.update(t=round(t, 3), local_mm=[round(1000 * v, 1) for v in lu])
            m["arm6_cube_slip_mm"] = max(m["arm6_cube_slip_mm"], slip)
        if held_f and not held_u:
            m["fanuc_cube_slip_mm"] = max(m["fanuc_cube_slip_mm"], 1000 * abs(lf[2] - TCP))
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
json.dump({"name": spec["name"], "passed": ok, "metrics": m, "checks": results, "events": events, "worst_slip": worst_slip, "worst_handover": worst_handover,
           "final_cubes": [[round(v, 3) for v in c.getPosition()] for c in cubes]},
          open(report_path, "w"), indent=2)
print(f"[cell_test] {spec['name']}: {'PASSED' if ok else 'FAILED'}", flush=True)
sup.simulationQuit(0 if ok else 1)
