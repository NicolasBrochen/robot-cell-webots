"""Robotiq 2F-85 grasp calibration on the 40 mm / 50 g cube (run by tools/calibrate.sh).
    controllerArgs: <output.json> <depth_1> <depth_2> ...   (gripper origin -> cube centre, m)
The gripper points down on a motorised lift; for each depth it closes on the cube, lifts it 10 cm
and records the finger joint positions, the cube pose in the gripper frame and whether it is held."""
import json, math, sys
from controller import Supervisor
sup = Supervisor(); dt = int(sup.getBasicTimeStep())
lift = sup.getDevice("lift"); lift_s = sup.getDevice("lift_sensor")
grip = sup.getFromDef("GRIP"); cube = sup.getFromDef("CUBE")
m = [sup.getDevice("ROBOTIQ 2F-85 Gripper::left finger joint"), sup.getDevice("ROBOTIQ 2F-85 Gripper::right finger joint")]
s = [sup.getDevice("ROBOTIQ 2F-85 Gripper left finger joint sensor"), sup.getDevice("ROBOTIQ 2F-85 Gripper right finger sensor")]
for x in s: x.enable(dt)
lift_s.enable(dt)
cube.enableContactPointsTracking(dt)
CUBE0 = [0, 0, 0.07]
out = []
def wait(sec):
    for _ in range(int(sec * 1000 / dt)): sup.step(dt)
def local(p):
    o, R = grip.getPosition(), grip.getOrientation(); d = [p[i] - o[i] for i in range(3)]
    return [sum(R[3 * j + i] * d[j] for j in range(3)) for i in range(3)]
for depth in [float(v) for v in sys.argv[2:]]:
    for x in m: x.setPosition(0.0)
    lift.setPosition(0.0); wait(0.8)
    cube.getField("translation").setSFVec3f(CUBE0); cube.getField("rotation").setSFRotation([0, 0, 1, 0]); cube.resetPhysics(); wait(0.3)
    lift.setPosition(0.5 - (CUBE0[2] + depth)); wait(1.0)
    for x in m: x.setPosition(0.8)
    wait(1.5)
    cps = [local(c.getPoint()) for c in cube.getContactPoints() if c.getPoint()[2] > 0.052]
    closed = [round(x.getValue(), 4) for x in s]
    c_before = [round(1000 * v, 2) for v in local(cube.getPosition())]
    lift.setPosition(0.5 - (CUBE0[2] + depth) - 0.10); wait(1.5)        # lift 10 cm
    c_after = [round(1000 * v, 2) for v in local(cube.getPosition())]
    out.append({"depth_mm": round(1000 * depth, 1), "finger_pos": closed, "cube_local_mm_before": c_before,
                "cube_local_mm_after": c_after, "held": cube.getPosition()[2] > CUBE0[2] + 0.08,
                "contact_z_mm": [round(1000 * min(p[2] for p in cps), 1), round(1000 * max(p[2] for p in cps), 1)] if cps else None,
                "contact_y_mm": [round(1000 * min(p[1] for p in cps), 1), round(1000 * max(p[1] for p in cps), 1)] if cps else None})
json.dump(out, open(sys.argv[1], "w"), indent=1)
sup.simulationQuit(0)
