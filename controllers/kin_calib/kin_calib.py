"""Kinematics calibration: drives the robot through random joint configurations and records,
for each one, the measured joint positions and the world pose of the gripper node.

    controllerArgs: <gripper DEF> <output.json> <seed>
The robot must have supervisor TRUE (it reads its own gripper node by DEF).
"""
import json
import random
import sys

from controller import Node, Supervisor

sup = Supervisor()
dt = int(sup.getBasicTimeStep())
grip_def, out, seed = sys.argv[1], sys.argv[2], int(sys.argv[3])
motors = [sup.getDeviceByIndex(i) for i in range(sup.getNumberOfDevices())]
motors = [m for m in motors if m.getNodeType() == Node.ROTATIONAL_MOTOR]
sensors = [m.getPositionSensor() for m in motors]
for s in sensors:
    s.enable(dt)
grip = sup.getFromDef(grip_def)
me = sup.getSelf()
rng = random.Random(seed)
samples = []
for k in range(31):
    if k == 0:
        target = [0.0] * len(motors)
    else:
        target = []
        for m in motors:
            lo, hi = max(m.getMinPosition(), -2.6), min(m.getMaxPosition(), 2.6)
            target.append(rng.uniform(lo, hi))
    for m, q in zip(motors, target):
        m.setPosition(q)
    for _ in range(int(3000 / dt)):
        sup.step(dt)
    samples.append({"target": target, "q": [s.getValue() for s in sensors],
                    "pos": list(grip.getPosition()), "R": list(grip.getOrientation())})
json.dump({"motors": [m.getName() for m in motors], "base_pos": list(me.getPosition()),
           "base_R": list(me.getOrientation()), "samples": samples}, open(out, "w"), indent=1)
open(out + ".done", "w").write("ok")
