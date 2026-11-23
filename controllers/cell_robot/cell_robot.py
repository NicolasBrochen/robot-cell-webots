"""Controller of a cell robot, driven by the PLC (controllers/conveyor).

    controllerArgs: path of the cycle .motion file, relative to this folder.

- on start-up, holds the first pose of the cycle and sends "READY <name>" (channel 2);
- on "START" (channel 1) plays the cycle once, then sends "READY <name>" again.
Both robots receive START on the same simulation step: their cycles are synchronised.
"""
import os
import sys

from controller import Motion, Robot

robot = Robot()
dt = int(robot.getBasicTimeStep())
name = robot.getName()
path = os.path.join(os.path.dirname(os.path.abspath(__file__)), sys.argv[1])
motion = Motion(path)
if not motion.isValid():
    print(f"[{name}] invalid motion file: {path}", flush=True)
    sys.exit(1)

with open(path) as f:                     # first pose of the cycle = waiting pose
    header = f.readline().strip().split(",")[2:]
    first = f.readline().strip().split(",")[2:]
for dev_name, value in zip(header, first):
    robot.getDevice(dev_name).setPosition(float(value))

emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")
receiver.enable(dt)
busy, announced = False, False

while robot.step(dt) != -1:
    if not announced:
        emitter.send(f"READY {name}".encode())
        announced = True
    while receiver.getQueueLength() > 0:
        msg = receiver.getString()
        receiver.nextPacket()
        if msg == "START" and not busy:
            motion.play()
            busy = True
    if busy and motion.isOver():
        busy = False
        emitter.send(f"READY {name}".encode())
