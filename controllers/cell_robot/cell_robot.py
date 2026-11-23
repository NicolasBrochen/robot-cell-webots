"""Controller of a cell robot, driven by the PLC (controllers/conveyor).

    controllerArgs: <cycle.motion> [via:<q1>,<q2>,...]
      cycle.motion  path relative to this folder; its first pose is the waiting pose
      via:...       optional safe intermediate pose reached first at start-up (rad, motion column order)

- at start-up, goes to the via pose (if any) then to the waiting pose, and sends "READY <name>"
  (channel 2) only once every joint has actually reached it;
- on "START" (channel 1) plays the cycle once, then sends "READY <name>" again.
Both robots receive START on the same simulation step: their cycles are synchronised.
Emitter "emitter" and Receiver "receiver" may sit anywhere in the robot (e.g. in its toolSlot).
"""
import os
import sys

from controller import Motion, Robot

robot = Robot()
dt = int(robot.getBasicTimeStep())
name = robot.getName()
path = os.path.join(os.path.dirname(os.path.abspath(__file__)), sys.argv[1])
via = [float(v) for a in sys.argv[2:] if a.startswith("via:") for v in a[4:].split(",")]
motion = Motion(path)
if not motion.isValid():
    print(f"[{name}] invalid motion file: {path}", flush=True)
    sys.exit(1)

with open(path) as f:                     # first pose of the cycle = waiting pose
    header = f.readline().strip().split(",")[2:]
    first = [float(v) for v in f.readline().strip().split(",")[2:]]
motors = [robot.getDevice(n) for n in header]
sensors = []
for m in motors:
    s = m.getPositionSensor()
    if s is not None:
        s.enable(dt)
    sensors.append(s)


def reach(target, tol=0.01, timeout=10.0):
    for m, q in zip(motors, target):
        m.setPosition(q)
    t0 = robot.getTime()
    while robot.step(dt) != -1:
        done = all(s is None or abs(s.getValue() - q) < tol or "finger" in m.getName()
                   for m, s, q in zip(motors, sensors, target))
        if done or robot.getTime() - t0 > timeout:
            return


if via:
    reach(via + first[len(via):])
reach(first)

emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")
receiver.enable(dt)
emitter.send(f"READY {name}".encode())
busy = False
while robot.step(dt) != -1:
    while receiver.getQueueLength() > 0:
        msg = receiver.getString()
        receiver.nextPacket()
        if msg == "START" and not busy:
            motion.play()
            busy = True
    if busy and motion.isOver():
        busy = False
        emitter.send(f"READY {name}".encode())
