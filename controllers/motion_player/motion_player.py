"""Plays a Webots .motion file in a loop (used by the examples).

    controllerArgs: path of the .motion file, relative to this folder.
The robot first moves to the first pose of the motion, then loops it forever.
"""
import os
import sys

from controller import Motion, Robot

robot = Robot()
dt = int(robot.getBasicTimeStep())
path = os.path.join(os.path.dirname(os.path.abspath(__file__)), sys.argv[1])
motion = Motion(path)
if not motion.isValid():
    print(f"[motion_player] invalid motion file: {path}", flush=True)
    sys.exit(1)
with open(path) as f:
    header = f.readline().strip().split(",")[2:]
    first = f.readline().strip().split(",")[2:]
for dev_name, value in zip(header, first):
    robot.getDevice(dev_name).setPosition(float(value))
for _ in range(int(1000 / dt)):          # 1 s to reach the first pose
    robot.step(dt)
motion.setLoop(True)
motion.play()
while robot.step(dt) != -1:
    pass
