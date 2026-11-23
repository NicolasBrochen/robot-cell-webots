"""Layout of the cell (world frame, ENU, metres). Single source of truth for
tools/gen_motions.py, tools/build_world.py and tools/check_kin.py.

Robots of the Webots / ROS-Industrial libraries, each with a Robotiq 2F-85 gripper:
  FANUC LR Mate 200iD : picks the cube at the conveyor end stop (top grasp, pads on the +-x faces)
                        and presents it at the hand-over point H.
  UR5e                : takes it from below (pads on the +-y faces), flips it and drops it into the bin.
"""
import math

FANUC_BASE = (0.0, 0.0, 0.001)       # 1 mm above the floor (static base mesh must not coincide with it)
FANUC_YAW = 0.0
UR_BASE = (1.0, 0.0, 0.02)           # on a 2 cm plate: its zero pose would touch the floor
UR_YAW = math.pi                     # faces -x, towards the FANUC

I = (0.30, 0.35)                     # input station = conveyor end stop
H = (0.45, 0.0)                      # hand-over point
O = (0.85, -0.45)                    # output bin (clear of the UR5e zero pose)

BELT_TOP = 0.35
CUBE = 0.04                          # cube edge (m), 50 g
CUBE_MASS = 0.05
CUBE_AT_I = BELT_TOP + CUBE / 2      # cube centre at the end stop
CUBE_AT_H = 0.50                     # cube centre at the hand-over

# Robotiq 2F-85 (measured in Webots, tools/grip_calib): fingers stop at ~0.45 rad on the 40 mm cube,
# the pads reach 160 mm along the gripper z axis. Grasp with the cube centre at TCP mm from the
# gripper origin: the pads cover 30 mm of the cube on the side of the gripper.
TCP = 0.15
G_OPEN, G_HALF, G_CLOSE = 0.0, 0.3, 0.8   # finger joint targets (rad): 85 mm, ~55 mm, closed (torque limited)

# gripper-origin heights (world z)
F_HOVER = CUBE_AT_I + TCP + 0.10          # FANUC above the end stop
F_GRASP = CUBE_AT_I + TCP                 # FANUC grasping at the end stop
F_HAND = CUBE_AT_H + TCP                  # FANUC presenting the cube at H (from above)
F_LEAVE = F_HAND + 0.05                  # pads clear the top of the cube
U_HAND = CUBE_AT_H - TCP                  # UR5e grasping at H (from below, gripper up)
U_PRE = U_HAND - 0.08
U_DROP = 0.37                             # UR5e gripper origin above the bin (cube bottom 0.20 m)

CYCLE = 14.4
DT = 0.04

# fixed obstacles (centre, size) used by the reach / clearance report
STATIC = {
    "conveyor": ((0.30, 0.89, 0.18), (0.14, 1.12, 0.36)),
    "end stop": ((0.30, 0.325, 0.365), (0.10, 0.01, 0.03)),
    "output bin": ((O[0], O[1], 0.07), (0.25, 0.25, 0.14)),
}
