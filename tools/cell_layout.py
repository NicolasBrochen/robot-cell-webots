"""Layout of the cell (world frame, ENU, metres). Single source of truth for
tools/gen_motions.py, tools/build_world.py and tools/check_kin.py."""
import math

SCARA_BASE = (0.0, 0.0, 0.30)        # SCARA on a 0.30 m pedestal
ARM_BASE = (1.20, 0.0, 0.0)          # 6-axis arm, yawed 180 deg (faces -x)
ARM_YAW = math.pi

I = (0.30, 0.35)                     # input station = conveyor end stop (cube centre z = 0.37)
H = (0.60, 0.0)                      # hand-over point (cube centre z = 0.40)
O = (0.95, -0.30)                    # output bin

BELT_TOP = 0.35
CUBE = 0.04                          # cube edge (m), 50 g
CUBE_MASS = 0.05

# gripper: contact at 10 mm of finger travel, 0.5 mm squeeze
OPEN, CLOSE = 0.0, 0.0105

# SCARA heights: finger centre = 0.475 - d3
S_UP, S_GRASP, S_PLACE, S_HAND = 0.0, 0.09, 0.088, 0.06
# 6-axis finger-centre heights (world). Hand-over grasp spans the full cube height,
# palm 1.5 mm below the cube so it settles on the palm when the SCARA releases it.
CUBE_AT_H = 0.475 - S_HAND - 0.015
R_HOVER = 0.20
R_HAND = CUBE_AT_H - 0.0015
R_HPRE = R_HAND - 0.08

CYCLE = 14.4                         # s, one cube per cycle
DT = 0.04                            # motion key-frame period

# fixed obstacles checked against the 6-axis arm (centre, size)
STATIC = {
    "SCARA pedestal": ((0.0, 0.0, 0.15), (0.3, 0.3, 0.3)),
    "conveyor": ((0.30, 0.89, 0.18), (0.14, 1.12, 0.36)),
    "end stop": ((0.30, 0.325, 0.365), (0.10, 0.01, 0.03)),
    "output bin": ((O[0], O[1], 0.07), (0.25, 0.25, 0.14)),
}
