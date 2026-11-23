"""Infeed conveyor + cell PLC (supervisor controller of the BeltConveyor PROTO).

Conveyor
  - animates the belt slats (DEF SLAT_0..SLAT_n, given in the PROTO `slats` field);
  - drives every cube lying on the belt towards the end stop (the belt itself is
    frictionless: the controller imposes the belt speed on the cubes);
  - feeds a new cube at the head of the belt as soon as the belt and the stop are empty
    (one cube at a time on the belt);
  - recycles the cubes dropped in the output bin (still for 1.5 s) into a reserve parked
    out of sight (x > 5 m), so that the cell runs forever with a few cubes.

PLC
  - reads the through-beam sensor "presence_butee" aimed at the end stop;
  - sends START on channel 1 when a cube has been present for 0.3 s AND both robots
    have reported "READY <name>" on channel 2: the robots start their cycle on the same step.
"""
import math

from controller import Supervisor

SPEED = 0.15                 # belt speed (m/s), towards -y
BELT_TOP = 0.35
FEED_DISTANCE = 1.03         # cube release point, measured from the stop along +y
SLAT_PITCH = 0.05
SLAT_Y_MIN, SLAT_Y_MAX = -0.015, 1.095   # slat travel in the PROTO frame
PRESENCE_MAX = 0.08          # m: below this the beam sees a cube
PRESENCE_DELAY = 0.3         # s of stable presence before a cycle starts
ROBOTS = {"fanuc", "ur5e"}
RESERVE_X = 10.0

sup = Supervisor()
dt = int(sup.getBasicTimeStep())
me = sup.getSelf()
X0, Y_STOP, _ = me.getPosition()          # origin of the PROTO = cube centre at the stop
Y_START = Y_STOP + FEED_DISTANCE
Y_BELT_END = Y_STOP + 1.095

bin_node = sup.getFromDef("OUTPUT_BIN")
BIN = bin_node.getPosition() if bin_node else (0.95, -0.30, 0.0)
BIN_HALF = 0.125

cubes = []
while sup.getFromDef(f"CUBE_{len(cubes)}") is not None:
    cubes.append(sup.getFromDef(f"CUBE_{len(cubes)}"))
slats = []
while sup.getFromDef(f"SLAT_{len(slats)}") is not None:      # nodes of the PROTO `slats` field
    slats.append(sup.getFromDef(f"SLAT_{len(slats)}").getField("translation"))

sensor = sup.getDevice("presence_butee")
sensor.enable(dt)
emitter = sup.getDevice("emitter")
receiver = sup.getDevice("receiver")
receiver.enable(dt)

still_since, ready = {}, set()
present_since, cycles = None, 0


def on_belt(p):
    # resting flat on the belt (+-3 mm) and more than 6 mm before the stop: a cube at the stop,
    # or already lifted by the gripper, must never be driven (it would be torn out of the grasp)
    return (abs(p[0] - X0) < 0.05 and Y_STOP + 0.006 < p[1] < Y_BELT_END
            and abs(p[2] - (BELT_TOP + 0.02)) < 0.003)


def at_stop(p):
    return abs(p[0] - X0) < 0.03 and abs(p[1] - Y_STOP) < 0.01 and BELT_TOP < p[2] < BELT_TOP + 0.03


def in_bin(p):
    return abs(p[0] - BIN[0]) < BIN_HALF and abs(p[1] - BIN[1]) < BIN_HALF and p[2] < 0.12


def park(node):
    return [RESERVE_X + 0.1 * cubes.index(node), RESERVE_X, 0.0205]


def place(node, xyz):
    node.getField("translation").setSFVec3f(list(xyz))
    node.getField("rotation").setSFRotation([0, 0, 1, 0])
    node.resetPhysics()


while sup.step(dt) != -1:
    t = sup.getTime()
    # ---- belt slats
    span = SLAT_PITCH * len(slats)
    for k, f in enumerate(slats):
        y = SLAT_Y_MAX - ((t * SPEED + k * SLAT_PITCH) % span)
        f.setSFVec3f([0, y if y >= SLAT_Y_MIN else y + span, 0.351])
    # ---- cubes
    busy = False
    for c in cubes:
        p = c.getPosition()
        if p[0] > RESERVE_X / 2:                 # reserve: keep it parked
            place(c, park(c))
        elif on_belt(p):
            busy = True
            v = c.getVelocity()
            c.setVelocity([-(p[0] - X0) * 2.0, -SPEED, v[2], 0, 0, 0])
        elif at_stop(p):
            busy = True
            # keep it pressed against the stop: the frictionless belt lets it drift a few mm away,
            # which would offset it in the gripper; only while it lies flat (never during a grasp)
            if p[1] > Y_STOP + 0.001 and abs(p[2] - (BELT_TOP + 0.02)) < 0.001:
                v = c.getVelocity()
                c.setVelocity([0, -0.02, v[2], 0, 0, 0])
        elif in_bin(p):
            if math.sqrt(sum(x * x for x in c.getVelocity()[:3])) < 0.005:
                still_since.setdefault(c.getId(), t)
                if t - still_since[c.getId()] > 1.5:
                    place(c, park(c))
                    still_since.pop(c.getId(), None)
            else:
                still_since.pop(c.getId(), None)
    if not busy:
        for c in cubes:
            if c.getPosition()[0] > RESERVE_X / 2:
                place(c, [X0, Y_START, BELT_TOP + 0.0205])
                break
    # ---- PLC
    while receiver.getQueueLength() > 0:
        msg = receiver.getString()
        receiver.nextPacket()
        if msg.startswith("READY "):
            ready.add(msg.split()[1])
    if sensor.getValue() < PRESENCE_MAX:
        present_since = t if present_since is None else present_since
    else:
        present_since = None
    if present_since is not None and t - present_since >= PRESENCE_DELAY and ROBOTS <= ready:
        emitter.send(b"START")
        ready.clear()
        cycles += 1
        print(f"[plc] cube at the end stop -> START cycle {cycles} (t={t:.2f} s)", flush=True)
