"""Records one production cycle of the cell as a Webots HTML5 animation, then quits.
Added to a copy of the world by tools/build_site.sh.

    controllerArgs: <output.html> <duration_s>
Recording starts on the first START order of the PLC (listened on channel 1).
"""
import sys

from controller import Supervisor

sup = Supervisor()
dt = int(sup.getBasicTimeStep())
out, duration = sys.argv[1], float(sys.argv[2])
rx = sup.getDevice("receiver")
rx.enable(dt)
t0 = None
while sup.step(dt) != -1:
    while rx.getQueueLength() > 0:
        msg = rx.getString()
        rx.nextPacket()
        if msg == "START" and t0 is None:
            sup.animationStartRecording(out)
            t0 = sup.getTime()
    if t0 is not None and sup.getTime() - t0 >= duration:
        sup.animationStopRecording()
        sup.step(dt)
        break
sup.simulationQuit(0)
