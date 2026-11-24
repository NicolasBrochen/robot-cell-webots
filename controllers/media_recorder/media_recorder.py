"""Records the README media (docs/robot_cell.mp4, docs/cell.png, docs/handover.png), then quits.
Added to a copy of the world by tools/build_media.sh.

    controllerArgs: <shots.json>
shots.json: {"width": W, "height": H, "warmup": <cycles skipped>, "clips": [
    {"file": "...mp4", "start": <s after START>, "duration": <s>, "position": [x,y,z], "orientation": [ax,ay,az,a]}]}
Each clip starts on a new START order of the PLC (channel 1), one clip per production cycle.
"""
import json
import sys

from controller import Supervisor

sup = Supervisor()
dt = int(sup.getBasicTimeStep())
cfg = json.load(open(sys.argv[1]))
rx = sup.getDevice("receiver")
rx.enable(dt)
vp = sup.getFromDef("VIEWPOINT")
clips = list(cfg["clips"])
skip = cfg.get("warmup", 0)
t0 = None
clip = None
recording = False
while sup.step(dt) != -1:
    while rx.getQueueLength() > 0:
        msg = rx.getString()
        rx.nextPacket()
        if msg == "START" and clip is None:
            if skip > 0:
                skip -= 1
                continue
            if not clips:
                sup.simulationQuit(0)
            clip = clips.pop(0)
            t0 = sup.getTime()
            vp.getField("position").setSFVec3f(clip["position"])
            vp.getField("orientation").setSFRotation(clip["orientation"])
    if clip is None:
        continue
    t = sup.getTime() - t0
    if not recording and t >= clip.get("start", 0):
        sup.movieStartRecording(clip["file"], cfg["width"], cfg["height"], 0, 100, 1, False)
        recording = True
    if recording and t >= clip.get("start", 0) + clip["duration"]:
        sup.movieStopRecording()
        while not sup.movieIsReady():
            sup.step(dt)
        recording = False
        clip = None
        if not clips:
            sup.simulationQuit(0)
