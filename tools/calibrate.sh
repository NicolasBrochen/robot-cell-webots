#!/usr/bin/env bash
# Calibrate the models against Webots itself (headless):
#  1. kinematics: UR5e and FANUC driven through 31 random joint configurations; the measured pose of
#     the Robotiq node is compared with tools/kinematics_lib.py (expected < 1 mm, < 0.1 deg);
#  2. Robotiq 2F-85 grasp: closes on the 40 mm cube at several depths, lifts it, reports hold and
#     finger positions (gives TCP and G_CLOSE of tools/cell_layout.py).
# Results: tests/results/kin_calib_*.json, tests/results/grip_calib.json.
set -eu
ROOT=$(cd "$(dirname "$0")/.." && pwd)
WEBOTS=${WEBOTS:-webots}
PY=${PYTHON:-$ROOT/.venv/bin/python}
W=https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects
R=$ROOT/tests/results
mkdir -p "$R"
run() { timeout 600 "$WEBOTS" --batch --mode=fast --no-rendering --minimize --stdout --stderr "$1" > "$1.log" 2>&1 || true; rm -f "$1" "$1.log"; }

cat > "$ROOT/worlds/.calib_kin.wbt" <<WBT
#VRML_SIM R2025a utf8
EXTERNPROTO "$W/robots/universal_robots/protos/UR5e.proto"
EXTERNPROTO "$W/devices/robotiq/protos/Robotiq2f85Gripper.proto"
EXTERNPROTO "../protos/FanucLrMate200iD.proto"
WorldInfo { basicTimeStep 8 coordinateSystem "ENU" }   # no floor: random poses must not be blocked
Viewpoint { position 1 -3 1.5 orientation -0.2 0.1 0.97 1.6 }
UR5e { name "ur5e" selfCollision FALSE supervisor TRUE controller "kin_calib"
  controllerArgs [ "UR_GRIP" "../../tests/results/kin_calib_ur5e.json" "1" ]
  toolSlot [ DEF UR_GRIP Robotiq2f85Gripper { rotation 1 0 0 -1.5708 } ] }
FanucLrMate200iD { translation 1.5 0 0.001 name "fanuc" supervisor TRUE controller "kin_calib"
  controllerArgs [ "FA_GRIP" "../../tests/results/kin_calib_fanuc.json" "2" ]
  toolSlot [ DEF FA_GRIP Robotiq2f85Gripper { } ] }
WBT
rm -f "$R"/kin_calib_*.json*
cd "$ROOT"
"$WEBOTS" --batch --mode=fast --no-rendering --minimize --stdout --stderr worlds/.calib_kin.wbt > /dev/null 2>&1 &
wpid=$!
for _ in $(seq 300); do         # both robots write a .done file when their 31 configurations are measured
  [ -f "$R/kin_calib_ur5e.json.done" ] && [ -f "$R/kin_calib_fanuc.json.done" ] && break
  kill -0 $wpid 2>/dev/null || break
  sleep 2
done
kill $wpid 2>/dev/null || true; wait $wpid 2>/dev/null || true
rm -f "$ROOT/worlds/.calib_kin.wbt" "$R"/*.done
"$PY" - "$R" <<'PY'
import json, math, sys, numpy as np
sys.path.insert(0, sys.argv[1] + "/../../tools")
from kinematics_lib import FANUC, UR5E, log_rot
for chain, f in ((UR5E, "kin_calib_ur5e.json"), (FANUC, "kin_calib_fanuc.json")):
    d = json.load(open(f"{sys.argv[1]}/{f}"))
    Rb = np.array(d["base_R"]).reshape(3, 3); pb = np.array(d["base_pos"])
    ep = er = 0.0
    for s in d["samples"]:
        T = chain.fk(np.array(s["q"][:6]), tcp_offset=0.0)
        ep = max(ep, np.linalg.norm(Rb @ T[:3, 3] + pb - np.array(s["pos"])))
        er = max(er, np.linalg.norm(log_rot(np.array(s["R"]).reshape(3, 3) @ (Rb @ T[:3, :3]).T)))
    ok = ep < 1e-3 and math.degrees(er) < 0.1
    print(f"{'PASS' if ok else 'FAIL'}  {chain.name:20s} FK vs Webots over {len(d['samples'])} configurations: "
          f"{1000 * ep:.3f} mm, {math.degrees(er):.4f} deg")
PY

sed -e "s#^EXTERNPROTO \"https://[^\"]*Robotiq2f85Gripper.proto\"#EXTERNPROTO \"$W/devices/robotiq/protos/Robotiq2f85Gripper.proto\"#" \
    -e 's#controllerArgs \[ [^]]* \]#controllerArgs [ "../../tests/results/grip_calib.json" "0.20" "0.18" "0.17" "0.16" "0.15" "0.14" ]#' \
    "$ROOT/tools/grip_calib.wbt.in" > "$ROOT/worlds/.calib_grip.wbt"
( cd "$ROOT" && run worlds/.calib_grip.wbt )
"$PY" -c "
import json
for r in json.load(open('$R/grip_calib.json')):
    print(f\"Robotiq  depth {r['depth_mm']:5.0f} mm  fingers {r['finger_pos'][0]:.3f} rad  cube held after a 10 cm lift: {r['held']}\")"
