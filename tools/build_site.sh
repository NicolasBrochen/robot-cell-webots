#!/usr/bin/env bash
# Build site/: a Webots HTML5 animation of one production cycle (interactive 3D, plays in any browser).
#   tools/build_site.sh [plain|frames]      default plain (opaque robots, no joint frames)
# Publish it on GitHub Pages with:  git subtree push --prefix site origin gh-pages
set -eu
ROOT=$(cd "$(dirname "$0")/.." && pwd)
WEBOTS=${WEBOTS:-webots}
STYLE=${1:-plain}
world="$ROOT/worlds/robot_cell_plain.wbt"; [ "$STYLE" = frames ] && world="$ROOT/worlds/robot_cell.wbt"
tmp="$ROOT/worlds/.site_record.wbt"
cp "$world" "$tmp"
cat >> "$tmp" <<WBT
DEF RECORDER Robot {
  name "animation_recorder"
  controller "animation_recorder"
  controllerArgs [ "../../site/robot_cell.html" "14.5" ]
  supervisor TRUE
  children [ Receiver { name "receiver" channel 1 } ]
}
WBT
rm -f "$ROOT"/site/robot_cell.{html,css,json,w3d}
timeout 600 "$WEBOTS" --batch --mode=fast --minimize --stdout --stderr "$tmp" > "$ROOT/site/.build.log" 2>&1 || true
rm -f "$tmp"
for f in robot_cell.html robot_cell.json robot_cell.w3d; do
  [ -s "$ROOT/site/$f" ] || { echo "✗ site/$f missing (see site/.build.log)"; exit 1; }
done
rm -f "$ROOT/site/.build.log"
du -sh "$ROOT/site"
