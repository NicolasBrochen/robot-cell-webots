#!/usr/bin/env bash
# Re-records the README media from worlds/robot_cell.wbt (needs a display: Webots renders the movies):
#   docs/robot_cell.mp4  one production cycle, overview
#   docs/cell.png        frame of that cycle
#   docs/handover.png    close-up of the in-air hand-over
#   docs/robot_cell.gif  animated preview (robots area of the cycle)
#   tools/build_media.sh
set -eu
ROOT=$(cd "$(dirname "$0")/.." && pwd)
WEBOTS=${WEBOTS:-webots}
tmpd=$(mktemp -d)
trap 'rm -rf "$tmpd" "$ROOT/worlds/.media_record.wbt"' EXIT
cat > "$tmpd/shots.json" <<JSON
{"width": 1280, "height": 720, "warmup": 1, "clips": [
  {"file": "$tmpd/cycle.mp4", "start": 0, "duration": 14.4,
   "position": ${OVERVIEW_POS:-[5.0, -4.2, 2.2]}, "orientation": ${OVERVIEW_ORI:-[-0.1834, 0.0741, 0.9802, 2.3871]}},
  {"file": "$tmpd/handover.mp4", "start": 4.6, "duration": 2.6,
   "position": ${HAND_POS:-[0.75, -0.55, 0.62]}, "orientation": ${HAND_ORI:-[-0.0943, 0.0560, 0.9940, 2.0755]}}
]}
JSON
world="$ROOT/worlds/.media_record.wbt"
sed 's/^Viewpoint {/DEF VIEWPOINT Viewpoint {/' "$ROOT/worlds/robot_cell.wbt" > "$world"
cat >> "$world" <<WBT
DEF RECORDER Robot {
  name "media_recorder"
  controller "media_recorder"
  controllerArgs [ "$tmpd/shots.json" ]
  supervisor TRUE
  children [ Receiver { name "receiver" channel 1 } ]
}
WBT
timeout 900 "$WEBOTS" --batch --mode=fast --stdout --stderr "$world" > "$tmpd/webots.log" 2>&1 || true
for f in cycle.mp4 handover.mp4; do
  [ -s "$tmpd/$f" ] || { tail -30 "$tmpd/webots.log"; echo "✗ $f not recorded"; exit 1; }
done
mkdir -p "${OUT:-$ROOT/docs}"
cp "$tmpd/cycle.mp4" "${OUT:-$ROOT/docs}/robot_cell.mp4"
ffmpeg -loglevel error -y -ss "${CELL_T:-6.6}" -i "$tmpd/cycle.mp4" -frames:v 1 "${OUT:-$ROOT/docs}/cell.png"
ffmpeg -loglevel error -y -ss "${HAND_T:-1.0}" -i "$tmpd/handover.mp4" -frames:v 1 "${OUT:-$ROOT/docs}/handover.png"
[ -n "${KEEP:-}" ] && cp "$tmpd/handover.mp4" "${OUT:-$ROOT/docs}/handover_clip.mp4"
ls -la "${OUT:-$ROOT/docs}"
# animated preview for the README: the robots area of the cycle, 12 fps
ffmpeg -loglevel error -y -i "${OUT:-$ROOT/docs}/robot_cell.mp4" -vf "fps=12,crop=560:360:400:90,split[a][b];[a]palettegen=max_colors=128:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=3:diff_mode=rectangle" "${OUT:-$ROOT/docs}/robot_cell.gif"
