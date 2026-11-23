#!/usr/bin/env bash
# Headless check of the example worlds: example 01 must run without error, examples 02-04
# must place the cube on B then bring it back on A (controllers/example_check).
#   tools/run_examples.sh        (exit code = number of failed examples)
set -u
ROOT=$(cd "$(dirname "$0")/.." && pwd)
WEBOTS=${WEBOTS:-webots}
mkdir -p "$ROOT/tests/results"
failed=0

run() {   # run <world> <duration> <with_check>
  local world=$1 duration=$2 check=$3 name tmp report status
  name=$(basename "$world" .wbt)
  tmp="$ROOT/worlds/.check_${name}.wbt"
  report="$ROOT/tests/results/${name}.json"
  rm -f "$report"
  cp "$world" "$tmp"
  if [ "$check" = 1 ]; then
    printf 'DEF CHECK Robot { name "example_check" controller "example_check" supervisor TRUE controllerArgs [ "%s" "../../tests/results/%s.json" ] }\n' "$duration" "$name" >> "$tmp"
    timeout 300 "$WEBOTS" --batch --mode=fast --no-rendering --minimize --stdout --stderr "$tmp" > "$ROOT/tests/results/${name}.log" 2>&1
    status=$?
    python3 -c "import json,sys; r=json.load(open(sys.argv[1])); print('  ', 'PASS' if r['passed'] else 'FAIL', r['metrics'])" "$report" 2>/dev/null \
      || { echo "   no report (see tests/results/${name}.log)"; status=1; }
  else
    timeout "$duration" "$WEBOTS" --batch --mode=fast --no-rendering --minimize --stdout --stderr "$tmp" > "$ROOT/tests/results/${name}.log" 2>&1
    if grep -qiE "error|traceback" "$ROOT/tests/results/${name}.log"; then status=1; echo "   FAIL (errors in the log)"; else status=0; echo "   PASS (ran ${duration} s without error)"; fi
  fi
  rm -f "$tmp"
  [ "$status" -ne 0 ] && failed=$((failed + 1))
}

echo "== example_01_scara_trajectory";  run "$ROOT/worlds/example_01_scara_trajectory.wbt" 15 0
echo "== example_02_scara_pick_place";  run "$ROOT/worlds/example_02_scara_pick_place.wbt" 17 1
echo "== example_03_arm6_pick_place";   run "$ROOT/worlds/example_03_arm6_pick_place.wbt" 17 1
echo "== example_04_arm6_obstacles";    run "$ROOT/worlds/example_04_arm6_obstacles.wbt" 19 1
echo
[ $failed -eq 0 ] && echo "ALL EXAMPLES PASSED" || echo "$failed EXAMPLE(S) FAILED"
exit $failed
