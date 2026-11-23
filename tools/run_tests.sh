#!/usr/bin/env bash
# Run the cell scenarios of tests/*.test.json headless (Webots --batch --mode=fast --no-rendering).
#   tools/run_tests.sh [tests/x.test.json ...]      default: all tests
# Each scenario runs on a copy of worlds/robot_cell.wbt with the cell_test supervisor added;
# reports are written to tests/results/<name>.json. Exit code = number of failed scenarios.
set -u
ROOT=$(cd "$(dirname "$0")/.." && pwd)
WEBOTS=${WEBOTS:-webots}
WORLD=${WORLD:-$ROOT/worlds/robot_cell.wbt}
mkdir -p "$ROOT/tests/results"
specs=("$@")
[ ${#specs[@]} -eq 0 ] && specs=("$ROOT"/tests/*.test.json)
failed=0
for spec in "${specs[@]}"; do
  name=$(basename "$spec" .test.json)
  world="$ROOT/worlds/.test_${name}.wbt"
  cp "$WORLD" "$world"
  cat >> "$world" <<EOF
DEF CELL_TEST Robot {
  name "cell_test"
  controller "cell_test"
  controllerArgs [ "../../tests/${name}.test.json" "../../tests/results/${name}.json" ]
  supervisor TRUE
  children [ Receiver { name "receiver" channel 1 } ]
}
EOF
  echo "== $name"
  report="$ROOT/tests/results/${name}.json"
  rm -f "$report"
  timeout 600 "$WEBOTS" --batch --mode=fast --no-rendering --minimize --stdout --stderr "$world" \
    > "$ROOT/tests/results/${name}.log" 2>&1
  status=$?
  if [ -f "$report" ]; then
    python3 - "$report" <<'PY'
import json, sys
r = json.load(open(sys.argv[1]))
for c in r["checks"]:
    bound = " and ".join(x for x in (f">= {c['min']}" if "min" in c else "", f"<= {c['max']}" if "max" in c else "") if x)
    print(f"  [{'PASS' if c['passed'] else 'FAIL'}] {c['metric']:22s} = {c['value']:9.3f}   (expected {bound})")
PY
  else
    echo "  no report (see tests/results/${name}.log)"; status=1
  fi
  rm -f "$world"
  if [ "$status" -ne 0 ]; then failed=$((failed + 1)); fi
done
echo
[ $failed -eq 0 ] && echo "ALL SCENARIOS PASSED" || echo "$failed SCENARIO(S) FAILED"
exit $failed
