#!/usr/bin/env bash
# Mutation check: the scenarios must FAIL on deliberately broken versions of the cell.
# Each mutation is applied to a scratch copy of the repository; the expected test must fail.
#   tools/perturb.sh            (exit code = number of mutations NOT caught)
set -u
ROOT=$(cd "$(dirname "$0")/.." && pwd)
PY=${PYTHON:-$ROOT/.venv/bin/python}
missed=0

mutate() {   # mutate <label> <test> <python snippet editing files in the copy>
  local label=$1 test=$2 code=$3
  local work
  work=$(mktemp -d)
  cp -r "$ROOT"/{controllers,motions,protos,tests,tools,worlds} "$work"/
  rm -rf "$work"/tests/results
  (cd "$work" && "$PY" -c "$code") || { echo "  could not apply mutation"; missed=$((missed + 1)); return; }
  (cd "$work" && "$PY" tools/gen_motions.py >/dev/null && "$PY" tools/build_world.py >/dev/null)
  echo "== mutation: $label  (expects tests/$test to fail)"
  if (cd "$work" && tools/run_tests.sh "tests/$test" | grep -E "\[(PASS|FAIL)\]" | sed 's/^/  /'; exit "${PIPESTATUS[0]}"); then
    echo "  MISSED: the test still passes"; missed=$((missed + 1))
  else
    echo "  caught"
  fi
  rm -rf "$work"
}

mutate "PLC ignores the presence sensor" presence-interlock.test.json '
p = "controllers/conveyor/conveyor.py"; s = open(p).read()
s = s.replace("if present_since is not None and t - present_since >= PRESENCE_DELAY and ROBOTS <= ready:", "if ROBOTS <= ready:")
open(p, "w").write(s)'

mutate "6-axis grasps the lower half only (original slip bug)" handover.test.json '
p = "tools/cell_layout.py"; s = open(p).read()
s = s.replace("R_HAND = CUBE_AT_H - 0.0015", "R_HAND = CUBE_AT_H - 0.015")
open(p, "w").write(s)'

mutate "conveyor keeps driving the cube at the end stop" transfer.test.json '
p = "controllers/conveyor/conveyor.py"; s = open(p).read()
s = s.replace("Y_STOP + 0.006 < p[1]", "Y_STOP + 0.002 < p[1]").replace("and abs(p[2] - (BELT_TOP + 0.02)) < 0.003)", "and BELT_TOP + 0.01 < p[2] < BELT_TOP + 0.03)")
open(p, "w").write(s)
p = "tools/build_world.py"; s = open(p).read()
s = s.replace(" bounce 0 }}", " }}")
open(p, "w").write(s)'

mutate "grippers do not squeeze" transfer.test.json '
p = "tools/cell_layout.py"; s = open(p).read()
s = s.replace("OPEN, CLOSE = 0.0, 0.0105", "OPEN, CLOSE = 0.0, 0.0")
open(p, "w").write(s)'

echo
[ $missed -eq 0 ] && echo "ALL MUTATIONS CAUGHT" || echo "$missed MUTATION(S) NOT CAUGHT"
exit $missed
