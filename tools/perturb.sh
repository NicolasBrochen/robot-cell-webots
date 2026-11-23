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
  # sub(path, old, new): every mutation must really change the code, otherwise it is reported as not applied
  local pre='def sub(p, a, b):
    s = open(p).read()
    assert a in s, "pattern not found in " + p + ": " + a
    open(p, "w").write(s.replace(a, b))
'
  (cd "$work" && "$PY" -c "$pre$code") || { echo "== mutation: $label"; echo "  NOT APPLIED (pattern not found)"; missed=$((missed + 1)); rm -rf "$work"; return; }
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
sub("controllers/conveyor/conveyor.py", "if present_since is not None and t - present_since >= PRESENCE_DELAY and ROBOTS <= ready:", "if ROBOTS <= ready:")'

mutate "UR5e never closes at the hand-over (hand-over handshake broken)" transfer.test.json '
sub("tools/gen_motions.py", "(6.4, *H, U_HAND, PI, G_CLOSE),\n    (8.0, *H, U_HAND, PI, G_CLOSE), (8.8, *H, U_PRE, PI, G_CLOSE), (11.6, *O, U_DROP, 0, G_CLOSE),",
    "(6.4, *H, U_HAND, PI, G_OPEN),\n    (8.0, *H, U_HAND, PI, G_OPEN), (8.8, *H, U_PRE, PI, G_OPEN), (11.6, *O, U_DROP, 0, G_OPEN),")'

mutate "no spinning friction on the Robotiq pads (cube pivots out during the flip)" transfer.test.json '
sub("tools/build_world.py", " rollingFriction 0 0 0.01", "")'

mutate "conveyor no longer keeps the cube against the end stop (cube off-centre in the grippers)" handover.test.json '
sub("controllers/conveyor/conveyor.py", "c.setVelocity([0, -0.02, v[2], 0, 0, 0])", "pass")'

mutate "grippers do not squeeze (stop half open)" transfer.test.json '
sub("tools/cell_layout.py", "G_OPEN, G_HALF, G_CLOSE = 0.0, 0.3, 0.8", "G_OPEN, G_HALF, G_CLOSE = 0.0, 0.3, 0.3")'

echo
[ $missed -eq 0 ] && echo "ALL MUTATIONS CAUGHT" || echo "$missed MUTATION(S) NOT CAUGHT"
exit $missed
