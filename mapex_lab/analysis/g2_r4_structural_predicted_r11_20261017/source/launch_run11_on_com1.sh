#!/usr/bin/env bash
# One-shot, fail-closed *native COM1 2D* run of G2-R11.
# Does NOT modify frozen Run10-R2. Does NOT run any other robotics pipeline.
# Execute explicitly on COM1 by a user/operator, never from a CI runner.
set -Eeuo pipefail
umask 022
BASE=/work/com1/mx072_g2_r4_frontier_exit_run10r2_20261017
TARGET=/work/com1/mx072_g2_r4_structural_predicted_run11_20261017
CHECKOUT=/work/com1/mx072_r11_immutable_checkout_20261017
REPO=https://github.com/vah103/turtlebot4_project.git
BRANCH=mx072-g2-r4-structural-predicted-r11-20261017
# PINNED: replace with verified exact commit before running. Do not use an unpinned branch.
EXPECTED_SHA=2b8feae49e19ce197b4760ef2a6db057ef9c0c05
P=/work/conda-envs/mapex-single-audit-com1/bin/python
NATIVE=/work/com1/mapex_single_audit/source/MapEx
test -f "$BASE/guard_g2_r4_run10r2.py"
test -f "$BASE/g2_r4_policy_sidecar.py"
test -x "$P"
test -f "$NATIVE/configs/base.yaml"
test ! -e "$TARGET" || { echo "REFUSE_OVERWRITE_RUN11" >&2; exit 12; }
test ! -e "$CHECKOUT" || { echo "REFUSE_REUSE_R11_CHECKOUT" >&2; exit 13; }
mkdir -p "$CHECKOUT"
git -C "$CHECKOUT" init -q
git -C "$CHECKOUT" remote add origin "$REPO"
git -C "$CHECKOUT" config core.sparseCheckout true
printf '%s\n' '/mapex_lab/analysis/g2_r4_structural_predicted_r11_20261017/' > "$CHECKOUT/.git/info/sparse-checkout"
git -C "$CHECKOUT" fetch -q --depth=1 origin "$EXPECTED_SHA"
git -C "$CHECKOUT" checkout -q --detach "$EXPECTED_SHA"
actual=$(git -C "$CHECKOUT" rev-parse HEAD)
test "$actual" = "$EXPECTED_SHA" || { echo "WRONG_SOURCE_COMMIT"; exit 14; }
PATCH="$CHECKOUT/mapex_lab/analysis/g2_r4_structural_predicted_r11_20261017/source"
test -f "$PATCH/prepare_run11_from_run10r2.py"
echo "R11_STAGE_START $(date -Is)"
"$P" "$PATCH/prepare_run11_from_run10r2.py" \
  --base "$BASE" --patch-dir "$PATCH" --target "$TARGET" --runtime-root "$TARGET"
cd "$TARGET"
export PYTHONPATH="$TARGET:$TARGET/g2src:$NATIVE:$NATIVE/scripts"
echo "R11_UNIT_TEST_START $(date -Is)"
"$P" -m unittest test_r11_structural_arbitration -v \
   |& tee "$TARGET/evidence/r11_unit_tests.log"
echo "R11_FROZEN_SNAPSHOT_REPLAY_START $(date -Is)"
"$P" "$PATCH/replay_r11_on_run10r2.py" \
  --frozen-run "$BASE" \
  --out "$TARGET/evidence/R11_OFF_POLICY_SNAPSHOT_REPLAY.json" \
  |& tee "$TARGET/evidence/r11_replay.log"
"$P" - <<'PY'
import json
from pathlib import Path
p=Path('/work/com1/mx072_g2_r4_structural_predicted_run11_20261017/evidence/R11_OFF_POLICY_SNAPSHOT_REPLAY.json')
j=json.loads(p.read_text())
assert j['local_decisions']>=15,j['local_decisions']
assert j['WFD_goal_violation']==0
assert j['native_run_executed'] is False
for row in j['results']:
    assert row['new_R11_final_executable_goal'] is not None
print('R11_REPLAY_GATE_PASS',j['local_decisions'], 'local decisions')
PY
test ! -e "$TARGET/native_g2_r4_structural_predicted_run11" || { echo "OUTPUT_COLLISION"; exit 17; }
echo "R11_NATIVE2D_LAUNCH $(date -Is)"
# Guard continuously saves native status/terminal receipt. This runs in the
# foreground and can be interrupted safely by SIGINT; not a background promise.
"$P" -u "$TARGET/guard_g2_r4_run11.py" \
  |& tee "$TARGET/evidence/r11_guard_console.log"
"$P" - <<'PY'
import json
from pathlib import Path
r=Path('/work/com1/mx072_g2_r4_structural_predicted_run11_20261017/evidence/guard_run11_structural_predicted/terminal_receipt.json')
receipt=json.loads(r.read_text())
print('R11_NATIVE_TERMINAL',receipt.get('outcome'), 'steps',receipt.get('completed_tick_records'),
      'elapsed_s',receipt.get('elapsed_s'))
assert receipt['outcome']=='SOURCE_TERMINAL',receipt
assert receipt['returncode']==0,receipt
PY
echo "R11_NATIVE_SUCCESS $(date -Is)"
