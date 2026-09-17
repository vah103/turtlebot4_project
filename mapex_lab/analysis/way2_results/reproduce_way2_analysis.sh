#!/usr/bin/env bash
set -u

# Reproduce and persist the historical Way2 development analyses.
#
# Run from anywhere inside the repository:
#   bash mapex_lab/analysis/way2_results/reproduce_way2_analysis.sh
#
# The script deliberately captures stdout+stderr verbatim instead of parsing it,
# so no analysis result is silently lost or transformed. A non-zero exit from one
# analysis is recorded and does not prevent the remaining analyses from running.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAPEX_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ANALYSIS_DIR="$SCRIPT_DIR/reproduced_logs"
PYTHON_BIN="${PYTHON_BIN:-python3}"

mkdir -p "$ANALYSIS_DIR"
cd "$MAPEX_ROOT/scripts"

scripts=(
  analyze_way2_utility.py
  audit_way2_gain.py
  compare_way2_gain_definitions.py
  audit_way2_cost.py
  analyze_way2_lambda.py
  inspect_way2_lambda_focus.py
  inspect_way2_rebounds.py
  compare_way2_guard_k.py
  compare_way2_guard_smoothing.py
  evaluate_way2_candidate_quality.py
  inspect_way2_quality_failures.py
  inspect_way2_hospital_quality_trajectory.py
  audit_way2_observed_quality.py
  sweep_way2_lambda_quality.py
  audit_way2_visible_unknown_guard.py
  sweep_way2_visible_unknown_guard.py
  inspect_way2_guard_candidates.py
  inspect_way2_mpx009_quality_trajectory.py
  evaluate_way2_adaptive_confirmation.py
  sweep_way2_adaptive_candidate_count.py
  sweep_way2_adaptive_rule_neighborhood.py
)

manifest="$ANALYSIS_DIR/manifest.csv"
printf 'script,exit_code,log_file\n' > "$manifest"

for script in "${scripts[@]}"; do
  log="$ANALYSIS_DIR/${script%.py}.log"
  echo "============================================================"
  echo "Running $script"
  echo "Output: $log"
  echo "============================================================"

  set +e
  "$PYTHON_BIN" "$script" >"$log" 2>&1
  rc=$?
  set -e

  printf '%s,%s,%s\n' "$script" "$rc" "$(basename "$log")" >> "$manifest"
  if [[ $rc -eq 0 ]]; then
    echo "OK: $script"
  else
    echo "FAILED ($rc): $script -- see $log"
  fi
done

{
  echo "generated_at=$(date --iso-8601=seconds 2>/dev/null || date)"
  echo "git_commit=$(git -C "$MAPEX_ROOT/.." rev-parse HEAD 2>/dev/null || true)"
  echo "python=$($PYTHON_BIN --version 2>&1)"
} > "$ANALYSIS_DIR/provenance.txt"

echo
echo "Saved Way2 analysis logs to: $ANALYSIS_DIR"
echo "Manifest: $manifest"
echo "After review, commit reproduced_logs/ so the exact sweep output is preserved."
