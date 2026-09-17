#!/usr/bin/env bash
set -u

# Reproduce and persist the historical Way2 development analyses.
#
# Run from anywhere inside the repository:
#   bash mapex_lab/analysis/way2_results/reproduce_way2_analysis.sh
#
# This runner is intentionally tied to the exact development cohort used during
# Way2 threshold selection:
#   New Room: mpx_001 ... mpx_015
#   Hospital: hpx_001, hpx_002
#
# Post-freeze Way2 online runs (mpx_w2_*, hpx_w2_*) are NOT inputs here and must
# not be used to retune the frozen thresholds.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAPEX_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ANALYSIS_DIR="$SCRIPT_DIR/reproduced_logs"
PYTHON_BIN="${PYTHON_BIN:-python3}"

DEV_RUNS=(
  mpx_001 mpx_002 mpx_003 mpx_004 mpx_005
  mpx_006 mpx_007 mpx_008 mpx_009 mpx_010
  mpx_011 mpx_012 mpx_013 mpx_014 mpx_015
  hpx_001 hpx_002
)

# Fail before overwriting the archive if the exact development cohort is not
# available locally. Running on a reduced subset would not reproduce the
# threshold-selection evidence and could be misleading.
missing_runs=()
for run in "${DEV_RUNS[@]}"; do
  if [[ ! -d "$MAPEX_ROOT/experiments/mapex/$run" ]]; then
    missing_runs+=("$run")
  fi
done

if (( ${#missing_runs[@]} > 0 )); then
  echo "ERROR: cannot reproduce the frozen Way2 development analysis." >&2
  echo "Missing required run directories:" >&2
  printf '  - %s\n' "${missing_runs[@]}" >&2
  echo >&2
  echo "Expected cohort: mpx_001..mpx_015 + hpx_001..hpx_002" >&2
  echo "Do not substitute post-freeze mpx_w2_*/hpx_w2_* runs." >&2
  exit 3
fi

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

# These scripts require the run list as a positional argument. Later scripts
# either consume their generated artifacts or already default to the same
# 17-run development cohort.
requires_run_args() {
  case "$1" in
    analyze_way2_utility.py|\
    audit_way2_gain.py|\
    compare_way2_gain_definitions.py|\
    audit_way2_cost.py|\
    analyze_way2_lambda.py)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

manifest="$ANALYSIS_DIR/manifest.csv"
printf 'script,exit_code,log_file\n' > "$manifest"
failures=0

for script in "${scripts[@]}"; do
  log="$ANALYSIS_DIR/${script%.py}.log"
  echo "============================================================"
  echo "Running $script"
  echo "Output: $log"
  echo "============================================================"

  if requires_run_args "$script"; then
    cmd=("$PYTHON_BIN" "$script" "${DEV_RUNS[@]}")
  else
    cmd=("$PYTHON_BIN" "$script")
  fi

  set +e
  "${cmd[@]}" >"$log" 2>&1
  rc=$?
  set -e

  printf '%s,%s,%s\n' "$script" "$rc" "$(basename "$log")" >> "$manifest"
  if [[ $rc -eq 0 ]]; then
    echo "OK: $script"
  else
    echo "FAILED ($rc): $script -- see $log"
    failures=$((failures + 1))
  fi
done

{
  echo "generated_at=$(date --iso-8601=seconds 2>/dev/null || date)"
  echo "git_commit=$(git -C "$MAPEX_ROOT/.." rev-parse HEAD 2>/dev/null || true)"
  echo "python=$($PYTHON_BIN --version 2>&1)"
  echo "development_runs=${DEV_RUNS[*]}"
  echo "failed_scripts=$failures"
} > "$ANALYSIS_DIR/provenance.txt"

echo
echo "Saved Way2 analysis logs to: $ANALYSIS_DIR"
echo "Manifest: $manifest"
echo "Failed scripts: $failures/${#scripts[@]}"

if (( failures > 0 )); then
  echo "Archive is incomplete. Fix the failed scripts before committing reproduced_logs/." >&2
  exit 1
fi

echo "All Way2 development analyses reproduced successfully."
echo "Now review and commit reproduced_logs/ so the exact sweep output is preserved."
