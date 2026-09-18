#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAPEX_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$MAPEX_ROOT/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"

cd "$REPO_ROOT"

"$PYTHON_BIN" "$SCRIPT_DIR/recompute_new_room_metrics_v2.py"

# Only after the run-level migration succeeds, remove v1-derived Way2 log
# archives from the active branch. Git history still preserves them; the
# current tree will contain only regenerated evidence.
rm -rf "$MAPEX_ROOT/analysis/way2_results/historical_logs"
rm -rf "$MAPEX_ROOT/analysis/way2_results/reproduced_logs"
rm -f "$MAPEX_ROOT/analysis/way2_results/archive_manifest.csv"

bash "$MAPEX_ROOT/analysis/way2_results/reproduce_way2_analysis.sh"
"$PYTHON_BIN" "$SCRIPT_DIR/refresh_way2_v2_evidence.py"

echo
echo "Migration finished. Active New Room derived metrics are v2."
echo "Raw experiment evidence was retained."
echo
echo "Review changes:"
git status --short

echo
echo "After review, publish with:"
echo "  git add -A mapex_lab"
echo "  git commit -m 'Backfill New Room metrics with frame-correct v2 evaluation'"
echo "  git push"
