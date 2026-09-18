#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAPEX_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$MAPEX_ROOT/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"

cd "$REPO_ROOT"

"$PYTHON_BIN" "$SCRIPT_DIR/recompute_new_room_metrics_v2.py"

echo
echo "Migration finished for the 20 baseline New Room runs:"
echo "  NF    nf_001..nf_010"
echo "  MapEx mpx_001..mpx_010"
echo "Way2 development/validation evidence is intentionally left untouched."
echo "Raw experiment evidence was retained."
echo
echo "Review changes:"
git status --short

echo
echo "After review, publish with:"
echo "  git add -A mapex_lab"
echo "  git commit -m 'Backfill New Room metrics with frame-correct v2 evaluation'"
echo "  git push"
