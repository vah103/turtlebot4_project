#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

python3 mapex_hospital_research/analysis/summarize_runs.py

echo "Basic run summary complete. Detailed stage/pipeline/oracle analysis will be added after logging schema is finalized."
