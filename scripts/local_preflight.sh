#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${REPO_ROOT}/ros2_ws"

"${REPO_ROOT}/scripts/check_environment.sh"
"${REPO_ROOT}/scripts/build_workspace.sh"

# shellcheck disable=SC1091
set +u
source /opt/ros/jazzy/setup.bash
set -u
# shellcheck disable=SC1091
set +u
source "${WORKSPACE}/install/setup.bash"
set -u

cd "${WORKSPACE}"
python3 -m compileall -q src/tb4_project_tools
colcon test --packages-select tb4_project_tools
colcon test-result --verbose

echo "Local preflight thành công; không cần kết nối robot."
