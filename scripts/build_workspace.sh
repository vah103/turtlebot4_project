#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${REPO_ROOT}/ros2_ws"

# shellcheck disable=SC1091
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "${WORKSPACE}"
colcon build --symlink-install
