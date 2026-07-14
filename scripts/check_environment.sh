#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ERROR: Không tìm thấy /opt/ros/jazzy/setup.bash" >&2
  exit 1
fi

# shellcheck disable=SC1091
set +u
source /opt/ros/jazzy/setup.bash
set -u

for command_name in ros2 colcon python3; do
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "ERROR: Thiếu lệnh ${command_name}" >&2
    exit 1
  fi
done

python3 - <<'PY'
from importlib.util import find_spec

required = ("rclpy", "sensor_msgs", "nav_msgs", "launch", "launch_ros")
missing = [name for name in required if find_spec(name) is None]
if missing:
    raise SystemExit("ERROR: Thiếu Python module ROS 2: " + ", ".join(missing))
print("Các Python module ROS 2 bắt buộc đều sẵn sàng.")
PY

echo "ROS_DISTRO=${ROS_DISTRO:-<chưa đặt>}"
echo "Repository: ${REPO_ROOT}"
echo "Kiểm tra môi trường thành công."
