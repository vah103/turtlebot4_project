#!/usr/bin/env python3
"""Hospital v1 Nearest-Frontier baseline + research recorder.

Run directly from the repository after sourcing ROS/workspace setup:

    python3 mapex_hospital_research/launch/hospital_nearest.launch.py

The default run id is `nearest_pilot_001`. A run directory is never overwritten.
UI-only arguments may be appended, e.g. `use_rviz:=false headless:=true`.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


WORKSPACE = Path(__file__).resolve().parents[1]
RECORDER = WORKSPACE / "scripts" / "research_recorder.py"
EXPLORATION_START_DELAY_S = 30.0


def generate_launch_description() -> LaunchDescription:
    package_dir = get_package_share_directory("frontier_exploration")

    use_rviz = LaunchConfiguration("use_rviz")
    headless = LaunchConfiguration("headless")
    run_id = LaunchConfiguration("run_id")

    recorder = ExecuteProcess(
        cmd=[
            sys.executable,
            str(RECORDER),
            "--method",
            "nearest",
            "--run-id",
            run_id,
        ],
        output="screen",
    )

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, "launch", "hospital_flat_stack.launch.py")
        ),
        launch_arguments={
            "use_rviz": use_rviz,
            "headless": headless,
            "use_sim_time": "true",
            "x_pose": "0.0",
            "y_pose": "12.0",
            "yaw": "-1.57",
            "slam_delay_sec": "10.0",
            "nav2_delay_sec": "20.0",
        }.items(),
    )

    nearest = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, "launch", "frontier_autonomy.launch.py")
        ),
        launch_arguments={
            "enable_navigation": "true",
            "enable_sim_twist_adapter": "false",
            "use_sim_time": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("run_id", default_value="nearest_pilot_001"),
            DeclareLaunchArgument("use_rviz", default_value="True"),
            DeclareLaunchArgument("headless", default_value="False"),
            recorder,
            stack,
            TimerAction(period=EXPLORATION_START_DELAY_S, actions=[nearest]),
        ]
    )


def main() -> int:
    service = LaunchService(argv=sys.argv[1:])
    service.include_launch_description(generate_launch_description())
    return service.run()


if __name__ == "__main__":
    raise SystemExit(main())
