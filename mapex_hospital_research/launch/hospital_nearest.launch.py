#!/usr/bin/env python3
"""Hospital v1 nearest baseline using the official MapEx frontier policy.

Run directly from the repository after sourcing ROS/workspace setup:

    /usr/bin/python3 mapex_hospital_research/launch/hospital_nearest.launch.py

The frontier policy comes from castacks/MapEx commit
53636bd1c79153acc3c74a532837d78c926bae5e. Only the original grid-simulator
A* execution layer is replaced by Nav2 for TurtleBot4 Hospital execution.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


WORKSPACE = Path(__file__).resolve().parents[1]
RECORDER = WORKSPACE / "scripts" / "research_recorder_safe.py"
MAPEX_NEAREST = WORKSPACE / "scripts" / "mapex_nearest_ros_hospital.py"
EXPLORATION_START_DELAY_S = 30.0


def generate_launch_description() -> LaunchDescription:
    package_dir = get_package_share_directory("frontier_exploration")
    frontier_config = os.path.join(package_dir, "config", "frontier.yaml")

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

    manager = Node(
        package="frontier_exploration",
        executable="exploration_manager",
        name="exploration_manager",
        output="screen",
        parameters=[
            frontier_config,
            {"use_sim_time": True, "enable_navigation": True},
        ],
    )

    mapex_nearest = ExecuteProcess(
        cmd=[sys.executable, str(MAPEX_NEAREST)],
        output="screen",
    )

    shutdown_when_policy_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=mapex_nearest,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="MapEx nearest policy completed or stopped"
                    )
                )
            ],
        )
    )

    exploration = TimerAction(
        period=EXPLORATION_START_DELAY_S,
        actions=[manager, mapex_nearest],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("run_id", default_value="nearest_pilot_004"),
            DeclareLaunchArgument("use_rviz", default_value="True"),
            DeclareLaunchArgument("headless", default_value="False"),
            recorder,
            stack,
            shutdown_when_policy_exits,
            exploration,
        ]
    )


def main() -> int:
    service = LaunchService(argv=sys.argv[1:])
    service.include_launch_description(generate_launch_description())
    return service.run()


if __name__ == "__main__":
    raise SystemExit(main())
