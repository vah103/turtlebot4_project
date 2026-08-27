#!/usr/bin/env python3
"""Launch Hospital + SLAM + Nav2 + full Stage-2 nearest baseline logging.

The full runner records per-run baseline data under:
  mapex_hospital_research/experiments/nearest/<run_id>/
"""

import os

from launch import LaunchDescription
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


def generate_launch_description() -> LaunchDescription:
    research_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    common_launch = os.path.join(
        research_dir,
        "launch",
        "hospital_simulation.launch.py",
    )
    full_script = os.path.join(
        research_dir,
        "scripts",
        "mapex_nearest_full.py",
    )

    use_rviz = LaunchConfiguration("use_rviz")
    headless = LaunchConfiguration("headless")
    use_sim_time = LaunchConfiguration("use_sim_time")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    yaw = LaunchConfiguration("yaw")
    slam_delay_sec = LaunchConfiguration("slam_delay_sec")
    nav2_delay_sec = LaunchConfiguration("nav2_delay_sec")
    nearest_delay_sec = LaunchConfiguration("nearest_delay_sec")
    run_id = LaunchConfiguration("run_id")

    hospital_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(common_launch),
        launch_arguments={
            "use_rviz": use_rviz,
            "headless": headless,
            "use_sim_time": use_sim_time,
            "x_pose": x_pose,
            "y_pose": y_pose,
            "yaw": yaw,
            "slam_delay_sec": slam_delay_sec,
            "nav2_delay_sec": nav2_delay_sec,
        }.items(),
    )

    nearest_full = ExecuteProcess(
        cmd=[
            "/usr/bin/python3",
            full_script,
            "--run-id",
            run_id,
            "--ros-args",
            "-p",
            ["use_sim_time:=", use_sim_time],
        ],
        output="screen",
    )

    shutdown_when_full_runner_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=nearest_full,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="Nearest full baseline completed or stopped"
                    )
                )
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("run_id", default_value="nearest_001"),
            DeclareLaunchArgument("use_rviz", default_value="True"),
            DeclareLaunchArgument("headless", default_value="False"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("x_pose", default_value="0.0"),
            DeclareLaunchArgument("y_pose", default_value="12.0"),
            DeclareLaunchArgument("yaw", default_value="-1.57"),
            DeclareLaunchArgument("slam_delay_sec", default_value="10.0"),
            DeclareLaunchArgument("nav2_delay_sec", default_value="20.0"),
            DeclareLaunchArgument(
                "nearest_delay_sec",
                default_value="25.0",
                description="Seconds after startup before the full nearest baseline starts.",
            ),
            hospital_stack,
            shutdown_when_full_runner_exits,
            TimerAction(period=nearest_delay_sec, actions=[nearest_full]),
        ]
    )
