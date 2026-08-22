# Copyright (C) 2023 Open Source Robotics Foundation
# Copyright (C) 2023 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch TurtleBot4 simulation after the temporary world SDF is ready."""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    sim_dir = get_package_share_directory('nav2_minimal_tb4_sim')
    desc_dir = get_package_share_directory('nav2_minimal_tb4_description')
    launch_dir = os.path.join(sim_dir, 'launch')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    headless = LaunchConfiguration('headless')
    gz_ip = LaunchConfiguration('gz_ip')
    world = LaunchConfiguration('world')
    pose = {
        'x': LaunchConfiguration('x_pose', default='-8.00'),
        'y': LaunchConfiguration('y_pose', default='0.00'),
        'z': LaunchConfiguration('z_pose', default='0.01'),
        'R': LaunchConfiguration('roll', default='0.00'),
        'P': LaunchConfiguration('pitch', default='0.00'),
        'Y': LaunchConfiguration('yaw', default='0.00'),
    }
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    declarations = [
        DeclareLaunchArgument(
            'namespace', default_value='', description='Top-level namespace'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true',
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=os.path.join(desc_dir, 'rviz', 'config.rviz'),
            description='Full path to the RViz config file to use',
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='True', description='Whether to start RViz'
        ),
        DeclareLaunchArgument(
            'use_simulator',
            default_value='True',
            description='Whether to start the simulator',
        ),
        DeclareLaunchArgument(
            'use_robot_state_pub',
            default_value='True',
            description='Whether to start robot_state_publisher',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='False',
            description='Whether to skip the Gazebo client',
        ),
        DeclareLaunchArgument(
            'gz_ip',
            default_value='127.0.0.1',
            description='Gazebo Transport interface for local simulation',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(sim_dir, 'worlds', 'depot.sdf'),
            description='Full path to the world model',
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='nav2_turtlebot4',
            description='Name of the robot',
        ),
        DeclareLaunchArgument(
            'robot_sdf',
            default_value=os.path.join(
                desc_dir, 'urdf', 'standard', 'turtlebot4.urdf.xacro'
            ),
            description='Full path to the robot description xacro',
        ),
    ]

    robot_state_publisher = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'robot_description': Command(['xacro', ' ', robot_sdf]),
            }
        ],
        remappings=remappings,
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=remappings,
    )

    world_sdf = tempfile.mktemp(prefix='nav2_', suffix='.sdf')
    world_sdf_xacro = ExecuteProcess(
        cmd=['xacro', '-o', world_sdf, ['headless:=', headless], world],
        output='screen',
    )

    # Call Gazebo directly. The ros_gz_sim wrapper rebuilds the process
    # environment and can stall before the world services appear on some
    # Jazzy installations, while the equivalent direct command works.
    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', '-s', world_sdf],
        output='screen',
        condition=IfCondition(use_simulator),
    )

    gazebo_client = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-g'],
        output='screen',
        condition=IfCondition(
            PythonExpression([use_simulator, ' and not ', headless])
        ),
    )

    # Register before starting xacro so a very fast process exit cannot be
    # missed. This is the key difference from the upstream all-in-one launch.
    start_gazebo_after_world = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=world_sdf_xacro,
            on_exit=[gazebo_server, gazebo_client],
        )
    )

    spawn_and_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'spawn_tb4.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'use_simulator': use_simulator,
            'use_sim_time': use_sim_time,
            'robot_name': robot_name,
            'robot_sdf': robot_sdf,
            'x_pose': pose['x'],
            'y_pose': pose['y'],
            'z_pose': pose['z'],
            'roll': pose['R'],
            'pitch': pose['P'],
            'yaw': pose['Y'],
        }.items(),
    )

    remove_temp_sdf = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[
                OpaqueFunction(
                    function=lambda _: (
                        os.remove(world_sdf) if os.path.exists(world_sdf) else None
                    )
                )
            ]
        )
    )

    resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(sim_dir, 'worlds')
    )
    local_gazebo_transport = SetEnvironmentVariable('GZ_IP', gz_ip)

    launch_description = LaunchDescription(declarations)
    launch_description.add_action(local_gazebo_transport)
    launch_description.add_action(resource_path)
    launch_description.add_action(start_gazebo_after_world)
    launch_description.add_action(world_sdf_xacro)
    launch_description.add_action(remove_temp_sdf)
    launch_description.add_action(spawn_and_bridge)
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(rviz)
    return launch_description
