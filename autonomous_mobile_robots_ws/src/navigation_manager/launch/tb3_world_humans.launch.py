#!/usr/bin/env python3
"""Launch the TurtleBot3 world WITH walking human actors (Phase 4).

Identical to turtlebot3_gazebo/turtlebot3_world.launch.py except the world
file is our package's copy containing <actor> blocks.

Usage:
    export TURTLEBOT3_MODEL=waffle
    ros2 launch navigation_manager tb3_world_humans.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    tb3_launch_dir = os.path.join(tb3_gazebo_dir, 'launch')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    # Our world copy with the walking actor(s)
    world = os.path.join(
        get_package_share_directory('navigation_manager'),
        'worlds',
        'tb3_world_humans.world'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')

    declare_x = DeclareLaunchArgument('x_pose', default_value='-2.0')
    declare_y = DeclareLaunchArgument('y_pose', default_value='-0.5')

    # Gazebo server with our world
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r -s -v2 ', world],
            'on_exit_shutdown': 'true',
        }.items()
    )

    # Gazebo GUI client
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g -v2 '}.items()
    )

    # Reuse TB3's own robot_state_publisher and spawner
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_launch_dir, 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_launch_dir, 'spawn_turtlebot3.launch.py')),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_x)
    ld.add_action(declare_y)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_turtlebot)
    return ld
