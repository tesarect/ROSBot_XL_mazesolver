#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Get package share directory
    pkg_share = get_package_share_directory('pid_maze_solver')

    # Path to python script inside scripts/
    script_path = os.path.join(pkg_share, 'scripts', 'scan_marker_all_dir_wheels.py')

    # Path to RViz config file
    rviz_config = os.path.join(pkg_share, 'rviz', 'default.rviz')

    # Launch Python script
    python_script = ExecuteProcess(
        cmd=['python3', script_path],
        output='screen'
    )

    # Launch RViz with config
    rviz = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        python_script,
        rviz
    ])