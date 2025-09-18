#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory for your world
    husky_pkg_dir = get_package_share_directory('husky_footplcmt')
    world_path = os.path.join(husky_pkg_dir, 'gazebo_worlds', 'balance_world.world')
    
    print(f"Loading world from: {world_path}")
    
    # Use champ_config's gazebo.launch.py
    champ_config_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('champ_config'),
            'launch',
            'gazebo.launch.py'
        ]),
        launch_arguments={
            'world': world_path,  # Your balance_world
            'use_sim_time': 'true',
            'gui': 'true',
            'rviz': 'true',  # Enable RViz now
            'robot_name': 'champ',
            'world_init_x': '0.0',
            'world_init_y': '0.0',
            'world_init_z': '2.5',
            'world_init_heading': '0.0',
        }.items(),
    )
    
    return LaunchDescription([
        champ_config_launch,
    ])