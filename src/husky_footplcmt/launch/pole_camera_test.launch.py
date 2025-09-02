import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('husky_footplcmt')
    
    package_arg = DeclareLaunchArgument(
        'urdf_package',
        description='The package where the robot description is located',
        default_value='husky_footplcmt'
    )
    
    model_arg = DeclareLaunchArgument(
        'urdf_package_path',
        description='The path to the robot description relative to the package root',
        default_value='urdf/pole_camera_test.urdf'
    )
    
    gazebo_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('husky_footplcmt'), 'launch', 'gazebo.launch.py']),
        launch_arguments={
            'urdf_package': LaunchConfiguration('urdf_package'),
            'urdf_package_path': LaunchConfiguration('urdf_package_path')
        }.items(),
    )
    
    # Spawn joint_state_broadcaster after a delay
    joint_state_broadcaster_spawner = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen',
            )
        ]
    )
    
    # Spawn pole_position_controller after joint_state_broadcaster
    pole_position_controller_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['pole_position_controller'],
                output='screen',
            )
        ]
    )
    
    # Static transform to fix camera frame orientation for RViz
    camera_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_transform',
        arguments=['0.1', '0', '0', '-1.5708', '0', '-1.5708', 'box_link', 'camera_optical_frame']
    )
    
    # Launch RViz with saved configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz2', 'camera_view.rviz')],
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        package_arg,
        model_arg,
        gazebo_launch,
        joint_state_broadcaster_spawner,
        pole_position_controller_spawner,
        camera_transform,
        rviz_node,
    ])