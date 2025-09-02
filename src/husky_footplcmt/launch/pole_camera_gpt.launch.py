import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('husky_footplcmt')

    urdf_file = os.path.join(pkg_share, 'urdf', 'pole_camera.urdf')
    yaml_file = os.path.join(pkg_share, 'config', 'pole_camera_controllers.yaml')

    robot_description = ParameterValue(
        Command(['cat ', urdf_file]),
        value_type=str
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={'verbose': 'true', 'pause': 'false'}.items()
    )

    # ROS2 Control Node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description},
                    yaml_file],
        output='screen'
    )

    # Spawn robot after Gazebo has started
    spawn_entity_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-topic', 'robot_description', '-entity', 'pole_box_robot'],
                output='screen'
            )
        ]
    )

    # Spawn controllers after robot and ros2_control_node are up
    spawn_controllers_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner.py',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen'
            ),
            Node(
                package='controller_manager',
                executable='spawner.py',
                arguments=['pole_position_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_launch,
        ros2_control_node,
        spawn_entity_node,
        spawn_controllers_node
    ])