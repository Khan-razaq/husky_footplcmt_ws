import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('husky_footplcmt')
    
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
    )
    
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
    
    # Updated to use your custom world file
    world_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py']),
        launch_arguments={
            'world': os.path.join(pkg_dir, 'gazebo_worlds', 'balance_world.world'),
            'gui': LaunchConfiguration('gui'),
            'pause': 'true',
            'verbose': 'true',
        }.items(),
    )
    
    # robot_state_publisher to publish /robot_description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['cat ', PathJoinSubstitution([
                    FindPackageShare(LaunchConfiguration('urdf_package')),
                    LaunchConfiguration('urdf_package_path')
                ])]),
                value_type=str
            )
        }]
    )
    
    # push robot_description to factory and spawn robot in gazebo
    urdf_spawner_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        arguments=['-topic', '/robot_description', '-entity', 'pole_box_robot', '-z', '0.1', '-unpause'],
        output='screen',
    )
    
    return LaunchDescription([
        gui_arg,
        package_arg,
        model_arg,
        world_launch,
        robot_state_publisher_node,
        urdf_spawner_node,
    ])