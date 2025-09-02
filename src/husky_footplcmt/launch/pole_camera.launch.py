import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory('husky_footplcmt')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Paths to files
    urdf_file = os.path.join(pkg_share, 'urdf', 'pole_camera.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'empty.sdf')  # Optional custom world
    controllers_file = os.path.join(pkg_share, 'config', 'pole_camera_controllers.yaml')
    
    # Read URDF
    with open(urdf_file, 'r') as file:
        robot_description_content = file.read()
    
    robot_description = {'robot_description': robot_description_content}
    
    # Launch Gazebo Harmonic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r -v 4 empty.sdf',  # -r for running, -v 4 for verbosity
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )
    
    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        arguments=[
            '-name', 'pole_box_robot',
            '-topic', 'robot_description',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # Bridge between Gazebo and ROS2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        output='screen'
    )
    
    # Controller Manager (start separately)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_file],
        output='screen'
    )
    
    # Load and start controllers
    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
                output='screen'
            )
        ]
    )
    
    pole_position_controller_spawner = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['pole_position_controller', '-c', '/controller_manager'],
                output='screen'
            )
        ]
    )
    
    # RViz (optional)
    rviz_config = os.path.join(pkg_share, 'config', 'pole_camera.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        bridge,
        TimerAction(period=2.0, actions=[spawn_entity]),
        TimerAction(period=3.0, actions=[controller_manager]),
        joint_state_broadcaster_spawner,
        pole_position_controller_spawner,
        TimerAction(period=7.0, actions=[rviz])
    ])