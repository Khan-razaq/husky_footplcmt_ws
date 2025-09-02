from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('husky_footplcmt')

    world_path = os.path.join(pkg_share, 'gazebo_worlds', 'block_pipe_camera.world')
    model_path = os.path.join(pkg_share, 'gazebo_worlds', 'models')
    robot_path = os.path.join(model_path, 'block_with_camera', 'model.sdf')
    controller_config_path = os.path.join(pkg_share, 'config', 'camera_joint_controllers.yaml')

    # Get full SDF as a string for robot_description
    robot_description = ParameterValue(
        Command(f'gz sdf -p {robot_path}'),
        value_type=str
    )

    return LaunchDescription([
        # Launch Gazebo with env var for GAZEBO_MODEL_PATH
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path],
            output='screen',
            env={'GAZEBO_MODEL_PATH': model_path, **os.environ}
        ),

        # Static TFs
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '0', '1.5', '0', '0', '0', '1.5708',
                'camera_link', 'camera_depth_optical_frame'
            ],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '0', '0', '0.75', '0', '0', '0',
                'map', 'camera_link'
            ],
            output='screen'
        ),

        # Publish robot_description (required for ros2_control)
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     parameters=[{'robot_description': robot_description}],
        #     output='screen'
        # ),

        # Launch RViz2
        ExecuteProcess(
            cmd=['rviz2'],
            output='screen'
        ),

        # Delay controller spawners to wait for Gazebo & robot_state_publisher
        # TimerAction(
        #     period=5.0,
        #     actions=[
        #         Node(
        #             package='controller_manager',
        #             executable='spawner',
        #             arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        #             output='screen'
        #         ),
        #         Node(
        #             package='controller_manager',
        #             executable='spawner',
        #             arguments=['camera_joint_position_controller', '--controller-manager', '/controller_manager'],
        #             output='screen'
        #         ),
        #     ]
        # )
    ])