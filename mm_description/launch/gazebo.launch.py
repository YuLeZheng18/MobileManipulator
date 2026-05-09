import os
import time

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_dir = get_package_share_directory('mm_description')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    default_model_path = os.path.join(package_dir, 'urdf', 'omni_base.urdf.xacro')
    default_world_path = os.path.join(package_dir, 'world', 'room.word')  # 恢复使用room.world文件

    model_arg = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='URDF/Xacro model path',
    )

    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str,
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}, {'use_sim_time': True}],
        output='screen',
    )

    gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': default_world_path}.items(),  # 使用room.world文件
    )

    cmd_vel_smoother_node = launch_ros.actions.Node(
        package='mm_description',
        executable='cmd_vel_smoother.py',
        parameters=[{
            'use_sim_time': True,
            'linear_acceleration': 0.25,
            'angular_acceleration': 0.6,
            'command_timeout': 0.3,
        }],
        output='screen',
    )

    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'mm_omni_base', '-x', '0.0', '-y', '0.0', '-z', '0.2'],
        output='screen',
    )

    return launch.LaunchDescription([
        model_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        gazebo,
        cmd_vel_smoother_node,
        spawn_entity,
    ])
