import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('mm_description')
    default_model_path = os.path.join(package_dir, 'urdf', 'omni_base.urdf.xacro')
    default_rviz_config_path = os.path.join(package_dir, 'config', 'rviz', 'display_model.rviz')

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
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path],
    )

    return launch.LaunchDescription([
        model_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ])
