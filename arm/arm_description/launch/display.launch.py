from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_name = 'arm_description'
    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)

    default_model_path = PathJoinSubstitution([pkg_share, 'urdf', 'arm_description.urdf'])

    declared_arguments = [
        DeclareLaunchArgument(
            'model',
            default_value=default_model_path,
            description='Path to robot URDF file'
        ),
    ]

    robot_description_content = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription(declared_arguments + [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])