from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_name = 'arm_description'
    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)

    default_model_path = PathJoinSubstitution([pkg_share, 'urdf', 'arm_description.urdf'])

    gazebo_ros_share = FindPackageShare(package='gazebo_ros').find('gazebo_ros')

    start_gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver',
            '-s',
            'libgazebo_ros_init.so',
            '-s',
            'libgazebo_ros_factory.so',
            PathJoinSubstitution([gazebo_ros_share, 'launch', 'empty_world.launch'])
        ],
        output='screen'
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-entity', 'arm_description',
            '-file', LaunchConfiguration('model'),
            '-urdf'
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': LaunchConfiguration('model')}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=default_model_path),
        start_gazebo_server,
        robot_state_publisher_node,
        spawn_entity_node
    ])