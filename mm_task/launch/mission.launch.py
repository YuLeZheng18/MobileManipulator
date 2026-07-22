"""mm_task 状态机 launch: 只起 mission_manager.

Gazebo / MoveIt / grasp / mock 感知 / Nav2 / lane_navigator 由各自既有 launch 分别起
(见 src/docs/system_architecture.md §7.2 与 M5 验证). 本 launch 是最后一环, 顶层编排.
"""
import os

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    default_mission = os.path.join(
        get_package_share_directory('mm_task'), 'config', 'mission.yaml')

    mission_file_arg = DeclareLaunchArgument(
        'mission_file', default_value=default_mission,
        description='任务列表 YAML 路径')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='仿真用 true (跟 Gazebo 时钟)')

    mission_manager = launch_ros.actions.Node(
        package='mm_task',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[{
            'mission_file': LaunchConfiguration('mission_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return launch.LaunchDescription([
        mission_file_arg,
        use_sim_time_arg,
        mission_manager,
    ])
