#!/usr/bin/env python3
"""雷达自遮挡滤除: /scan -> /scan_filtered.

砍掉车后方被机械臂本体与线材遮挡的扇区。边界的实测由来与"为什么必须填 NaN"
见 config/scan_filter.yaml 的注释, 那里是权威说明, 别在这里复述。

⚠️ 起了这个节点还不够 —— 下游四处都必须改指 /scan_filtered:
     AMCL 的 scan_topic、global_costmap 与 local_costmap 的 observation_sources、
     以及建图时 slam_toolbox 的 scan_topic。
   **漏掉建图那处, 机械臂就会被建进地图**, 成为跟着车走的幽灵墙。

用法:
  ros2 launch mm_navigation scan_filter.launch.py
  ros2 launch mm_navigation scan_filter.launch.py input:=/scan output:=/scan_filtered
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('mm_navigation'), 'config', 'scan_filter.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('input', default_value='/scan',
                              description='输入原始扫描 (rplidar_ros 直出)'),
        DeclareLaunchArgument('output', default_value='/scan_filtered',
                              description='输出滤除自遮挡后的扫描 (下游全部用这个)'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='filter chain 配置'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # scan_to_scan_filter_chain: 单入单出的 LaserScan filter chain 容器。
        # 节点名固定为 scan_filter_chain —— yaml 的顶层键必须与之一致, 否则参数加载不上
        # (ros2 param 按节点名匹配, 名字不对表现为"filter 没生效"却不报错)。
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_filter_chain',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[
                ('scan', LaunchConfiguration('input')),
                ('scan_filtered', LaunchConfiguration('output')),
            ],
        ),
    ])
