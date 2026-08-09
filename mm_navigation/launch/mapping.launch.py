#!/usr/bin/env python3
"""真机激光 SLAM 建图 (D4.1) — slam_toolbox online_async。

只起 slam_toolbox 一个节点。雷达、scan_filter、EKF 由 real_bringup 提供, 这里不重复起,
免得两份 rplidar 抢同一个串口设备。

===== 前置条件 (缺一不可, 缺了表现各不相同且都不好查) =====
1. /scan_filtered 在发   —— 由 rplidar + scan_filter 提供 (real_bringup use_lidar:=true)
2. odom->base_link TF    —— 由 EKF 提供; 缺了 slam_toolbox 静默不出地图
3. base_link->laser_link TF —— 由整车 RSP (臂栈 real_bringup) 提供
   ⚠️ 2026-08-08 状态: laser_link 是当天新加进 URDF 的, 真机 RSP 若未重启过则**没有这个帧**,
      slam_toolbox 会持续报 "frame does not exist" 而不出图。起建图前先确认:
        ros2 run tf2_ros tf2_echo base_link laser_link
      预期 xyz=[0.106, 0, 0.221] rpy=[0, 0, pi]。不出就是 RSP 还是旧的, 需重启臂栈。

===== 用法 =====
  ros2 launch mm_navigation mapping.launch.py
  ros2 launch mm_navigation mapping.launch.py open_rviz:=true    # 本机看覆盖情况

存图 (在 map 完整覆盖目标区域后):
  ros2 run nav2_map_server map_saver_cli -f <路径>/warehouse --ros-args -p save_map_timeout:=10.0
  或走 slam_toolbox 自己的服务(带位姿图, 便于日后续建):
  ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializeMap "{filename: '<路径>/warehouse'}"

===== 存图之后还有两件事, 别漏 =====
  1. lane_graph.yaml 要在新地图坐标系里重写 —— 现值是仿真 room 地图的, 直接用会导航到错位置
  2. 地图原点 = 建图起点。起点按地面 L 形胶带摆(车头平行货架通道) ⇒ initial_pose 恒 (0,0,0),
     mission_real.yaml 的 initial_pose TODO 由此零代码闭合
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mm_nav_share = get_package_share_directory('mm_navigation')
    default_params = os.path.join(mm_nav_share, 'config', 'slam_toolbox.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    open_rviz = LaunchConfiguration('open_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='真机建图恒 false; 仿真里复用才设 true'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='slam_toolbox 参数 (scan_topic 等)'),
        DeclareLaunchArgument('open_rviz', default_value='false',
                              description='起 RViz 看建图覆盖 (通常在本机开, 不在 Nano)'),

        # async_slam_toolbox_node: 扫描匹配与位姿图优化异步跑, 不阻塞扫描回调。
        # Nano 算力有限, 同步版(sync) 在回环优化时会丢扫描, 故用 async。
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_mapping',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(open_rviz),
        ),
    ])
