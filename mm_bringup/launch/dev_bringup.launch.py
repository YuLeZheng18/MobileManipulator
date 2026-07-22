"""本机 (笔记本) bringup — 分布式调试的"上半场" (架构 §7.4)。

只跑给人看的可视化 + 粗粒度调度, 不碰任何硬件、不产任何 TF:
  - RViz (MoveIt 视图: 机器人模型 + 规划场景 + TF; 可手动加 Nav2 的 map/costmap/path 显示)。
  - rqt_image_view x2 (view_cameras:=true): 监视车体相机 + 手眼深度相机画面。
  - mm_task 状态机 (默认关): 发 /go_to /initialpose、调 /grasp/* 服务, 都是小消息粗指令。

⚠️ 相机画面必须走压缩传输, 别直传原始大图/点云把 WiFi 打满:
  两机都需装 `compressed_image_transport` + `compressed_depth_image_transport`(apt);
  Nano 相机驱动随 image_transport 自动发 <topic>/compressed 与 <topic>/compressedDepth,
  本机 rqt 打开后在 Transport 下拉里选 compressed / compressedDepth(默认 raw 会打满带宽)。
  深度点云更重, 调试看深度"图"(compressedDepth)即可, 别把点云拉过网。

机器人端全栈 (硬件/控制/Nav2/MoveIt/感知) 在 Nano 上由 nano_bringup.launch.py 起。
两机同一 ROS_DOMAIN_ID + 同 LAN, DDS 自动发现, 话题/TF/服务/action 跨机透明。
RViz 的机器人模型/TF/规划场景全部来自 Nano (over LAN, 只读可视化)。

调试常用 (本机直接敲, 无需进 launch):
  ros2 topic pub /go_to std_msgs/String "{data: p2}"      # 手动派一段导航
  ros2 service call /grasp/execute std_srvs/srv/Trigger    # 手动触发一次抓取
  ros2 topic echo /perception/object_pose                  # 看感知输出 (小消息, 过网 OK)

⚠️ 运行前置 (本机):
  - `export ROS_DOMAIN_ID=<N>`  与 Nano 一致, 同 RMW。
  - `source install/setup.bash`。
  - 两机 NTP/chrony 对时。
  - 不要在本机 source microros_ws / 不接任何硬件驱动。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    run_mission = LaunchConfiguration('run_mission')
    view_cameras = LaunchConfiguration('view_cameras')
    color_image_topic = LaunchConfiguration('color_image_topic')
    depth_image_topic = LaunchConfiguration('depth_image_topic')

    args = [
        DeclareLaunchArgument('run_mission', default_value='false',
                              description='true=起 mm_task 自动跑 S0->S5; '
                                          'false=只起 RViz, 手动派命令调试 (推荐)'),
        DeclareLaunchArgument('view_cameras', default_value='false',
                              description='起 rqt_image_view 监视相机 (需 Nano use_cameras:=true '
                                          '且真机相机驱动就绪; 相机型号定后开)'),
        # 话题名沿用架构 §7.1 阶段B 约定, 需与真机相机驱动实际发布名对齐
        DeclareLaunchArgument('color_image_topic', default_value='/camera/color/image_raw',
                              description='车体相机彩色图 (rqt 里 Transport 选 compressed)'),
        DeclareLaunchArgument('depth_image_topic', default_value='/camera/depth/image_rect_raw',
                              description='手眼深度相机深度图 (rqt 里 Transport 选 compressedDepth)'),
    ]

    # RViz: MoveIt 视图 (use_sim_time=false 实机时钟)。robot_state_publisher 在 Nano,
    # 本机 RViz 只消费 /robot_description + /tf + 规划场景, 不另起 rsp。
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('arm_moveit_config'),
                         'launch', 'moveit_rviz.launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    # 相机监视: 各开一个 rqt_image_view (view_cameras:=true)。透传的是压缩流,
    # 打开后在 Transport 下拉选 compressed(彩色)/ compressedDepth(深度), 勿用 raw。
    view_color = Node(
        package='rqt_image_view', executable='rqt_image_view', name='view_color',
        arguments=[color_image_topic], output='screen',
        condition=IfCondition(view_cameras))
    view_depth = Node(
        package='rqt_image_view', executable='rqt_image_view', name='view_depth',
        arguments=[depth_image_topic], output='screen',
        condition=IfCondition(view_cameras))

    # mm_task: 顶层调度 (默认关, 调试时手动派命令; run_mission:=true 才自动整轮跑)
    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mm_task'),
                         'launch', 'mission.launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items(),
        condition=IfCondition(run_mission),
    )

    return LaunchDescription(args + [rviz, view_color, view_depth, mission])
