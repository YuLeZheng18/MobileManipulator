"""本机 (笔记本) bringup — 分布式调试的"上半场" (架构 §7.4)。

只跑给人看的可视化 + 粗粒度调度, 不碰任何硬件、不产任何 TF:
  - RViz (MoveIt 视图: 机器人模型 + 规划场景 + TF; 可手动加 Nav2 的 map/costmap/path 显示)。
  - 相机监视 x3 (view_cameras:=true): cam_a(ArUco 转正)、cam_b(监视转正)、D435i 彩色。
  - mm_task 状态机 (默认关): 发 /go_to /initialpose、调 /grasp/* 服务, 都是小消息粗指令。

⚠️ 相机画面走"本机解码"而非 rqt 直吃 compressed:
  Nano 把每路转正流/彩色流用 image_transport 发 <topic>/compressed 过网 (~0.5MB/s);
  本机 republish 把 compressed 解回本地 raw (<name>/view, 走 loopback 不占 WiFi),
  rqt 直接看本地 raw — 无需在 Transport 下拉手动切 compressed (那下拉易空/易错)。
  两机都需装 compressed_image_transport (apt)。别直传原始大图/点云把 WiFi 打满。
  深度图暂不看: compressedDepth 的 republish 有 bug, 且 depth raw ~15MB/s 过网太重。

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

    args = [
        DeclareLaunchArgument('run_mission', default_value='false',
                              description='true=起 mm_task 自动跑 S0->S5; '
                                          'false=只起 RViz, 手动派命令调试 (推荐)'),
        DeclareLaunchArgument('view_cameras', default_value='false',
                              description='本机监视三路相机 (需 Nano use_cameras:=true + '
                                          'D435i 就绪); 每路 republish 解码 compressed 再 rqt'),
    ]

    # RViz: MoveIt 视图 (use_sim_time=false 实机时钟)。robot_state_publisher 在 Nano,
    # 本机 RViz 只消费 /robot_description + /tf + 规划场景, 不另起 rsp。
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('arm_moveit_config'),
                         'launch', 'moveit_rviz.launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    # 相机监视 (view_cameras:=true): 每路 = republish(compressed->本地raw) + rqt 看本地raw。
    # compressed 过网 (~0.5MB/s), 本机解回 <name>/view (loopback), rqt 无需切 Transport。
    def _view(name, compressed_in):
        rep = Node(
            package='image_transport', executable='republish',
            name=f'decode_{name}', output='screen',
            arguments=['compressed', 'raw'],
            remappings=[('in/compressed', compressed_in), ('out', f'/{name}/view')],
            condition=IfCondition(view_cameras))
        view = Node(
            package='rqt_image_view', executable='rqt_image_view', name=f'view_{name}',
            arguments=[f'/{name}/view'], output='screen',
            condition=IfCondition(view_cameras))
        return [rep, view]

    # cam_a/cam_b: 转正流的 compressed (mm_perception/cameras.launch.py 里的 republish 发的);
    # D435i 彩色: realsense 双层命名空间 /camera/camera/...; 深度不看 (见文件头说明)。
    cams = (_view('cam_a', '/cam_a/image_rot/compressed')
            + _view('cam_b', '/cam_b/image_rot/compressed')
            + _view('color', '/camera/camera/color/image_raw/compressed'))

    # mm_task: 顶层调度 (默认关, 调试时手动派命令; run_mission:=true 才自动整轮跑)
    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mm_task'),
                         'launch', 'mission.launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items(),
        condition=IfCondition(run_mission),
    )

    return LaunchDescription(args + [rviz] + cams + [mission])
