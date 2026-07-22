"""机器人端 (Nano) bringup — 分布式调试的"下半场" (架构 §7.4)。

分布式部署原则: 摸硬件的(USB/串口/CAN) + 高带宽原始数据 + 延迟敏感闭环
(底盘控制环 / 视觉伺服 / 感知闭环) 全部在 Nano 就地产出、就地闭环, 绝不过 WiFi;
本机(笔记本)只跑 RViz 可视化 + mm_task 粗粒度调度 (见 dev_bringup.launch.py)。

本 launch = real_bringup 的机器人端全栈 (硬件桥接 + EKF + RSP + Nav2 + MoveIt +
moveit_servo + grasp_node + mm_perception), 无头, 且 **不含 mm_task** (mm_task 在本机起)。
实现上直接复用 real_bringup.launch.py 并强制 run_mission:=false, 避免逻辑重复漂移。
单机整机自主跑(不拆分)仍用 real_bringup.launch.py run_mission:=true。

⚠️ 运行前置 (Nano 上):
  - `export ROS_DOMAIN_ID=<N>`  两机必须一致, 且同 RMW (默认 rmw_fastrtps_cpp)。
  - `source ~/microros_ws/install/setup.bash`  (micro_ros_agent 独立 ws)。
  - `source install/setup.bash`  (本 colcon ws)。
  - CAN 接口已 up (ip link set can0 up type can bitrate 1000000)。
  - 两机 NTP/chrony 对时 (TF 时间戳跨机对齐, 否则 tf2 报 extrapolation)。
  - 多播可达 (WiFi 常屏蔽多播 -> 发现失败; 不行则配 Fast-DDS Discovery Server 单播)。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_lidar = LaunchConfiguration('use_lidar')
    use_cameras = LaunchConfiguration('use_cameras')
    use_perception = LaunchConfiguration('use_perception')
    agent_serial_dev = LaunchConfiguration('agent_serial_dev')
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')

    args = [
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_cameras', default_value='false',
                              description='相机驱动 (型号定后接线, 感知开时需一并开)'),
        DeclareLaunchArgument('use_perception', default_value='false',
                              description='mm_perception 真感知 (队友节点就绪后开)'),
        DeclareLaunchArgument('agent_serial_dev', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyUSB0'),
    ]

    real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mm_bringup'),
                         'launch', 'real_bringup.launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'run_mission': 'false',        # mm_task 在本机 dev_bringup 起, Nano 不起
            'use_lidar': use_lidar,
            'use_cameras': use_cameras,
            'use_perception': use_perception,
            'agent_serial_dev': agent_serial_dev,
            'lidar_serial_port': lidar_serial_port,
        }.items(),
    )

    return LaunchDescription(args + [real])
