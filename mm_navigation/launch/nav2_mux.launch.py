"""Nav2 + twist_mux 单独起 (不含臂/相机/感知), 用于导航调试与精度测试。

与 real_bringup.launch.py 的区别: 那条起真机全栈 (臂 + CAN + move_group + servo + 感知),
测导航时不需要, 起一遍要 20 多秒且占满 Nano。本 launch 只起导航必需的两样。

⚠️ 前置 (本 launch 不管, 需已在跑):
  micro_ros_agent (底盘) / EKF (odom->base_link) / rplidar + scan_filter (/scan_filtered)

cmd_vel 走向 (固件订阅的 /cmd_vel 名字不变, 只在它上游插一级 mux):

  controller_server → /cmd_vel_nav → velocity_smoother → /cmd_vel_nav_out ┐
  behavior_server (spin/backup/drive_on_heading/assisted_teleop/wait) ────┤
                     (这 5 路不经 smoother, Nav2 原设计如此) ─────────────┤
                                                                          ├→ twist_mux → /cmd_vel → 固件
  joy_arm_teleop (仅 DRIVE 态发) → /cmd_vel_manual ───────────────────────┘

⚠️ **remap 的 key 必须是 `cmd_vel_smoothed` 而不是 `cmd_vel`** —— 这里踩过一次:
   velocity_smoother 的原生话题名是 cmd_vel(入) / cmd_vel_smoothed(出), 而
   nav2_bringup/navigation_launch.py:183 已经先 remap 过一层:
       ('cmd_vel', 'cmd_vel_nav')  ('cmd_vel_smoothed', 'cmd_vel')
   所以 SetRemap('/cmd_vel', ...) 命中的是**第一条的 key**, 改掉的是 smoother 的
   **输入**(它于是去订 /cmd_vel_nav_out), 输出反而仍直发 /cmd_vel 绕过 mux。
   净效果是既丢了平滑、又让 mux 失效 —— 且不报任何错, 只能靠 `ros2 node info
   /velocity_smoother` 看 Subscribers/Publishers 才发现。
⚠️ **已知限制: behavior_server 的 5 路恢复行为不经 mux, 仍直发 /cmd_vel。**
   根因是话题名撞车且无法用 launch 手段分开: nav2_behaviors/timed_behavior.hpp:130
   写死 create_publisher("cmd_vel"), 与 velocity_smoother 的**输入** key 同名, 而两者在
   同一个 GroupAction 内 —— 一条 SetRemap('/cmd_vel',...) 必然同时命中两处, 改了
   behavior 的输出就等于同时掐断 smoother 的输入。nav2_bringup 也没有单独关掉
   smoother 的开关 (navigation_launch.py 只有 use_composition/container_name 等)。
   影响面: 恢复行为(spin/backup/drive_on_heading/assisted_teleop/wait)只在导航卡住时
   触发, 期间 controller 不出 cmd -> mux 的 nav 通道 0.5s 超时归零, 故正常情况下两者
   不会同时发。**唯一真风险**: 手柄按住接管的同时 Nav2 正在跑 spin, 此时 behavior 与
   mux 会抢 /cmd_vel (谁后到谁生效, 表现为车抽动)。规避: 要手动接管时先取消导航目标。
   彻底解法需改 nav2_behaviors 源码或给 behavior_server 单独套 namespace, 都超出本
   launch 范围。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    mm_nav_share = get_package_share_directory('mm_navigation')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    args = [
        DeclareLaunchArgument(
            'map', default_value=os.path.join(mm_nav_share, 'maps', 'room.yaml'),
            description='地图 yaml。真机实测图在 ~/Desktop/moveit/maps_real/room_real.yaml'),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(mm_nav_share, 'config', 'nav2_params.yaml'),
            description='Nav2 参数'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='实机恒 false'),
    ]

    # 只 remap velocity_smoother 的输出 (key = 它的原生名 cmd_vel_smoothed)。
    # **不能**再加一条 SetRemap('/cmd_vel', ...): 那会命中 smoother 的输入 key `cmd_vel`,
    # 把它从 /cmd_vel_nav 拽到 /cmd_vel_nav_out -> 它就收不到 controller_server 了
    # (实测过, 见 docstring)。代价是 behavior_server 那 5 路恢复行为仍直发 /cmd_vel
    # 绕过 mux —— 见 docstring 末尾的已知限制。
    nav2 = GroupAction([
        SetRemap('cmd_vel_smoothed', '/cmd_vel_nav_out'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_yaml,
                'params_file': params_file,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
    ])

    # 输出话题名 cmd_vel_out 是源码硬编码, 只能 remap (官方 twist_mux_launch.py 同样如此)。
    twist_mux = Node(
        package='twist_mux', executable='twist_mux', name='twist_mux', output='screen',
        parameters=[os.path.join(mm_nav_share, 'config', 'twist_mux.yaml'),
                    {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel_out', '/cmd_vel')],
    )

    return LaunchDescription(args + [nav2, twist_mux])
