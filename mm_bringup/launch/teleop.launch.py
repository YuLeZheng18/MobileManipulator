"""手柄遥控 launch (real-only) — 手柄 -> /cmd_vel -> 底盘固件 (路线图 D2.2)。

    joy_node -> /joy -> teleop_twist_joy_node -> /cmd_vel

⚠️ 只起遥控链, 不起底盘代理。ESP32 的 micro_ros_agent 要另外起, 否则 /cmd_vel
没人订阅、车不动 (起栈时最容易漏的一步):
    source ~/microros_ws/install/setup.bash
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp32_chassis

⚠️ 不与 Nav2 同时跑。两者都往 /cmd_vel 发, 松开死人开关时 teleop 发的零速会把
Nav2 的指令按住。"随时接管"要等做完优先级仲裁 (mux) 才能开 (路线图 D4.2)。

不经 cmd_vel_smoother: 固件已有一级加速度斜坡 (config.h MAX_LIN_ACCEL 0.6 /
MAX_ANG_ACCEL 3.0), 再串一级手感发黏; teleop_ps2.yaml 里的 scale 值也是直发量到的。

⚠️ 斜推时合速度比单轴快 41% (实测 0.281 m/s vs 0.200): teleop_twist_joy 不做
摇杆归一化, 固件也没有速度钳位。演示走位注意, 别斜着推满。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    default_config = os.path.join(
        get_package_share_directory('mm_bringup'), 'config', 'teleop_ps2.yaml')

    args = [
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='手柄参数 (轴/按键映射 + 速度上限), 默认 config/teleop_ps2.yaml'),
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='/cmd_vel',
            description='速度指令话题 (默认直发 /cmd_vel; 将来接 mux 时改这里)'),
    ]

    # 手柄驱动。设备靠 config 里的 device_name 按 HID 名字匹配 (joy_node 不吃设备路径)。
    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[config_file],
    )

    # 摇杆 -> Twist。节点名必须是 teleop_twist_joy_node: yaml 里参数挂在这个名字下,
    # 改名会导致整份参数静默不生效 (退回代码默认值, 速度上限会变)。
    teleop = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[config_file],
        remappings=[('/cmd_vel', cmd_vel_topic)],
    )

    return LaunchDescription(args + [joy, teleop])
