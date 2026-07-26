#!/usr/bin/env python3
# 只起思岚 A3 雷达 -> /scan (frame_id=Link_12), 供 Nano 单独验雷达用,
# 不拉 micro-ROS/臂/整栈。参数全可覆盖。串口默认 /dev/rplidar
# (经 CP2102 USB-TTL 接入; udev 软链见 99-rplidar.rules)。A3 波特 256000。
# 注: 排针 UART(ttyTHS1)信号完整性不足且 Jetson 内核无 CH340 驱动, 故走 CP2102 USB-TTL。
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    scan_mode = LaunchConfiguration('scan_mode')
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/rplidar',
                              description='雷达串口 (CP2102 USB-TTL, udev 软链 /dev/rplidar; 无软链可传 /dev/ttyUSB0)'),
        DeclareLaunchArgument('scan_mode', default_value='Standard',
                              description='A3 扫描模式: Standard/Express/Boost/Sensitivity'),
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': serial_port,
                'serial_baudrate': 256000,
                'frame_id': 'Link_12',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': scan_mode,
            }],
        ),
    ])
