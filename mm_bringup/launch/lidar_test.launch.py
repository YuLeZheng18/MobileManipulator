#!/usr/bin/env python3
# 只起思岚 A3M1 雷达 -> /scan (frame_id=Link_12), 供 Nano 单独验雷达用,
# 不拉 micro-ROS/臂/整栈。参数全可覆盖。串口默认 Jetson 硬件 UART /dev/ttyTHS1
# (雷达走排针串口非 USB; 用前需 stop nvgetty 释放该口)。A3 波特 256000。
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    scan_mode = LaunchConfiguration('scan_mode')
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyTHS1',
                              description='雷达串口 (Jetson 硬件 UART; 若走 USB 转接改 /dev/ttyUSB0)'),
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
