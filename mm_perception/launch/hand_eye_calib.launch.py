"""手眼标定 (eye-in-hand): 起 hand_eye_calibrator 节点.

前提 (本 launch 不代起, 需先各自就绪):
  - 真机臂 bringup 已起 (arm_controller / ros2_control / can_bridge / robot_state_publisher),
    即 /arm_controller/follow_joint_trajectory 与 TF Link_20->Link_29 可用;
  - 深度相机(RealSense)驱动已起, 发 color/image_raw + color/camera_info;
  - 固定 ArUco 标记在所有预存姿态下都能被相机看到.

用法:
  ros2 launch mm_perception hand_eye_calib.launch.py
  ros2 launch mm_perception hand_eye_calib.launch.py params_file:=/path/to/my.yaml

输出: 打印 Link_29->Link_30 的 xyz+rpy, 并写 hand_eye_result.yaml (交集成者回填 Joint_17).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    default_params = os.path.join(
        get_package_share_directory('mm_perception'),
        'config', 'hand_eye_calib.yaml')
    params_file = LaunchConfiguration('params_file').perform(context) or default_params
    node = Node(
        package='mm_perception', executable='hand_eye_calibrator',
        name='hand_eye_calibrator', output='screen',
        parameters=[params_file],
    )
    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='手眼标定参数 yaml (留空用包内默认 config/hand_eye_calib.yaml)'),
        OpaqueFunction(function=_setup),
    ])
