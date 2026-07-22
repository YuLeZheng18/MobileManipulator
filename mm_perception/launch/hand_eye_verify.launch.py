"""手眼标定结果验证 (在线实测一致性): 起 hand_eye_verify 节点.

前提 (本 launch 不代起):
  - 真机臂 bringup 已起 (arm_controller + TF Link_20->Link_29);
  - 深度相机 infra1 流已起 (若被重启, 先 enable_infra1:=true + emitter_enabled:=0);
  - 固定 ArUco 标记 (DICT_4X4_50, id=0, 边长0.135m) 在相机视野内.

用法:
  ros2 launch mm_perception hand_eye_verify.launch.py
  ros2 launch mm_perception hand_eye_verify.launch.py params_file:=/path/to/my.yaml

manual 采样: 手动把臂摆到几个能看到 marker 的姿态, 回车逐组采样, 最后按 s 出对比报告.
输出: [标定结果] 与 [URDF名义值] 两套外参的 base<-marker 位置/姿态散布, 散布小者更准.
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
        'config', 'hand_eye_verify.yaml')
    params_file = LaunchConfiguration('params_file').perform(context) or default_params
    node = Node(
        package='mm_perception', executable='hand_eye_verify',
        name='hand_eye_verify', output='screen',
        parameters=[params_file],
    )
    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='验证参数 yaml (留空用包内 config/hand_eye_verify.yaml)'),
        OpaqueFunction(function=_setup),
    ])
