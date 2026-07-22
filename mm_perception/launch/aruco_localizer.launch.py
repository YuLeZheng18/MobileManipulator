"""启动 ArUco 定位节点.

用法:
  # 用 config/aruco_localizer.yaml 里的默认参数
  ros2 launch mm_perception aruco_localizer.launch.py

  # 换一份参数文件
  ros2 launch mm_perception aruco_localizer.launch.py params_file:=/path/to/xxx.yaml

  # 不改 yaml, 命令行临时覆盖话题 (非空即覆盖 yaml 内配置)
  ros2 launch mm_perception aruco_localizer.launch.py \
      image_topic:=/cam/image_raw camera_info_topic:=/cam/camera_info

说明:
  本节点通过参数 image_topic/camera_info_topic 决定订阅哪个话题 (不是固定话题名+remap),
  故这里用 OpaqueFunction 在运行时判断: 命令行传了非空值才追加参数覆盖 yaml, 否则完全用 yaml.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('mm_perception')
    default_params = os.path.join(pkg_share, 'config', 'aruco_localizer.yaml')

    params_file = LaunchConfiguration('params_file').perform(context) or default_params
    image_topic = LaunchConfiguration('image_topic').perform(context)
    info_topic = LaunchConfiguration('camera_info_topic').perform(context)

    # yaml 为主; 命令行非空值再以字典形式追加覆盖 (后者优先级更高)
    parameters = [params_file]
    overrides = {}
    if image_topic:
        overrides['image_topic'] = image_topic
    if info_topic:
        overrides['camera_info_topic'] = info_topic
    if overrides:
        parameters.append(overrides)

    aruco_node = Node(
        package='mm_perception',
        executable='aruco_localizer',
        name='aruco_localizer',
        output='screen',
        parameters=parameters,
    )
    return [aruco_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='aruco_localizer 参数 yaml 路径 (留空用包内默认 config)'),
        DeclareLaunchArgument(
            'image_topic', default_value='',
            description='相机图像话题 (非空则覆盖 yaml)'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='',
            description='相机内参话题 (非空则覆盖 yaml)'),
        OpaqueFunction(function=_launch_setup),
    ])
