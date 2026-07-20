"""手眼标定 - 相机侧可行性验证 (不用机械臂).

只起 hand_eye_camera_check 节点, 订阅深度相机, 验证能否稳定检出固定 ArUco 并
solvePnP 解出 相机->标记 位姿. 相机侧过关后再上机械臂跑 hand_eye_calib.launch.py.

前提:
  - 深度相机(RealSense)驱动已起, 发 color/image_raw + color/camera_info;
  - 一个固定 ArUco 标记 (默认 DICT_4X4_50, id=0, 边长0.10m) 摆在相机视野内.

用法:
  ros2 launch mm_perception hand_eye_camera_check.launch.py
  ros2 launch mm_perception hand_eye_camera_check.launch.py \
      image_topic:=/camera/camera/color/image_raw \
      camera_info_topic:=/camera/camera/color/camera_info \
      marker_id:=0 marker_size:=0.10

复用 config/hand_eye_calib.yaml 里相机/标记相关参数, 命令行非空值再覆盖.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    # 沿用标定 yaml 的相机/标记参数, 保证与完整标定同一套约定.
    default_params = os.path.join(
        get_package_share_directory('mm_perception'),
        'config', 'hand_eye_calib.yaml')
    params_file = LaunchConfiguration('params_file').perform(context) or default_params

    overrides = {}
    _float_keys = ('marker_size', 'override_fx', 'override_fy', 'override_cx', 'override_cy')
    for key in ('image_topic', 'camera_info_topic', 'marker_id', 'marker_size',
                'override_fx', 'override_fy', 'override_cx', 'override_cy'):
        val = LaunchConfiguration(key).perform(context)
        if val:
            overrides[key] = (int(val) if key == 'marker_id'
                              else float(val) if key in _float_keys else val)
    # show_window: 'true'/'false' 字符串 -> bool. 开则弹 OpenCV 可视化窗口.
    sw = LaunchConfiguration('show_window').perform(context)
    if sw:
        overrides['show_window'] = sw.lower() in ('1', 'true', 'yes', 'on')

    # yaml 顶层键是 hand_eye_calibrator, 而本节点名 hand_eye_camera_check;
    # 用 dict override 直接喂到本节点, 不依赖 yaml 顶层节点名匹配.
    parameters = [params_file]
    if overrides:
        parameters.append(overrides)

    node = Node(
        package='mm_perception', executable='hand_eye_camera_check',
        name='hand_eye_camera_check', output='screen',
        parameters=parameters,
    )
    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value='',
                              description='参数 yaml (留空用包内 config/hand_eye_calib.yaml)'),
        DeclareLaunchArgument('image_topic', default_value='',
                              description='深度相机彩色图话题 (非空则覆盖)'),
        DeclareLaunchArgument('camera_info_topic', default_value='',
                              description='深度相机内参话题 (非空则覆盖)'),
        DeclareLaunchArgument('marker_id', default_value='',
                              description='固定标记 id (非空则覆盖)'),
        DeclareLaunchArgument('marker_size', default_value='',
                              description='标记黑边边长(米) (非空则覆盖)'),
        DeclareLaunchArgument('override_fx', default_value='',
                              description='内参兜底 fx (camera_info 为 NaN 时用; 非空则覆盖)'),
        DeclareLaunchArgument('override_fy', default_value='',
                              description='内参兜底 fy (非空则覆盖)'),
        DeclareLaunchArgument('override_cx', default_value='',
                              description='内参兜底 cx (<=0 用图像中心; 非空则覆盖)'),
        DeclareLaunchArgument('override_cy', default_value='',
                              description='内参兜底 cy (<=0 用图像中心; 非空则覆盖)'),
        DeclareLaunchArgument('show_window', default_value='',
                              description='true 则弹 OpenCV 可视化窗口 (需显示器)'),
        OpaqueFunction(function=_setup),
    ])
