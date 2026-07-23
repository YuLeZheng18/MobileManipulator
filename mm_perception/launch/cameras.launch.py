"""车体两路 USB 相机驱动 + 装反校正 (image_rotator).

两相机同型号同序列号 (Generic PC Camera A2), by-id 撞车; 用 by-path (USB 拓扑口) 区分:
  cam_a (ArUco, Link_13): USB 口 2.2.2, 上下颠倒 -> rotation:=180
  cam_b (监视, Link_14):  USB 口 2.2.3, 装反     -> rotation:=180
每路: usb_cam 发 /cam_x/image_raw(+camera_info) -> image_rotator 转正
      -> /cam_x/image_rot(+camera_info_rot). 下游 (ArUco/rqt 监视) 只吃转正流.

usb_cam 对符号链接设备路径有 bug (拼成 /dev/../../videoN), 故启动时 realpath 成真实
/dev/videoN 再传. 由 real_bringup 在 use_cameras:=true 时 include; 也可单独起:
  ros2 launch mm_perception cameras.launch.py
  ros2 launch mm_perception cameras.launch.py cam_a_rotation:=0   # 关某路旋转
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# by-path 稳定路径 (USB 拓扑口不随插拔换号); {} 填 USB 口号如 2.2.2
_BYPATH = '/dev/v4l/by-path/platform-3610000.usb-usb-0:{}:1.0-video-index0'


def _one_cam(percep_share, name, device, frame_id, rotation, cam_info_url=''):
    """构造一路相机: usb_cam + image_rotator, 话题都挂在 /<name>/ 命名空间下."""
    usb_cam_cfg = os.path.join(percep_share, 'config', 'usb_cam.yaml')
    device = os.path.realpath(device)  # 解符号链接绕开 usb_cam 路径 bug
    params = {'video_device': device, 'frame_id': frame_id}
    if cam_info_url:
        params['camera_info_url'] = cam_info_url
    cam = Node(
        package='usb_cam', executable='usb_cam_node_exe', name='usb_cam',
        namespace=name, output='screen',
        parameters=[usb_cam_cfg, params],
        remappings=[('/image_raw', f'/{name}/image_raw'),
                    ('/camera_info', f'/{name}/camera_info')],
    )
    rot = Node(
        package='mm_perception', executable='image_rotator',
        name='image_rotator', namespace=name, output='screen',
        parameters=[{'rotation': rotation}],
        remappings=[('image_in', f'/{name}/image_raw'),
                    ('info_in', f'/{name}/camera_info'),
                    ('image_out', f'/{name}/image_rot'),
                    ('info_out', f'/{name}/camera_info_rot')],
    )
    return [cam, rot]


def _setup(context, *args, **kwargs):
    percep_share = get_package_share_directory('mm_perception')
    # cam_a 供 ArUco: 未标定, 用粗略默认内参 (fx=fy=600, 主点居中); 标定后换此文件.
    cam_a_info = 'file://' + os.path.join(
        percep_share, 'config', 'default_camera_info.yaml')
    lc = lambda n: LaunchConfiguration(n).perform(context)

    nodes = _one_cam(percep_share, 'cam_a', lc('cam_a_device'), 'Link_13',
                     int(lc('cam_a_rotation')), cam_info_url=cam_a_info)
    # cam_b 纯监视: 不喂感知, 无需标定内参 (cam_info_url 留空).
    nodes += _one_cam(percep_share, 'cam_b', lc('cam_b_device'), 'Link_14',
                      int(lc('cam_b_rotation')))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cam_a_device', default_value=_BYPATH.format('2.2.2'),
                              description='cam_a (ArUco) 设备; by-path 稳定口 2.2.2'),
        DeclareLaunchArgument('cam_b_device', default_value=_BYPATH.format('2.2.3'),
                              description='cam_b (监视) 设备; by-path 稳定口 2.2.3'),
        DeclareLaunchArgument('cam_a_rotation', default_value='180',
                              description='cam_a 旋转角 0/90/180/270 (装反校正)'),
        DeclareLaunchArgument('cam_b_rotation', default_value='180',
                              description='cam_b 旋转角 0/90/180/270 (装反校正)'),
        OpaqueFunction(function=_setup),
    ])
