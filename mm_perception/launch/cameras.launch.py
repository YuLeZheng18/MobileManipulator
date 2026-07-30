"""整车相机驱动: 车体两路 USB 相机 (+装反校正) + 手眼深度相机 D435i.

车体两路: 同型号同序列号 (Generic PC Camera A2), by-id 撞车; 用 by-path (USB 拓扑口) 区分:
  cam_a (ArUco, Link_13): USB 口 2.2.2, 上下颠倒 -> rotation:=180
  cam_b (监视, Link_14):  USB 口 2.2.3, 装反     -> rotation:=180
每路: usb_cam 发 /cam_x/image_raw(+camera_info) -> image_rotator 转正
      -> /cam_x/image_rot(+camera_info_rot). 下游 (ArUco/rqt 监视) 只吃转正流.

手眼深度相机 (Link_30, D435i): realsense2_camera, **align_depth 必须启动时开** ——
yolo_box_detector 吃 aligned_depth_to_color, 彩色框才落得到深度上; 事后 ros2 param set
改不了 (驱动按启动配置建 pipeline). 顺带 republish 彩色流的 compressed 供本机跨 WiFi 监视
(dev_bringup 订的就是 /camera/camera/color/image_raw/compressed).

usb_cam 对符号链接设备路径有 bug (拼成 /dev/../../videoN), 故启动时 realpath 成真实
/dev/videoN 再传. 由 real_bringup 在 use_cameras:=true 时 include; 也可单独起:
  ros2 launch mm_perception cameras.launch.py
  ros2 launch mm_perception cameras.launch.py use_body_cameras:=false  # 只起深度相机
  ros2 launch mm_perception cameras.launch.py use_depth_camera:=false  # 只起车体两路
  ros2 launch mm_perception cameras.launch.py cam_a_rotation:=0        # 关某路旋转
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    # 转正流 raw->compressed republish: 本机(笔记本)跨 WiFi 只订阅 compressed 看实时,
    # 别直传 raw 转正流(640x480@30 ~220Mbps 打满 WiFi). usb_cam 原始图有 compressed,
    # 但那是歪的; 转正流是 image_rotator 现发的, 得自己补一路 compressed.
    republish = Node(
        package='image_transport', executable='republish',
        name='republish', namespace=name, output='screen',
        arguments=['raw', 'compressed'],
        remappings=[('in', f'/{name}/image_rot'),
                    ('out/compressed', f'/{name}/image_rot/compressed')],
    )
    return [cam, rot, republish]


def _depth_cam():
    """手眼深度相机 D435i (Link_30): realsense2_camera + 彩色流 compressed republish.

    align_depth.enable=true 是硬要求: yolo_box_detector 订
    /camera/camera/aligned_depth_to_color/image_raw, 靠它把彩色框的像素直接查深度.
    不开则只有未对齐的 depth/image_rect_raw, 彩色框落不到深度上 -> 检测拿不到 z.
    双层命名空间 /camera/camera/ 是 realsense2_camera 的默认行为 (节点名也叫 camera).
    """
    rs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'),
                         'launch', 'rs_launch.py')),
        launch_arguments={
            'align_depth.enable': 'true',
            'pointcloud.enable': 'false',   # 点云绝不过网, 也不在机上白算 (架构 §7.4)
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_depth_camera')),
    )
    # 彩色流补一路 compressed: 本机 dev_bringup 跨 WiFi 只订 compressed 看画面.
    republish = Node(
        package='image_transport', executable='republish',
        name='republish_color', output='screen',
        arguments=['raw', 'compressed'],
        remappings=[('in', '/camera/camera/color/image_raw'),
                    ('out/compressed', '/camera/camera/color/image_raw/compressed')],
        condition=IfCondition(LaunchConfiguration('use_depth_camera')),
    )
    return [rs, republish]


def _setup(context, *args, **kwargs):
    percep_share = get_package_share_directory('mm_perception')
    lc = lambda n: LaunchConfiguration(n).perform(context)

    nodes = []
    if lc('use_body_cameras').lower() == 'true':
        # cam_a 供 ArUco: 未标定, 用粗略默认内参 (fx=fy=600, 主点居中); 标定后换此文件.
        cam_a_info = 'file://' + os.path.join(
            percep_share, 'config', 'default_camera_info.yaml')
        nodes += _one_cam(percep_share, 'cam_a', lc('cam_a_device'), 'Link_13',
                          int(lc('cam_a_rotation')), cam_info_url=cam_a_info)
        # cam_b 纯监视: 不喂感知, 无需标定内参 (cam_info_url 留空).
        nodes += _one_cam(percep_share, 'cam_b', lc('cam_b_device'), 'Link_14',
                          int(lc('cam_b_rotation')))
    nodes += _depth_cam()
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_body_cameras', default_value='true',
                              description='起车体两路 USB 相机 (cam_a ArUco / cam_b 监视)'),
        DeclareLaunchArgument('use_depth_camera', default_value='true',
                              description='起手眼深度相机 D435i (align_depth 必开, 抓取识别用)'),
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
