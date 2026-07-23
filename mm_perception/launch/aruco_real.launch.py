"""真机 ArUco 定位: usb_cam 驱动 + aruco_localizer (无 Gazebo/合成相机).

拉起:
  usb_cam          -> 读真实 USB 相机, 发 /camera/image_raw + /camera/camera_info
  aruco_localizer  -> 检测+solvePnP, 广播 Link_13 -> aruco_<id> 到 /tf
可选:
  robot_state_publisher (with_rsp:=true) -> 发布整车 TF, 让 aruco TF 挂上整棵树

注意:
  未标定内参, 用 usb_cam 默认粗略内参 -> 距离/角度有系统误差, 仅供跑通流程.
  标定后在 config/usb_cam.yaml 里设 camera_info_url 指向标定文件即可.

用法:
  ros2 launch mm_perception aruco_real.launch.py                  # 默认弹可视化窗口
  ros2 launch mm_perception aruco_real.launch.py video_device:=/dev/video1
  ros2 launch mm_perception aruco_real.launch.py with_viewer:=false  # 不要窗口
  ros2 launch mm_perception aruco_real.launch.py with_rsp:=true   # 同时发整车TF
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _setup(context, *args, **kwargs):
    percep_share = get_package_share_directory('mm_perception')
    usb_cam_cfg = os.path.join(percep_share, 'config', 'usb_cam.yaml')
    aruco_cfg = os.path.join(percep_share, 'config', 'aruco_localizer.yaml')
    # 未标定兜底内参: usb_cam 默认发全 0 的 K (fx=fy=0 会让 solvePnP 算出垃圾位姿),
    # 故用 camera_info_url 加载一份粗略默认内参 (fx=fy=600, 主点居中).
    cam_info_url = 'file://' + os.path.join(percep_share, 'config', 'default_camera_info.yaml')

    video_device = LaunchConfiguration('video_device').perform(context)
    # by-id 路径(如 /dev/v4l/by-id/xxx)是符号链接, usb_cam 拼路径有 bug(变成 /dev/../../videoN),
    # 故这里启动时解析成真实 /dev/videoN 再传. 每次启动重新解析, 插拔换号也不用改.
    video_device = os.path.realpath(video_device)
    with_rsp = LaunchConfiguration('with_rsp').perform(context).lower() == 'true'
    with_viewer = LaunchConfiguration('with_viewer').perform(context).lower() == 'true'

    # usb_cam: 图像话题统一 remap 到 /camera/image_raw + /camera/camera_info,
    # 与 aruco_localizer.yaml 默认订阅话题对齐 (免得改两处).
    usb_cam = Node(
        package='usb_cam', executable='usb_cam_node_exe', name='usb_cam',
        output='screen',
        parameters=[usb_cam_cfg, {'video_device': video_device,
                                  'camera_info_url': cam_info_url}],
        remappings=[
            ('/image_raw', '/cam_a/image_raw'),
            ('/camera_info', '/cam_a/camera_info'),
        ],
    )

    # cam_a 装反(180°): image_rotator 转正 -> /cam_a/image_rot(+camera_info_rot),
    # 与 aruco_localizer.yaml 订阅话题对齐 (ArUco 吃转正流, 非原始歪图).
    rotation = int(LaunchConfiguration('rotation').perform(context))
    rotator = Node(
        package='mm_perception', executable='image_rotator', name='image_rotator',
        output='screen', parameters=[{'rotation': rotation}],
        remappings=[('image_in', '/cam_a/image_raw'),
                    ('info_in', '/cam_a/camera_info'),
                    ('image_out', '/cam_a/image_rot'),
                    ('info_out', '/cam_a/camera_info_rot')],
    )

    # with_viewer 时强制打开调试图发布 (在 yaml 之上追加覆盖)
    aruco_params = [aruco_cfg]
    if with_viewer:
        aruco_params.append({'publish_debug_image': True})
    aruco = Node(
        package='mm_perception', executable='aruco_localizer',
        name='aruco_localizer', output='screen',
        parameters=aruco_params,
    )

    nodes = [usb_cam, rotator, aruco]

    # 可视化窗口: rqt_image_view 直接订阅调试图 (画了检测边框+id 的相机画面).
    # 调试图话题 = 节点名 + 相对话题 ~/debug_image -> /aruco_localizer/debug_image
    if with_viewer:
        nodes.append(Node(
            package='rqt_image_view', executable='rqt_image_view',
            name='aruco_viewer', output='screen',
            arguments=['/aruco_localizer/debug_image'],
        ))

    # 可选整车 TF: aruco_<id> 的父系是 Link_13, 需 rsp 把 Link_13 挂到 base/map 上才成树
    if with_rsp:
        desc_share = get_package_share_directory('mm_description')
        with open(os.path.join(desc_share, 'urdf', 'mm_robot.urdf')) as f:
            robot_desc = f.read()
        nodes.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            output='screen', parameters=[{'robot_description': robot_desc}],
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device',
            # 用 by-id 稳定路径: /dev/videoN 会随插拔/重启变号, by-id 恒定指向本相机的捕获口.
            default_value='/dev/v4l/by-id/usb-Generic_PC_Camera_A2_200901010001-video-index0',
            description='USB 相机设备 (默认 by-id 稳定路径; 换相机则传 /dev/videoN)'),
        DeclareLaunchArgument(
            'with_rsp', default_value='false',
            description='true=同时起 robot_state_publisher 发整车 TF (让 aruco TF 挂上树)'),
        DeclareLaunchArgument(
            'with_viewer', default_value='true',
            description='true=弹出 rqt_image_view 显示检测画面(带边框+id); false=无窗口'),
        DeclareLaunchArgument(
            'rotation', default_value='180',
            description='cam_a 装反校正角 0/90/180/270 (image_rotator)'),
        OpaqueFunction(function=_setup),
    ])
