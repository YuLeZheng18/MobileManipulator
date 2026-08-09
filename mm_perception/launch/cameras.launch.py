"""整车相机驱动: 车体两路 USB 相机 (+装反校正) + 手眼深度相机 D435i.

车体两路: 同型号同序列号 (Generic PC Camera A2), by-id 撞车; 用 by-path (USB 拓扑口) 区分:
  cam_a (ArUco, Link_13): USB 口 2.2.2, 上下颠倒
  cam_b (监视, Link_14):  USB 口 2.2.3, 装反
每路**只起 usb_cam**, 发 /cam_x/image_raw(+/compressed) + /cam_x/camera_info, 到此为止.

⚠️ 本文件**不做转正** (2026-08-03/08-04 两次精简, 别照 rotation 参数名想当然):
  - 看画面: teleop_stack 起的 web_video_server 在 URL 里加 invert=1 服务端转正,
    见 mm_bringup/web/monitor.html。本机不起任何图像 ROS 节点。
  - ArUco: aruco_real.launch.py **自带**一个 image_rotator (image_in <- /cam_a/image_raw,
    image_out -> /cam_a/image_rot), 满足 aruco_localizer.yaml 的 image_topic。
  故 cam_a_rotation / cam_b_rotation 两个参数**已不起作用**, 保留只为不动调用处签名.

手眼深度相机 (Link_30, D435i): 只起 realsense2_camera. 彩色 compressed 由驱动自带的
image_transport 直接发, 无需额外 republish.

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
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# by-path 稳定路径 (USB 拓扑口不随插拔换号); {} 填 USB 口号如 2.2.2
_BYPATH = '/dev/v4l/by-path/platform-3610000.usb-usb-0:{}:1.0-video-index0'


def _fix_framerate(device):
    """关掉 v4l2 的 exposure_dynamic_framerate, 否则相机自己降帧 (30 -> 18~20Hz).

    2026-08-03 定案。这个 UVC 控制项的含义是"允许为了拉长曝光而降低帧率", 默认应为 0,
    但这两个相机上读到的是 1 (谁置的不明, 可能是固件默认或某次 v4l2 调试残留)。
    表现: framerate:=30.0 给了也没用, 实测 camera_info 只有 18.061 / 19.875 Hz;
    关掉后立刻 30.354 / 30.292 Hz。auto_exposure 本身保持自动 (=3 光圈优先), 只是
    不再允许它牺牲帧率 —— 暗环境会改为增益补偿, 画面噪点变多但帧率稳住。

    ⚠️ usb_cam **没有**这个参数入口 (ros2 param list 里没有, libusb_cam.so 里也搜不到),
    所以只能在起 usb_cam 前用 v4l2-ctl 外部设。也因此它是 v4l2 设备级状态, 重启即丢,
    必须每次起栈都设一遍 —— 这就是它写在 launch 里而不是 usb_cam.yaml 里的原因。
    失败不阻塞启动 (|| true): 顶多回到降帧, 不该因此起不来相机。
    """
    return ExecuteProcess(
        cmd=['bash', '-c',
             f'v4l2-ctl -d {device} --set-ctrl=exposure_dynamic_framerate=0 || true'],
        output='screen',
    )


def _one_cam(percep_share, name, device, frame_id, rotation, cam_info_url=''):
    """构造一路相机: 只起 usb_cam, 话题挂在 /<name>/ 命名空间下.

    ⚠️ 2026-08-03 删掉了原先跟在后面的 image_rotator + republish (转正 + 补发 compressed):
    车体两路只当监视画面用 (yolo 只吃 D435i, 不碰这两路), 而转正和编码这两级都有更省的去处。
    Orin 本地实测的依据 —— 同一路 usb_cam 自带的 image_raw/compressed 是 30.4Hz,
    而 republish 现编的 image_rot/compressed 只有 22.6Hz: 多那一级 JPEG 编码白掉 8Hz,
    还吃掉 ~16-20% 一个核。
    故 rotation 参数在这里已不起作用, 保留形参只为不动调用处签名。
    ⚠️ 转正现在由 **web_video_server 的 invert=1** 在服务端做 (2026-08-04 定案), 不在本机 ——
    本机一个图像 ROS 节点都不起了, 见 mm_bringup/web/monitor.html 与 dev_bringup 文件头。
    早前那句"那两级搬到本机去做"已作废, 别照着把本机那条链加回来。

    ⚠️ 别信"mjpeg2rgb 软解码是帧率瓶颈"那套说法 (我 2026-08-03 早前写错过, 已删):
    实测 usb_cam 只吃 13-17% CPU, 6 核 load 3.34, 根本不忙。当时看到的 11Hz 是**在本机
    隔着 WiFi 订 raw** 量的 —— 640x480 rgb8 = 921KB/帧, 那是网络丢帧, 不是采集帧率。
    判据: 量同一路的 camera_info (几十字节, 与图像同一次 publish), 它才是真实采集频率。
    真正压着帧率的是 v4l2 的 exposure_dynamic_framerate, 见下面 _setup 里的说明。

    ⚠️ ArUco 不受影响: 它走自己的 aruco_real.launch.py, 那里**自带**一个 image_rotator
    (image_in <- /cam_a/image_raw), 不依赖本文件。aruco_localizer.yaml 的
    image_topic=/cam_a/image_rot 由那条链自己满足。
    """
    usb_cam_cfg = os.path.join(percep_share, 'config', 'usb_cam.yaml')
    device = os.path.realpath(device)  # 解符号链接绕开 usb_cam 路径 bug
    # ⚠️⚠️ 长期卡顿的真凶 (2026-08-04 实测定案), 现在靠"本机不订任何图像话题"解决:
    # usb_cam 的 image_transport::CameraPublisher 同时发 image_raw(未压缩) 和
    # image_raw/compressed。raw 是普通 ROS 发布者, **跨机订它**时 DDS 就把 640x480 rgb8
    # @30Hz ≈ 27MB/s 推上 WiFi, 两路把链路彻底打满, 所有流一起被挤垮。
    # 判据: 停掉两个 usb_cam, Orin 网卡发送量 15312 KB/s -> 24 KB/s。
    # 之前查的一堆现象(rqt 选错 transport / D435i 彩色打满 / 画面卡成一帧) 都是它的下游。
    #
    # 现在的架构下 raw 留着是安全的, 也是必需的 (2026-08-04 改回来):
    #   看画面走 web_video_server (HTTP/MJPEG, 见 teleop_stack.launch.py), 它跑在 Nano 上,
    #   订 raw 是**机内**通信不过网; 过网的只有浏览器那条 HTTP 流。
    #   本机侧不再起任何图像 ROS 节点 —— 跨机 DDS 图像流彻底没有了, 真凶被根除。
    # ⚠️ 所以铁律是: **本机(或任何跨机进程)绝不订阅 /cam_x/image_raw**。要看画面开浏览器。
    #   web_video_server 的 type=ros_compressed 虽然零编码开销, 但它原样转发、忽略
    #   width/height/quality (实测 2.9MB/s 一路); 默认 mjpeg 模式吃 raw 可缩放到
    #   320x240 只要 ~270KB/s。为了这 10 倍带宽差, 宁可让 Nano 多编一次。
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
    # 先关动态降帧再起驱动 (launch 同批 action 无严格序, 但 v4l2-ctl 是设备级状态,
    # 先后到位都生效; 实测 usb_cam 跑着时改也立刻起效).
    return [_fix_framerate(device), cam]


def _depth_cam():
    """手眼深度相机 D435i (Link_30): 只起 realsense2_camera.

    双层命名空间 /camera/camera/ 是 realsense2_camera 的默认行为 (节点名也叫 camera).

    ⚠️ 别再给彩色流补 republish (2026-08-03 删): realsense 驱动的 image_transport **自带**
    发 /camera/camera/color/image_raw/compressed, 再起一个 republish 就是两个发布者往同一个
    话题名发同一路画面. 实测该话题 hz=42 (30 原生 + 11 republish 叠加), 而多出来的那次
    JPEG 编码吃掉 41% 一个核, 而原生那路已经够用.
    ⚠️ 括号里原有一句"车体两路的 republish 才是必需的"已作废 (2026-08-04): 那条链整个删了,
    现在谁也不订 compressed —— 看画面走 web_video_server 吃 raw 现编 (见文件头).

    align_depth 刻意**关掉** (2026-08-03): 本机彩色内参坏 (K/外参 NaN), 对齐图全废, 故
    yolo_box_detector 走 use_raw_depth=true 直接吃 depth/image_rect_raw + 深度内参反投影
    (见 yolo_box_detector.yaml 那条注释). 开着则驱动每秒 30 次把 848x480 深度重投影到
    1280x720 彩色画幅, 而 /aligned_depth_to_color/image_raw 实测**订阅者 0** —— 纯白烧.
    ⚠️ 别信"关掉能省近半个核"(我早前写过这句, 已删): 实测 realsense 节点 67.9% -> 79.6%,
    没有下降。关它的理由只是"无人订阅的计算没必要做", 不是实测省了多少 CPU.
    ⚠️ 早前各处注释写"align_depth 必开, yolo 吃 aligned_depth"是切到原始深度路线之前的
    遗留, 已不成立; 别照那些注释把它改回来. 真要用对齐图, 前提是先把彩色内参标好.
    """
    rs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'),
                         'launch', 'rs_launch.py')),
        launch_arguments={
            'align_depth.enable': 'false',  # 无人订阅, 纯白烧 CPU (见上 docstring)
            'pointcloud.enable': 'false',   # 点云绝不过网, 也不在机上白算 (架构 §7.4)
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_depth_camera')),
    )
    return [rs]


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
                              description='起手眼深度相机 D435i (抓取识别用)'),
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
