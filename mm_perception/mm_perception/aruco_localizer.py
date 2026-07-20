#!/usr/bin/env python3
"""
ArUco 定位节点 (车体二维相机).

用途:
  1. 上电初始位姿标定 —— 识别墙面/货架已知 ArUco, 下游反推 map->base_footprint 发 /initialpose.
  2. 到点位置精矫正 —— 用 aruco 相对位姿做底盘伺服对位.

数据流:
  订阅 image + camera_info (相机 Link_13)
    -> cv2.aruco.ArucoDetector 检测角点
    -> cv2.solvePnP(IPPE_SQUARE) 解每个标记位姿 (结果在相机"光学系": z 朝前, x 朝右, y 朝下)
    -> 按约定直接当 父系(Link_13_optical) -> aruco_<id> 广播到 /tf

坐标系约定 (重要):
  OpenCV/ArUco 解出的位姿在"光学系"(REP-104: z 朝前). URDF 提供的 Link_13_optical
  即该光学系, 故默认直接广播 Link_13_optical -> aruco_<id>, 无需补旋转.
  若 Link_13_optical 尚未就绪, 打开 apply_optical_rotation: 父系改用机械系 Link_13,
  节点内左乘固定旋转 (rpy=-pi/2,0,-pi/2) 把光学系位姿变换回 Link_13 机械系.

兼容性:
  面向 OpenCV >= 4.7 的新版 ArUco API (ArucoDetector 类). 旧版 estimatePoseSingleMarkers 已废弃.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import cv2
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# OpenCV 预定义 ArUco 字典名 -> 常量. 覆盖常用几种, 现场按打印的标记选.
ARUCO_DICTS = {
    'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
    'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
    'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
    'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
    'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
    'DICT_5X5_250': cv2.aruco.DICT_5X5_250,
    'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
    'DICT_6X6_100': cv2.aruco.DICT_6X6_100,
    'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
    'DICT_7X7_50': cv2.aruco.DICT_7X7_50,
    'DICT_ARUCO_ORIGINAL': cv2.aruco.DICT_ARUCO_ORIGINAL,
}

# 光学系(z 朝前, x 朝右, y 朝下) -> 相机机械系(x 朝前) 的固定旋转.
# 等价于 URDF <origin rpy="-pi/2 0 -pi/2">. 仅在 apply_optical_rotation=True 时使用.
# R = Rz(-pi/2) * Ry(0) * Rx(-pi/2), 数值上把光学系向量表示到机械系.
_OPTICAL_TO_MECH_RPY = (-math.pi / 2.0, 0.0, -math.pi / 2.0)

# ---- 纯 numpy 图像编解码 (绕开 cv_bridge, 规避其与 NumPy 2.x C 扩展的二进制冲突) ----
# 只覆盖车载相机常见编码, 统一解码为 BGR uint8 供检测/绘制.
_BGR_FROM = {          # 编码 -> (通道数, 转 BGR 的 cvtColor code 或 None)
    'bgr8':  (3, None),
    'rgb8':  (3, cv2.COLOR_RGB2BGR),
    'bgra8': (4, cv2.COLOR_BGRA2BGR),
    'rgba8': (4, cv2.COLOR_RGBA2BGR),
    'mono8': (1, cv2.COLOR_GRAY2BGR),
}
# Bayer(原始工业相机) -> BGR. 注: 绿色打头的 gbrg/grbg 若 debug 图偏色再核对相机手册;
# 检测走灰度不受影响.
_BAYER_FROM = {
    'bayer_rggb8': cv2.COLOR_BAYER_BG2BGR,
    'bayer_bggr8': cv2.COLOR_BAYER_RG2BGR,
    'bayer_gbrg8': cv2.COLOR_BAYER_GB2BGR,
    'bayer_grbg8': cv2.COLOR_BAYER_GR2BGR,
}

def imgmsg_to_bgr(msg):
    """sensor_msgs/Image -> BGR uint8 ndarray (纯 numpy, 不依赖 cv_bridge).

    支持 bgr8/rgb8/bgra8/rgba8/mono8 及常见 bayer_*8. 正确处理行 step 填充.
    不支持的编码抛 ValueError, 由调用方节流告警.
    """
    enc = msg.encoding.lower()
    h, w = msg.height, msg.width
    buf = np.frombuffer(msg.data, dtype=np.uint8)

    if enc in _BGR_FROM:
        ch, cvt = _BGR_FROM[enc]
    elif enc in _BAYER_FROM:
        ch, cvt = 1, _BAYER_FROM[enc]
    else:
        raise ValueError("不支持的图像编码 '%s'" % msg.encoding)

    # 按 step 还原每行, 去掉行尾填充字节, 再裁到 w*ch
    step = msg.step if msg.step else w * ch
    if buf.size < step * h:
        raise ValueError("图像数据长度不足: 期望>=%d, 实际%d" % (step * h, buf.size))
    rows = buf[:step * h].reshape(h, step)
    img = rows[:, :w * ch].reshape(h, w, ch) if ch > 1 else rows[:, :w].reshape(h, w)

    return img if cvt is None else cv2.cvtColor(img, cvt)


def enhance_gray_for_aruco(gray, clip_limit=3.0, tile=8):
    """对灰度图做 CLAHE 局部对比度增强, 返回增强后的灰度图.

    用于红外/低对比成像: 打印件发灰 + 欠曝时, 黑白模块差异过小, ArUco 自适应阈值
    时检时不检. CLAHE(局部直方图均衡)按 tile 分块拉开局部对比, 对"信息在、只是灰"
    的图极有效 (实测 D435i 红外下检测率 0%->100%). 全局直方图拉伸对有高光的场景无效,
    故用局部自适应的 CLAHE. clip_limit 越大对比越强(噪声也放大), tile 为分块网格数.
    """
    clahe = cv2.createCLAHE(clipLimit=float(clip_limit),
                            tileGridSize=(int(tile), int(tile)))
    return clahe.apply(gray)


def make_aruco_detector(aruco_dict):
    """构造统一约定的 ArucoDetector. 标定/体检/车体三处共用, 保证检测参数一致.

    默认参数在【红外发灰】成像下时检时不检: ArUco 首步自适应阈值二值化, 灰模块与白底
    灰度差小, 单一窗口尺寸易分错. 这里针对性放宽二值化搜索 + 亚像素角点:

      - adaptiveThreshWinSizeMax 23->53, Step 10->6: 扩大并加密自适应阈值窗口, 让检测器
        对多种模块像素尺寸各试一遍, 发灰时命中率显著提高 (marker 占画面大时尤其需要大窗口).
      - adaptiveThreshConstant 7->5: 阈值偏置略降, 让偏灰的黑模块更易被判为黑.
      - minMarkerPerimeterRate 0.03->0.02: 放宽下限, 容忍更小/更远的 marker.
      - cornerRefinementMethod=SUBPIX: 亚像素角点细化, 标定/定位精度必需.

    代价: 搜索范围变大 -> 稍慢, 且理论上略增误检; 但本场景 (单个已知 id、静止/低速)
    可靠靠 marker_id 过滤误检, 慢一点无碍, 取舍偏向"宁可慢也要稳检出".
    彩色流(车体)不发灰, 这些放宽对其无害 (仅稍慢).
    """
    params = cv2.aruco.DetectorParameters()
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 53
    params.adaptiveThreshWinSizeStep = 6
    params.adaptiveThreshConstant = 5.0
    params.minMarkerPerimeterRate = 0.02
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(aruco_dict, params)


def bgr_to_imgmsg(img, header):
    """BGR uint8 ndarray -> sensor_msgs/Image (encoding=bgr8). 供 debug 图发布用."""
    msg = Image()
    msg.header = header
    msg.height, msg.width = img.shape[0], img.shape[1]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = img.shape[1] * 3
    msg.data = np.ascontiguousarray(img, dtype=np.uint8).tobytes()
    return msg


def rot_matrix_to_quat(R):
    """3x3 旋转矩阵 -> 四元数 (x, y, z, w). 标准 Shepperd 法, 数值稳定."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0  # s = 4*w
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0  # s = 4*x
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0  # s = 4*y
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0  # s = 4*z
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / n, y / n, z / n, w / n)


def rpy_to_rot_matrix(roll, pitch, yaw):
    """固定轴 RPY (与 URDF <origin rpy> 同约定: R = Rz*Ry*Rx) -> 3x3 旋转矩阵."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


class ArucoLocalizer(Node):
    def __init__(self):
        super().__init__('aruco_localizer')

        # ---- 参数声明 (全部可在 yaml/命令行覆盖) ----
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('parent_frame', 'Link_13_optical')
        self.declare_parameter('marker_size', 0.10)             # 标记边长(米), 按实际打印
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        # 空=接受所有 id; 非空=白名单. Humble 无法从空列表推断类型, 且带固定 type 的
        # 描述符遇空默认值会判为"未初始化", 故用 dynamic_typing 允许空/整型数组两种取值.
        self.declare_parameter(
            'marker_ids', [],
            ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('child_frame_prefix', 'aruco_')
        self.declare_parameter('apply_optical_rotation', False)  # True: 父系用 Link_13 并补光学旋转
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('debug_image_topic', '~/debug_image')
        self.declare_parameter('use_image_transport_best_effort', True)  # 图像走 BEST_EFFORT

        gp = self.get_parameter
        self.parent_frame = gp('parent_frame').value
        self.marker_size = float(gp('marker_size').value)
        self.child_prefix = gp('child_frame_prefix').value
        self.apply_optical_rotation = bool(gp('apply_optical_rotation').value)
        self.publish_debug = bool(gp('publish_debug_image').value)
        # marker_ids 允许配成 int 列表; 空列表/None(dynamic_typing 下空数组) 表示不过滤
        self.marker_whitelist = set(int(i) for i in (gp('marker_ids').value or []))

        dict_name = gp('aruco_dictionary').value
        if dict_name not in ARUCO_DICTS:
            raise ValueError(
                f"未知 aruco_dictionary='{dict_name}', 可选: {list(ARUCO_DICTS.keys())}")

        # ---- ArUco 检测器 (新版 API) ----
        # 走共享工厂: 统一发灰友好的二值化参数 + 亚像素角点, 与标定/体检节点一致.
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[dict_name])
        self._detector = make_aruco_detector(self._aruco_dict)

        # 若需补光学旋转, 预计算 R(机械系<-光学系) 备用
        self._R_mech_optical = rpy_to_rot_matrix(*_OPTICAL_TO_MECH_RPY)

        # ---- 相机内参缓存 (来自 camera_info) ----
        self._camera_matrix = None   # 3x3 K
        self._dist_coeffs = None     # 1xN 畸变
        self._cam_frame = None       # camera_info.header.frame_id (仅日志参考)
        self._info_warned = False    # 未收到内参时只告警一次

        # ---- QoS: 图像/相机信息一般走 BEST_EFFORT (传感器数据) ----
        if bool(gp('use_image_transport_best_effort').value):
            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
        else:
            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            )
        # camera_info 用 RELIABLE + VOLATILE: 兼容常规相机驱动(如 usb_cam, 发 VOLATILE).
        # 注: VOLATILE 订阅端能收 VOLATILE 与 TRANSIENT_LOCAL 两种发布端; 若用 TRANSIENT_LOCAL
        # 订阅则收不到 usb_cam 的 VOLATILE 发布 (DURABILITY 不兼容 -> 一直等 camera_info).
        # 常规驱动持续发 camera_info, 不依赖 latch, 故用 VOLATILE 无碍.
        info_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        image_topic = gp('image_topic').value
        info_topic = gp('camera_info_topic').value
        self._tf_broadcaster = TransformBroadcaster(self)
        self._info_sub = self.create_subscription(
            CameraInfo, info_topic, self._on_camera_info, info_qos)
        self._image_sub = self.create_subscription(
            Image, image_topic, self._on_image, sensor_qos)

        self._debug_pub = None
        if self.publish_debug:
            self._debug_pub = self.create_publisher(
                Image, gp('debug_image_topic').value, 1)

        self.get_logger().info(
            "aruco_localizer 启动: image='%s' camera_info='%s' 字典=%s 尺寸=%.3fm "
            "父系=%s 补光学旋转=%s 白名单=%s" % (
                image_topic, info_topic, dict_name, self.marker_size,
                self.parent_frame, self.apply_optical_rotation,
                sorted(self.marker_whitelist) if self.marker_whitelist else '全部'))
        if self.apply_optical_rotation and self.parent_frame == 'Link_13_optical':
            self.get_logger().warn(
                "apply_optical_rotation=True 但 parent_frame 仍是 Link_13_optical; "
                "补旋转模式下 parent_frame 应设为机械系 Link_13.")

    def _on_camera_info(self, msg: CameraInfo):
        """缓存相机内参. K 恒有效; 畸变系数按消息长度取, 无则按无畸变处理."""
        self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if msg.d is not None and len(msg.d) > 0:
            self._dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(1, -1)
        else:
            self._dist_coeffs = np.zeros((1, 5), dtype=np.float64)
        self._cam_frame = msg.header.frame_id
        # 内参一般不变, 收到即可退订以省资源
        if self._info_sub is not None:
            self.destroy_subscription(self._info_sub)
            self._info_sub = None
            self.get_logger().info(
                "已获取相机内参 (frame_id='%s', fx=%.1f fy=%.1f), 停止订阅 camera_info."
                % (self._cam_frame, self._camera_matrix[0, 0], self._camera_matrix[1, 1]))

    def _on_image(self, msg: Image):
        # 内参未就绪则跳过 (等 camera_info)
        if self._camera_matrix is None:
            self._warn_throttle('等待 camera_info, 暂不解算位姿...')
            return

        # ROS Image -> OpenCV BGR (纯 numpy 解码). 编码兼容彩色/灰度/bayer.
        try:
            frame = imgmsg_to_bgr(msg)
        except Exception as e:  # noqa: BLE001 - 编码不支持/数据异常, 统一记录
            self._warn_throttle('图像解码失败: %s' % e)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)
        if ids is None or len(ids) == 0:
            if self.publish_debug:
                self._publish_debug(frame, msg.header)
            return

        # 标记坐标系下的 4 个角点 3D 模型 (marker 平面, 原点在中心, z=0).
        # 顺序必须与 detectMarkers 输出角点一致: 左上, 右上, 右下, 左下 (光学系).
        h = self.marker_size / 2.0
        obj_pts = np.array([
            [-h,  h, 0.0],
            [ h,  h, 0.0],
            [ h, -h, 0.0],
            [-h, -h, 0.0],
        ], dtype=np.float64)

        stamp = msg.header.stamp
        transforms = []
        detected_ids = []
        for i, marker_id in enumerate(ids.flatten()):
            mid = int(marker_id)
            # 白名单过滤 (空=全部接受)
            if self.marker_whitelist and mid not in self.marker_whitelist:
                continue

            img_pts = corners[i].reshape(-1, 2).astype(np.float64)
            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, img_pts, self._camera_matrix, self._dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue

            # rvec(旋转向量) -> 3x3 旋转矩阵. R,t 描述 光学系 -> 标记系.
            R, _ = cv2.Rodrigues(rvec)
            t = tvec.reshape(3)

            # 若父系用机械系 Link_13, 左乘 R(机械系<-光学系) 把位姿搬到机械系.
            if self.apply_optical_rotation:
                R = self._R_mech_optical @ R
                t = self._R_mech_optical @ t

            qx, qy, qz, qw = rot_matrix_to_quat(R)

            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.parent_frame
            tf_msg.child_frame_id = '%s%d' % (self.child_prefix, mid)
            tf_msg.transform.translation.x = float(t[0])
            tf_msg.transform.translation.y = float(t[1])
            tf_msg.transform.translation.z = float(t[2])
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            transforms.append(tf_msg)
            detected_ids.append(mid)

        if transforms:
            # 一次性广播本帧所有标记, 时间戳一致
            self._tf_broadcaster.sendTransform(transforms)
            self._info_throttle('已广播 %d 个标记 TF: %s' % (len(detected_ids), detected_ids))

        if self.publish_debug:
            self._publish_debug(frame, msg.header, corners, ids)

    def _publish_debug(self, frame, header, corners=None, ids=None):
        """在图像上画出检测到的标记与坐标轴, 发到 debug_image_topic (仅调试)."""
        if self._debug_pub is None:
            return
        img = frame
        if ids is not None and len(ids) > 0:
            img = frame.copy()
            cv2.aruco.drawDetectedMarkers(img, corners, ids)
        try:
            out = bgr_to_imgmsg(img, header)
            self._debug_pub.publish(out)
        except Exception as e:  # noqa: BLE001
            self._warn_throttle('debug 图发布失败: %s' % e)

    def _warn_throttle(self, text, period_ns=2_000_000_000):
        """告警节流, 默认最多 2s 一条, 避免刷屏."""
        now = self.get_clock().now().nanoseconds
        last = getattr(self, '_last_warn_ns', 0)
        if now - last >= period_ns:
            self._last_warn_ns = now
            self.get_logger().warn(text)

    def _info_throttle(self, text, period_ns=2_000_000_000):
        now = self.get_clock().now().nanoseconds
        last = getattr(self, '_last_info_ns', 0)
        if now - last >= period_ns:
            self._last_info_ns = now
            self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

