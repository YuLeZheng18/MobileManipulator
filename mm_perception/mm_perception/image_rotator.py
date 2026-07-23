#!/usr/bin/env python3
"""图像旋转校正节点: 相机装反(上下颠倒/侧装)时把话题流转正.

usb_cam 无旋转参数. 本节点订阅 image_raw(+camera_info), 按 rotation 参数
旋转 0/90/180/270 度后重新发布, 同时正确变换 camera_info 的 K/P(主点/焦距/尺寸),
使下游 ArUco/标定/监视看到的都是转正且内参自洽的图像.

每个相机起一个实例, 各配自己的 rotation. 例:
  cam_a(ArUco, 上下颠倒) -> rotation:=180
  cam_b(监视, 装反)      -> rotation:=180

用法(单相机快速测):
  ros2 run mm_perception image_rotator --ros-args \
    -p rotation:=180 \
    -r image_in:=/cam_a/image_raw -r info_in:=/cam_a/camera_info \
    -r image_out:=/cam_a/image_rot -r info_out:=/cam_a/camera_info_rot
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

_CV_ROT = {
    90: cv2.ROTATE_90_CLOCKWISE,
    180: cv2.ROTATE_180,
    270: cv2.ROTATE_90_COUNTERCLOCKWISE,
}


class ImageRotator(Node):
    def __init__(self):
        super().__init__('image_rotator')
        self.declare_parameter('rotation', 180)
        self.declare_parameter('use_best_effort', True)
        rot = int(self.get_parameter('rotation').value) % 360
        if rot not in (0, 90, 180, 270):
            self.get_logger().warn(f'rotation={rot} 非 0/90/180/270, 归零不旋转')
            rot = 0
        self.rot = rot
        self.bridge = CvBridge()
        self._info_out = None  # 缓存变换后的 camera_info(尺寸不变时只算一次)

        # 输入 QoS 跟相机驱动 (多数发 BEST_EFFORT); 输出恒 RELIABLE.
        # DDS 兼容规则: RELIABLE 发布者对 RELIABLE 和 BEST_EFFORT 订阅者都兼容, 反之不行.
        # image_rot 只在 Nano 本地被消费 (ArUco best_effort + image_transport republish
        # reliable -> compressed 过网监视), reliable 无 WiFi 成本却能同时喂饱两者.
        sub_qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST)
        sub_qos.reliability = (ReliabilityPolicy.BEST_EFFORT
                               if self.get_parameter('use_best_effort').value
                               else ReliabilityPolicy.RELIABLE)
        pub_qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST)
        pub_qos.reliability = ReliabilityPolicy.RELIABLE

        self.pub_img = self.create_publisher(Image, 'image_out', pub_qos)
        self.pub_info = self.create_publisher(CameraInfo, 'info_out', pub_qos)
        self.create_subscription(Image, 'image_in', self._on_image, sub_qos)
        self.create_subscription(CameraInfo, 'info_in', self._on_info, sub_qos)
        self.get_logger().info(f'image_rotator 启动: rotation={self.rot}°')

    def _on_image(self, msg: Image):
        if self.rot == 0:
            self.pub_img.publish(msg)
            return
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        img = cv2.rotate(img, _CV_ROT[self.rot])
        out = self.bridge.cv2_to_imgmsg(img, encoding=msg.encoding)
        out.header = msg.header
        self.pub_img.publish(out)

    def _on_info(self, msg: CameraInfo):
        if self.rot == 0:
            self.pub_info.publish(msg)
            return
        self.pub_info.publish(self._rotate_info(msg))

    def _rotate_info(self, msg: CameraInfo) -> CameraInfo:
        if self._info_out is not None and self._info_out.header.frame_id == msg.header.frame_id:
            self._info_out.header.stamp = msg.header.stamp
            return self._info_out
        W, H = msg.width, msg.height
        K = list(msg.k)
        fx, fy, cx, cy = K[0], K[4], K[2], K[5]
        if self.rot == 180:
            nW, nH = W, H
            nfx, nfy, ncx, ncy = fx, fy, W - 1 - cx, H - 1 - cy
        elif self.rot == 90:   # 顺时针
            nW, nH = H, W
            nfx, nfy, ncx, ncy = fy, fx, H - 1 - cy, cx
        else:                  # 270 顺时针 = 逆时针 90
            nW, nH = H, W
            nfx, nfy, ncx, ncy = fy, fx, cy, W - 1 - cx
        out = CameraInfo()
        out.header = msg.header
        out.height, out.width = nH, nW
        out.distortion_model = msg.distortion_model
        out.d = msg.d  # 畸变默认 0; 非零时切向系数理论需换轴, 标定阶段在转正流上重标即可
        out.k = [nfx, 0.0, ncx, 0.0, nfy, ncy, 0.0, 0.0, 1.0]
        out.r = msg.r
        out.p = [nfx, 0.0, ncx, 0.0, 0.0, nfy, ncy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._info_out = out
        return out


def main():
    rclpy.init()
    node = ImageRotator()
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
