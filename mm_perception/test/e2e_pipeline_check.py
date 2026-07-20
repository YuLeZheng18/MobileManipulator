#!/usr/bin/env python3
"""端到端离线自测: 合成一张正对相机的 ArUco 图 -> 走完整管线, 验证:
  1. 纯 numpy 解码 imgmsg_to_bgr 不再段错误 (cv_bridge 已移除)
  2. detectMarkers 能检出标记
  3. solvePnP 解出的 tvec/四元数数值合理
不需要 ROS 运行时, 直接 import 节点模块里的纯函数即可.
"""
import math
import sys

import numpy as np
import cv2

from mm_perception.aruco_localizer import (
    imgmsg_to_bgr, bgr_to_imgmsg, rot_matrix_to_quat, ARUCO_DICTS,
)
from std_msgs.msg import Header

class FakeImg:
    """最小化仿 sensor_msgs/Image: 只带管线用到的字段, 避免拉起 rclpy."""
    def __init__(self, img_bgr, header):
        self.height, self.width = img_bgr.shape[0], img_bgr.shape[1]
        self.encoding = 'bgr8'
        self.step = self.width * 3
        self.data = np.ascontiguousarray(img_bgr, dtype=np.uint8).tobytes()
        self.header = header


def main():
    marker_size = 0.10          # 米, 与默认参数一致
    dict_name = 'DICT_4X4_50'
    marker_id = 7
    img_px = 480                # 合成方图边长(像素)

    # 1) 生成一张标记图, 贴到画布中央 (四周留白, 检测更稳)
    adict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[dict_name])
    marker_px = 300
    marker = cv2.aruco.generateImageMarker(adict, marker_id, marker_px)
    canvas = np.full((img_px, img_px), 255, dtype=np.uint8)
    off = (img_px - marker_px) // 2
    canvas[off:off + marker_px, off:off + marker_px] = marker
    frame_bgr = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)

    # 2) 走纯 numpy 解码 (关键: 这一步以前用 cv_bridge 会段错误)
    header = Header()
    header.frame_id = 'Link_13'
    msg = FakeImg(frame_bgr, header)
    decoded = imgmsg_to_bgr(msg)
    assert decoded.shape == (img_px, img_px, 3), "解码 shape 不对: %s" % (decoded.shape,)
    assert np.array_equal(decoded, frame_bgr), "解码内容与原图不一致"
    print("[1/4] 纯 numpy 解码 OK, shape =", decoded.shape, "(无段错误)")

    # 3) 检测标记
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(adict, params)
    gray = cv2.cvtColor(decoded, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    assert ids is not None and marker_id in ids.flatten(), "未检出目标标记 %d" % marker_id
    print("[2/4] detectMarkers OK, 检出 ids =", ids.flatten().tolist())

    # 4) solvePnP 解位姿. 用一组合理的假内参 (fx=fy=600, 主点在中心)
    K = np.array([[600.0, 0, img_px / 2.0],
                  [0, 600.0, img_px / 2.0],
                  [0, 0, 1.0]], dtype=np.float64)
    dist = np.zeros((1, 5), dtype=np.float64)
    h = marker_size / 2.0
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
    idx = list(ids.flatten()).index(marker_id)
    img_pts = corners[idx].reshape(-1, 2).astype(np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, img_pts, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    assert ok, "solvePnP 失败"
    R, _ = cv2.Rodrigues(rvec)
    t = tvec.reshape(3)

    # 正对相机: x,y 应接近 0; z>0 且量级合理 (marker 300px, fx=600, size=0.1 -> z≈0.2m)
    assert abs(t[0]) < 0.02 and abs(t[1]) < 0.02, "tvec x/y 偏离中心过大: %s" % t
    assert 0.1 < t[2] < 0.4, "tvec z 不在合理范围: %.3f" % t[2]
    qx, qy, qz, qw = rot_matrix_to_quat(R)
    qn = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    assert abs(qn - 1.0) < 1e-6, "四元数未归一化: |q|=%.6f" % qn
    print("[3/4] solvePnP OK, tvec(m) = [%.4f %.4f %.4f]" % (t[0], t[1], t[2]))
    print("[4/4] 四元数 OK, (x,y,z,w) = (%.4f %.4f %.4f %.4f), |q|=%.6f" % (qx, qy, qz, qw, qn))

    # 附带验证 debug 图编码往返
    back = imgmsg_to_bgr(bgr_to_imgmsg(decoded, header))
    assert np.array_equal(back, decoded), "编码->解码往返不一致"
    print("[+]  debug 图 bgr<->imgmsg 往返 OK")
    print("\n全部通过: 绕开 cv_bridge 后整条管线可用, 无段错误.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
