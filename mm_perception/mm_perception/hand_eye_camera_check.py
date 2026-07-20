#!/usr/bin/env python3
"""手眼标定 - 相机侧可行性验证 (不用机械臂, 只用深度相机).

目的:
  手眼标定的数据由两半构成 —— gripper2base(查 TF) 与 target2camera(相机 solvePnP).
  本节点只验证 **相机这一半**: 深度相机能否稳定检测到那个固定 ArUco 标记,
  并用 solvePnP 解出可信的 相机->标记 位姿. 相机侧过关, 再上机械臂做完整标定
  (hand_eye_calibrator) 才有意义, 否则采到的 target2camera 是垃圾, 标定必失败.

与 hand_eye_calibrator._detect_target2camera 用同一套约定 (同字典/同 solvePnP flag/
同角点模型), 故这里能稳定检出 = 标定节点也能采到有效 target2camera.

验证内容 (逐帧累计, 终端持续打印, Ctrl-C 输出汇总):
  1. 检测率     : 收到的帧里能检出目标标记的比例 (低=标记太小/太远/光照差/id错).
  2. 位姿抖动   : 相机静止时, 解出的 xyz 的标准差 (mm 级抖动才够做标定).
  3. 重投影误差 : solvePnP 残差 (像素). 大=内参错/marker_size 填错/角点检测差.
  4. 距离/朝向  : 打印 相机->标记 的距离与 rpy, 肉眼核对是否合理.

用法:
  ros2 run mm_perception hand_eye_camera_check --ros-args \
    -p image_topic:=/camera/camera/color/image_raw \
    -p camera_info_topic:=/camera/camera/color/camera_info \
    -p marker_id:=0 -p marker_size:=0.10
  # 或用 launch: ros2 launch mm_perception hand_eye_camera_check.launch.py

不发任何话题/TF, 纯只读验证. 相机静止摆好、标记固定在视野内后观察输出即可.
"""
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import cv2
from sensor_msgs.msg import Image, CameraInfo

# 复用标定/车体节点同一套字典表、图像解码与旋转->rpy, 保证约定一致.
from mm_perception.aruco_localizer import (
    ARUCO_DICTS, imgmsg_to_bgr, enhance_gray_for_aruco, make_aruco_detector)
from mm_perception.hand_eye_calibrator import rot_matrix_to_rpy


class HandEyeCameraCheck(Node):
    def __init__(self):
        super().__init__('hand_eye_camera_check')

        # ---- 参数 (默认对齐 hand_eye_calib.yaml 的相机/标记设置) ----
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('image_is_best_effort', True)
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('marker_size', 0.10)   # 黑边实际边长(米), 必须与实物一致
        self.declare_parameter('marker_id', 0)         # 标定用的固定标记 id
        # CLAHE 局部对比度增强: 红外/低对比成像 (打印件发灰) 下检测率会从个位数飙到满分.
        # 彩色正常曝光时可关. clip_limit 越大对比越强, tile 为分块网格数.
        self.declare_parameter('equalize_clahe', True)
        self.declare_parameter('clahe_clip_limit', 3.0)
        self.declare_parameter('clahe_tile', 8)
        self.declare_parameter('report_period', 1.0)   # 终端刷新周期(秒)
        # 抖动统计窗口: 保留最近 N 组位姿算标准差 (相机静止时才有意义)
        self.declare_parameter('jitter_window', 30)
        # 可视化窗口: 开启后弹 OpenCV 窗口, 实时叠加检测框/坐标轴/HUD (需有显示器).
        # 关闭时行为不变, 仅终端打印文本报告. 窗口内按 q/ESC 退出, c 切 CLAHE.
        self.declare_parameter('show_window', False)
        # ---- 内参兜底 ----
        # 本机 D435i 彩色标定表损坏, camera_info 的 K 全是 NaN, solvePnP 无法用.
        # 这里允许外部注入内参: 当订阅到的 K 无效(NaN 或 fx<=0)时, 改用下面的值.
        # fx/fy<=0 表示不提供(仍等 camera_info); cx/cy<=0 表示用图像中心兜底 (w/2,h/2).
        # D435 彩色 640x480 标称约 fx=fy≈617; 精确值需棋盘格标定后回填, 勿用于交付级标定.
        self.declare_parameter('override_fx', 0.0)
        self.declare_parameter('override_fy', 0.0)
        self.declare_parameter('override_cx', 0.0)
        self.declare_parameter('override_cy', 0.0)

        gp = self.get_parameter
        self.marker_size = float(gp('marker_size').value)
        self.marker_id = int(gp('marker_id').value)
        self.equalize_clahe = bool(gp('equalize_clahe').value)
        self.clahe_clip = float(gp('clahe_clip_limit').value)
        self.clahe_tile = int(gp('clahe_tile').value)
        self.report_period = float(gp('report_period').value)
        self.jitter_window = int(gp('jitter_window').value)
        self.show_window = bool(gp('show_window').value)
        self._ov_fx = float(gp('override_fx').value)
        self._ov_fy = float(gp('override_fy').value)
        self._ov_cx = float(gp('override_cx').value)
        self._ov_cy = float(gp('override_cy').value)
        self._using_override = False   # 当前内参是否来自兜底 (报告里标注)

        dict_name = gp('aruco_dictionary').value
        if dict_name not in ARUCO_DICTS:
            raise ValueError("未知 aruco_dictionary='%s', 可选: %s"
                             % (dict_name, list(ARUCO_DICTS.keys())))
        adict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[dict_name])
        self._detector = make_aruco_detector(adict)   # 共享工厂: 发灰友好参数

        # 标记系下角点 3D 模型: 与标定节点完全一致 (中心为原点, z=0, 左上->右上->右下->左下)
        h = self.marker_size / 2.0
        self._obj_pts = np.array([
            [-h,  h, 0.0], [h,  h, 0.0], [h, -h, 0.0], [-h, -h, 0.0],
        ], dtype=np.float64)

        # ---- 相机数据 ----
        self._camera_matrix = None
        self._dist_coeffs = None
        self._cam_frame = None

        # ---- 统计累计 ----
        self._frames = 0        # 收到的图像帧数
        self._detected = 0      # 检出目标标记的帧数
        self._recent = []       # 最近 N 组 (x,y,z) 用于抖动统计
        self._last_pose = None  # (t[3], rpy[3], reproj_err)
        # 可视化: 缓存最新一帧的绘制输入 (原图, 全部角点/id, 目标标记 rvec/tvec 或 None).
        # 供 main 的窗口循环取用, 与文本报告共用同一次检测结果.
        self._vis = None

        img_qos = QoSProfile(
            reliability=(ReliabilityPolicy.BEST_EFFORT
                         if bool(gp('image_is_best_effort').value)
                         else ReliabilityPolicy.RELIABLE),
            history=HistoryPolicy.KEEP_LAST, depth=1)
        info_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE,
                              history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, gp('image_topic').value, self._on_image, img_qos)
        self.create_subscription(CameraInfo, gp('camera_info_topic').value,
                                 self._on_info, info_qos)
        self.create_timer(self.report_period, self._report)

        self.get_logger().info(
            "相机侧验证就绪: 标记 id=%d 边长=%.3fm 字典=%s. 等待图像与内参..."
            % (self.marker_id, self.marker_size, dict_name))

    def _on_info(self, msg: CameraInfo):
        self._cam_frame = msg.header.frame_id
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        fx, fy = K[0, 0], K[1, 1]
        info_valid = np.all(np.isfinite(K)) and fx > 0 and fy > 0

        if info_valid:
            self._camera_matrix = K
            if msg.d is not None and len(msg.d) > 0 and np.all(np.isfinite(msg.d)):
                self._dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(1, -1)
            else:
                self._dist_coeffs = np.zeros((1, 5), dtype=np.float64)
            self._using_override = False
        elif self._ov_fx > 0 and self._ov_fy > 0:
            # camera_info 无效, 用外部注入内参. cx/cy<=0 时用图像中心兜底.
            cx = self._ov_cx if self._ov_cx > 0 else msg.width / 2.0
            cy = self._ov_cy if self._ov_cy > 0 else msg.height / 2.0
            self._camera_matrix = np.array([[self._ov_fx, 0.0, cx],
                                            [0.0, self._ov_fy, cy],
                                            [0.0, 0.0, 1.0]], dtype=np.float64)
            self._dist_coeffs = np.zeros((1, 5), dtype=np.float64)  # 无畸变数据, 置零
            if not self._using_override:
                self.get_logger().warn(
                    'camera_info 内参无效(NaN/非正), 改用注入内参 fx=%.1f fy=%.1f cx=%.1f cy=%.1f. '
                    '仅供链路验证, 非交付级精度.' % (self._ov_fx, self._ov_fy, cx, cy))
            self._using_override = True
        else:
            # 无效且无兜底: 保持 None, 报告里提示
            if self._camera_matrix is None:
                self.get_logger().warn(
                    'camera_info 内参无效(NaN), 且未提供 override_fx/fy. '
                    '本机彩色标定损坏, 需棋盘格标定后注入或修设备.', throttle_duration_sec=5.0)

    def _on_image(self, msg: Image):
        if self._camera_matrix is None:
            return  # 等内参
        try:
            frame = imgmsg_to_bgr(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn('图像解码失败: %s' % e)
            return
        self._frames += 1

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.equalize_clahe:
            gray = enhance_gray_for_aruco(gray, self.clahe_clip, self.clahe_tile)
        corners, ids, _ = self._detector.detectMarkers(gray)

        # 未检出目标标记: 仍缓存原图+已检出的其它标记, 让窗口能显示画面.
        if ids is None or self.marker_id not in ids.flatten().tolist():
            if self.show_window:
                self._vis = (frame, corners, ids, None, None)
            return
        ids_list = ids.flatten().tolist()
        idx = ids_list.index(self.marker_id)

        img_pts = corners[idx].reshape(-1, 2).astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(
            self._obj_pts, img_pts, self._camera_matrix, self._dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            if self.show_window:
                self._vis = (frame, corners, ids, None, None)
            return

        self._detected += 1
        R, _ = cv2.Rodrigues(rvec)
        t = tvec.reshape(3)
        roll, pitch, yaw = rot_matrix_to_rpy(R)
        reproj = self._reproj_error(img_pts, rvec, tvec)

        self._recent.append(t.copy())
        if len(self._recent) > self.jitter_window:
            self._recent.pop(0)
        self._last_pose = (t, (roll, pitch, yaw), reproj)
        if self.show_window:
            self._vis = (frame, corners, ids, rvec, tvec)

    def _reproj_error(self, img_pts, rvec, tvec):
        """把模型角点按解出的位姿投回像素, 与实测角点比, 返回 RMS 像素误差."""
        proj, _ = cv2.projectPoints(
            self._obj_pts, rvec, tvec, self._camera_matrix, self._dist_coeffs)
        proj = proj.reshape(-1, 2)
        return float(np.sqrt(np.mean(np.sum((proj - img_pts) ** 2, axis=1))))

    def _report(self):
        if self._camera_matrix is None:
            self.get_logger().warn('尚未收到 camera_info, 确认内参话题名与相机驱动.')
            return
        if self._frames == 0:
            self.get_logger().warn('已有内参但没收到图像, 确认 image_topic 与 QoS.')
            return

        rate = 100.0 * self._detected / max(1, self._frames)
        src = '注入(非交付级)' if self._using_override else 'camera_info'
        lines = ['---- 相机侧手眼可行性 ----',
                 '  内参来源: %s' % src,
                 '  检测率: %.1f%% (%d/%d 帧检出 id=%d)'
                 % (rate, self._detected, self._frames, self.marker_id)]

        if self._last_pose is None:
            lines.append('  尚未检出目标标记: 核对 id/字典/marker_size, 让标记完整入镜.')
            self.get_logger().info('\n'.join(lines))
            return

        t, rpy, reproj = self._last_pose
        dist = float(np.linalg.norm(t))
        lines.append('  相机->标记: xyz=[%.4f, %.4f, %.4f] m  距离=%.3f m' %
                     (t[0], t[1], t[2], dist))
        lines.append('  朝向 rpy° = [%.2f, %.2f, %.2f]' %
                     (math.degrees(rpy[0]), math.degrees(rpy[1]), math.degrees(rpy[2])))
        lines.append('  重投影 RMS = %.3f px  %s' %
                     (reproj, '(优<1, 可用<2, 需查>3)' if reproj else ''))

        if len(self._recent) >= 2:
            arr = np.array(self._recent)
            std_mm = arr.std(axis=0) * 1000.0
            lines.append('  位姿抖动 σ(最近%d帧) = [%.2f, %.2f, %.2f] mm  %s'
                         % (len(self._recent), std_mm[0], std_mm[1], std_mm[2],
                            '(相机静止时应 <2mm)'))

        # 逐项体检结论
        verdict = []
        if rate < 80:
            verdict.append('检测率偏低')
        if reproj > 3.0:
            verdict.append('重投影过大(查内参/marker_size)')
        if len(self._recent) >= 5 and (np.array(self._recent).std(axis=0) * 1000).max() > 5:
            verdict.append('抖动过大')
        lines.append('  结论: ' + ('相机侧可用于手眼标定 ✓' if not verdict
                                   else '需处理: ' + ', '.join(verdict)))
        self.get_logger().info('\n'.join(lines))

    def render_frame(self):
        """把最新缓存帧画成带标注的 BGR (检测框+坐标轴+HUD). 无帧时返回 None.
        与文本报告共用同一次检测结果, 不重复跑检测/solvePnP."""
        if self._vis is None:
            return None
        frame, corners, ids, rvec, tvec = self._vis
        vis = frame.copy()
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(vis, corners, ids)
        if rvec is not None:
            cv2.drawFrameAxes(vis, self._camera_matrix, self._dist_coeffs,
                              rvec, tvec, self.marker_size * 0.5, 2)

        rate = 100.0 * self._detected / max(1, self._frames)
        src = '注入(非交付级)' if self._using_override else 'camera_info'
        green, red = (80, 220, 80), (60, 60, 235)

        def put(y, txt, color=(240, 240, 240)):
            cv2.putText(vis, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
            cv2.putText(vis, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)

        put(24, 'K:%s  CLAHE:%s  id=%d size=%.3f'
            % (src, 'ON' if self.equalize_clahe else 'OFF',
               self.marker_id, self.marker_size))
        put(48, 'detect rate: %.1f%% (%d/%d)' % (rate, self._detected, self._frames),
            green if rate >= 80 else red)
        if self._last_pose is None:
            put(72, 'target id=%d NOT seen' % self.marker_id, red)
            return vis
        t, rpy, reproj = self._last_pose
        put(72, 'cam->marker xyz=[%.3f %.3f %.3f]  d=%.3fm'
            % (t[0], t[1], t[2], float(np.linalg.norm(t))))
        put(96, 'rpy deg=[%.1f %.1f %.1f]'
            % (math.degrees(rpy[0]), math.degrees(rpy[1]), math.degrees(rpy[2])))
        put(120, 'reproj RMS=%.2f px' % reproj, green if reproj < 2 else red)
        if len(self._recent) >= 2:
            std_mm = np.array(self._recent).std(axis=0) * 1000.0
            put(144, 'jitter sigma=[%.2f %.2f %.2f] mm'
                % (std_mm[0], std_mm[1], std_mm[2]),
                green if std_mm.max() < 2.0 else red)
        return vis


def _final_summary(node):
    if node._frames:
        rate = 100.0 * node._detected / max(1, node._frames)
        node.get_logger().info(
            '最终: 检测率 %.1f%% (%d/%d). %s'
            % (rate, node._detected, node._frames,
               '相机侧就绪, 可上机械臂做完整标定.' if rate >= 80
               else '检测率不足, 先解决标记可见性再标定.'))


def _spin_with_window(node):
    """开窗口模式: 手动 spin + OpenCV 显示. q/ESC 退出, c 切 CLAHE."""
    win = 'hand-eye camera check (q=quit, c=toggle CLAHE)'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        vis = node.render_frame()
        if vis is not None:
            cv2.imshow(win, vis)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            break
        if key == ord('c'):
            node.equalize_clahe = not node.equalize_clahe
    cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeCameraCheck()
    try:
        if node.show_window:
            _spin_with_window(node)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        _final_summary(node)   # 退出前打印最终汇总
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
