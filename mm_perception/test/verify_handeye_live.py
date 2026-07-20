#!/usr/bin/env python3
"""手眼标定结果 - 实物闭环验证 (只读, 不改 URDF/不发 TF).

原理: 固定 marker 在 base_link 下的位姿是【常量】. 对每一帧:
    base<-marker = (base<-Link_29 : 查TF) · (Link_29<-Link_30 : 待验外参) · (Link_30<-marker : solvePnP)
若外参正确, 则无论臂摆到什么姿态, 反推出的 base<-marker 都应【高度一致】(xyz 散布小);
外参错则各姿态反推结果散开. 用一致性直接判定标定可信度, 无需量盒子/卷尺.

同时用两组外参各算一遍做对照:
  - CALIB : 本次手眼标定结果 (光学系), 内部换算到 Link_30 机械系;
  - NOMINAL: URDF Joint_17 现有名义值 (CAD).
哪组在多姿态下更一致, 哪组更可信.

用法 (真机 TF 树 + 相机就绪, 投射器关):
  python3 mm_perception/test/verify_handeye_live.py
你在 RViz/MoveIt 把臂摆到多个差异大的姿态, 每个停稳后看打印的 base<-marker;
脚本还累计每组外参的 running 均值与散布 (std), Ctrl-C 输出汇总.
"""
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import cv2
import tf2_ros
from sensor_msgs.msg import Image, CameraInfo, JointState

from mm_perception.aruco_localizer import (
    ARUCO_DICTS, imgmsg_to_bgr, enhance_gray_for_aruco, make_aruco_detector,
    rpy_to_rot_matrix, _OPTICAL_TO_MECH_RPY)
from mm_perception.hand_eye_calibrator import quat_to_rot_matrix, rot_matrix_to_rpy

# ---- 待验外参 (Link_29 -> Link_30) ----
# CALIB: 本次标定结果在【光学系】, 换算到 Link_30 机械系 (原点不变, 旋转右乘 R(光学->机械)).
_CALIB_XYZ = [0.008204, -0.042158, -0.047865]
_CALIB_RPY_OPTICAL = [-3.111097, 0.292215, -1.564613]
# NOMINAL: URDF Joint_17 现有名义值 (机械系).
_NOMINAL_XYZ = [0.05537897, 0.00725, -0.01913]
_NOMINAL_RPY = [0.0, 1.57079633, -1.57079633]


def _homog(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).reshape(3)
    return T


def _calib_T_mech():
    """标定光学系结果 -> Link_29->Link_30 机械系齐次矩阵."""
    R_opt = rpy_to_rot_matrix(*_CALIB_RPY_OPTICAL)
    R_mech_opt = rpy_to_rot_matrix(*_OPTICAL_TO_MECH_RPY)   # R(机械<-光学)
    R_mech = R_opt @ R_mech_opt.T                           # R_cal · R(光学->机械)
    return _homog(R_mech, _CALIB_XYZ)


def _nominal_T():
    return _homog(rpy_to_rot_matrix(*_NOMINAL_RPY), _NOMINAL_XYZ)


class Accum:
    """累计一组外参反推的 base<-marker 位置, 算均值/散布."""
    def __init__(self, name):
        self.name = name
        self.pts = []

    def add(self, p):
        self.pts.append(np.asarray(p).reshape(3))

    def stats(self):
        if len(self.pts) < 1:
            return None
        a = np.array(self.pts)
        return a.mean(axis=0), a.std(axis=0), len(self.pts)


class Verify(Node):
    def __init__(self):
        super().__init__('verify_handeye_live')
        self.declare_parameter('image_topic', '/camera/camera/infra1/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/infra1/camera_info')
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('marker_size', 0.135)
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ee_frame', 'Link_29')
        self.declare_parameter('clahe_clip', 3.0)
        self.declare_parameter('clahe_tile', 8)
        self.declare_parameter('report_period', 1.0)
        # 快照模式: 采到 snapshot_n 个有效样本后, 打印均值并自动退出 (0=持续运行不退).
        self.declare_parameter('snapshot_n', 0)

        gp = self.get_parameter
        self.marker_size = float(gp('marker_size').value)
        self.marker_id = int(gp('marker_id').value)
        self.base_frame = gp('base_frame').value
        self.ee_frame = gp('ee_frame').value
        self.clahe_clip = float(gp('clahe_clip').value)
        self.clahe_tile = int(gp('clahe_tile').value)
        self.snapshot_n = int(gp('snapshot_n').value)
        self.done = False   # 快照采够后置 True, main 里据此退出

        adict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[gp('aruco_dictionary').value])
        self._detector = make_aruco_detector(adict)
        h = self.marker_size / 2.0
        self._obj = np.array([[-h, h, 0.], [h, h, 0.], [h, -h, 0.], [-h, -h, 0.]])

        self._K = None
        self._dist = None
        self._frame = None

        # 光学系 -> Link_30 机械系: solvePnP 出的是光学系点, 外参 T 是 Link_29->机械系,
        # 故 Link_30(机械)<-marker = R(机械<-光学) · (光学<-marker).
        self._R_mech_opt = rpy_to_rot_matrix(*_OPTICAL_TO_MECH_RPY)

        self._T_calib = _calib_T_mech()
        self._T_nominal = _nominal_T()
        # 纯光学路径: 标定结果本就是 Link_29->光学系, solvePnP 也直接出光学系, 两者直接链乘,
        # 完全不碰 _R_mech_opt 转换. 若这条一致而机械系路径漂 => 是转换 bug 不是 FK/标定.
        self._T_calib_opt = _homog(rpy_to_rot_matrix(*_CALIB_RPY_OPTICAL), _CALIB_XYZ)
        self._acc_calib = Accum('CALIB(机械系换算)')
        self._acc_nom = Accum('NOMINAL(URDF名义)')
        self._acc_calib_opt = Accum('CALIB(纯光学)')

        img_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        info_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE,
                              history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, gp('image_topic').value, self._on_img, img_qos)
        self.create_subscription(CameraInfo, gp('camera_info_topic').value, self._on_info, info_qos)
        # 记录当前关节角, 单关节实验时把 marker 漂移和关节转角对应起来.
        self._joints = None
        self._joint_names = None
        js_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(JointState, '/joint_states', self._on_joints, js_qos)
        self._tf_buf = tf2_ros.Buffer()
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf, self)
        self.create_timer(float(gp('report_period').value), self._report)
        self.get_logger().info('实物验证就绪: 摆多个姿态, 看两组外参反推的 base<-marker 一致性.')

    def _on_joints(self, msg):
        self._joints = list(msg.position)
        self._joint_names = list(msg.name)

    def _on_info(self, msg):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if np.all(np.isfinite(K)) and K[0, 0] > 0:
            self._K = K
            self._dist = (np.array(msg.d).reshape(1, -1)
                          if msg.d is not None and len(msg.d) > 0 else np.zeros((1, 5)))

    def _on_img(self, msg):
        if self._K is None:
            return
        try:
            self._frame = imgmsg_to_bgr(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn('解码失败: %s' % e)

    def _cam_from_marker(self):
        """solvePnP -> (机械系<-marker, 光学系<-marker) 两个齐次矩阵, 或 None."""
        if self._frame is None or self._K is None:
            return None
        gray = cv2.cvtColor(self._frame, cv2.COLOR_BGR2GRAY)
        gray = enhance_gray_for_aruco(gray, self.clahe_clip, self.clahe_tile)
        corners, ids, _ = self._detector.detectMarkers(gray)
        if ids is None or self.marker_id not in ids.flatten().tolist():
            return None
        idx = ids.flatten().tolist().index(self.marker_id)
        ok, rvec, tvec = cv2.solvePnP(self._obj, corners[idx].reshape(-1, 2).astype(np.float64),
                                      self._K, self._dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None
        R_opt, _ = cv2.Rodrigues(rvec)          # 光学系 <- marker
        T_opt = _homog(R_opt, tvec.reshape(3))
        R_mech = self._R_mech_opt @ R_opt       # 机械系 <- marker
        t_mech = self._R_mech_opt @ tvec.reshape(3)
        return _homog(R_mech, t_mech), T_opt

    def _base_from_ee(self):
        try:
            tf = self._tf_buf.lookup_transform(self.base_frame, self.ee_frame, rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        q = tf.transform.rotation
        t = tf.transform.translation
        return _homog(quat_to_rot_matrix(q.x, q.y, q.z, q.w), [t.x, t.y, t.z])

    def _report(self):
        T_be = self._base_from_ee()
        cm = self._cam_from_marker()
        if T_be is None:
            self.get_logger().warn('查不到 TF %s<-%s' % (self.base_frame, self.ee_frame))
            return
        if cm is None:
            self.get_logger().warn('未检出 marker id=%d (核对可见性/投射器关)' % self.marker_id)
            return
        T_cm, T_cm_opt = cm
        # base<-marker = base<-ee · ee<-cam(外参) · cam<-marker
        p_cal = (T_be @ self._T_calib @ T_cm)[:3, 3]
        p_nom = (T_be @ self._T_nominal @ T_cm)[:3, 3]
        # 纯光学: ee<-光学(标定原始) · 光学<-marker(solvePnP原始), 不碰 _R_mech_opt
        p_cal_opt = (T_be @ self._T_calib_opt @ T_cm_opt)[:3, 3]
        self._acc_calib.add(p_cal)
        self._acc_nom.add(p_nom)
        self._acc_calib_opt.add(p_cal_opt)

        # 诊断: cam<-marker 的距离与朝向. 换姿态时若 rpy 突跳几十度 => solvePnP 平面歧义(法向翻转).
        t_cm = T_cm[:3, 3]
        rcm = rot_matrix_to_rpy(T_cm[:3, :3])

        def fmt(p):
            return '[%.4f, %.4f, %.4f]' % (p[0], p[1], p[2])
        lines = ['---- base<-marker (越稳=外参越准) ----',
                 '  cam<-marker: d=%.3fm rpy°=[%.1f,%.1f,%.1f]'
                 % (float(np.linalg.norm(t_cm)),
                    math.degrees(rcm[0]), math.degrees(rcm[1]), math.degrees(rcm[2])),
                 '  CALIB(机械): %s' % fmt(p_cal),
                 '  CALIB(光学): %s' % fmt(p_cal_opt),
                 '  NOMINAL    : %s' % fmt(p_nom)]
        for acc in (self._acc_calib, self._acc_calib_opt, self._acc_nom):
            st = acc.stats()
            if st and st[2] >= 2:
                mean, std, n = st
                lines.append('  [%s] n=%d 均值%s σ=[%.4f,%.4f,%.4f] |σ|=%.4fm'
                             % (acc.name, n, fmt(mean), std[0], std[1], std[2],
                                float(np.linalg.norm(std))))
        self.get_logger().info('\n'.join(lines))
        # 快照模式: 采够 snapshot_n 个有效样本, 标记退出.
        if self.snapshot_n > 0 and len(self._acc_calib.pts) >= self.snapshot_n:
            self.done = True


def main():
    rclpy.init()
    node = Verify()
    try:
        if node.snapshot_n > 0:
            # 快照: spin 到 done 或超时, 自动退出并打印汇总 (不靠外部 timeout).
            end = node.get_clock().now().nanoseconds + int(60 * 1e9)
            while rclpy.ok() and not node.done:
                rclpy.spin_once(node, timeout_sec=0.1)
                if node.get_clock().now().nanoseconds > end:
                    break
            print('\n===== 快照汇总 =====')
            for acc in (node._acc_calib, node._acc_calib_opt, node._acc_nom):
                st = acc.stats()
                if st:
                    mean, std, n = st
                    print('[%s] n=%d 均值=[%.4f,%.4f,%.4f] σ=[%.4f,%.4f,%.4f] |σ|=%.4fm'
                          % (acc.name, n, mean[0], mean[1], mean[2],
                             std[0], std[1], std[2], float(np.linalg.norm(std))))
            if node._joints is not None:
                print('关节角 rad = [%s]' % ', '.join('%.4f' % v for v in node._joints))
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            return 0
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n===== 最终一致性汇总 (σ 越小外参越准) =====')
        for acc in (node._acc_calib, node._acc_calib_opt, node._acc_nom):
            st = acc.stats()
            if st:
                mean, std, n = st
                print('[%s] n=%d 均值=[%.4f,%.4f,%.4f] σ=[%.4f,%.4f,%.4f] |σ|=%.4fm'
                      % (acc.name, n, mean[0], mean[1], mean[2],
                         std[0], std[1], std[2], float(np.linalg.norm(std))))
        print('结论: |σ| 更小的那组外参在多姿态下更自洽, 更可信.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main() or 0)
