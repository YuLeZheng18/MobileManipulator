"""手眼标定结果验证 (在线实测一致性, eye-in-hand).

不重新标定. 复用 hand_eye_calibrator 的采样设施 (摆臂/PnP/TF/相机/CLAHE),
采一批 (gripper2base, target2camera) 样本后, 用两套外参各自把固定 marker 反算到
base_frame 下:  base<-marker = g2b @ X @ t2c.  marker 不动, 各姿态结果理应一致,
【位置散布(std/max, mm)】即该外参的实测准确度 —— 越小越准.

对比两套 X:
  - 标定结果 X_calib   : hand_eye_result.yaml, 落在【红外光学系】, 直接用.
  - URDF 名义值 X_nom  : Joint_17 origin, 落在【机械系 Link_30】, PnP 是光学系观测,
                         故需补 光学->机械 旋转 M = rpy(-pi/2,0,-pi/2):
                         base<-marker = g2b @ X_nom @ M @ t2c.

用法:
  ros2 launch mm_perception hand_eye_verify.launch.py           # 默认 manual 采样
  按提示手动摆几个能看到 marker 的姿态, 回车采样, 最后 s 出对比报告.

前提: 真机 bringup (arm_controller + TF Link_20->Link_29) + 相机 infra1 + 固定 marker.
"""
import math

import numpy as np
import rclpy

from mm_perception.aruco_localizer import rpy_to_rot_matrix, _OPTICAL_TO_MECH_RPY
from mm_perception.hand_eye_calibrator import HandEyeCalibrator


def _homogeneous(R, t):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return M


class HandEyeVerify(HandEyeCalibrator):
    """继承标定器: 白拿采样(auto/manual)与相机/TF/PnP, 只把结算换成一致性验证."""

    def __init__(self):
        super().__init__()
        gp_declare = self.declare_parameter

        # 待验证外参 1: 标定结果 (光学系). 默认 = hand_eye_result.yaml 里的数值.
        gp_declare('calib_xyz', [0.008204, -0.042158, -0.047865])
        gp_declare('calib_rpy', [-3.111097, 0.292215, -1.564613])
        # 待验证外参 2: URDF 名义值 (机械系 Joint_17 origin).
        gp_declare('nominal_xyz', [0.05537897, 0.00725, -0.01913])
        gp_declare('nominal_rpy', [0.0, 1.57079633, -1.57079633])

        g = self.get_parameter
        self._calib_X = _homogeneous(
            rpy_to_rot_matrix(*[float(v) for v in g('calib_rpy').value]),
            [float(v) for v in g('calib_xyz').value])
        # 名义值(机械系) + 补 光学->机械 旋转, 使其能吃光学系 PnP 的 t2c.
        M_mech_optical = _homogeneous(rpy_to_rot_matrix(*_OPTICAL_TO_MECH_RPY),
                                      [0.0, 0.0, 0.0])
        self._nominal_X = _homogeneous(
            rpy_to_rot_matrix(*[float(v) for v in g('nominal_rpy').value]),
            [float(v) for v in g('nominal_xyz').value]) @ M_mech_optical

        self.get_logger().info(
            '验证模式: 采样后对比 [标定结果 X_calib] 与 [URDF名义值 X_nominal] 的 '
            'base<-marker 一致性散布 (越小越准).')

    def _wait_for_camera(self, timeout_s=15.0):
        """在等相机基础上, 再等 TF 树就位: tf2 listener 的 buffer 启动后需数秒累积,
        否则进采样循环第一次 lookup 必失败(Link_20 does not exist). 首采失败很误导."""
        if not super()._wait_for_camera(timeout_s):
            return False
        self.get_logger().info('等待 TF %s->%s 就位...' % (self.base_frame, self.ee_frame))
        t0 = self.get_clock().now().nanoseconds
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._tf_buffer.can_transform(
                    self.base_frame, self.ee_frame, rclpy.time.Time()):
                self.get_logger().info('TF 就位, 可开始采样.')
                return True
            if self.get_clock().now().nanoseconds - t0 > timeout_s * 1e9:
                self.get_logger().warn('TF 未就位(超时), 仍进采样; 首次若失败按回车重试即可.')
                return True
        return False

    def _scatter_for(self, X):
        """给定外参 X (gripper<-camera, 已对齐光学系), 算各姿态 base<-marker,
        返回位置散布(mm)与姿态散布(deg) dict. marker 固定, 散布小=外参自洽准确."""
        pts = []       # 各姿态 marker 在 base 下的平移 (m)
        zdirs = []     # 各姿态 marker z 轴在 base 下的朝向 (查姿态一致性)
        n = len(self.R_g2b)
        for k in range(n):
            g2b = _homogeneous(self.R_g2b[k], self.t_g2b[k])
            t2c = _homogeneous(self.R_t2c[k], self.t_t2c[k])
            base_marker = g2b @ X @ t2c
            pts.append(base_marker[:3, 3])
            zdirs.append(base_marker[:3, 2])   # marker 法向
        pts = np.array(pts)
        centroid = pts.mean(axis=0)
        dev = np.linalg.norm(pts - centroid, axis=1) * 1000.0   # 每姿态离质心距离 mm
        # 姿态散布: 各 z 轴与平均 z 轴夹角
        zmean = np.mean(zdirs, axis=0)
        zmean = zmean / (np.linalg.norm(zmean) + 1e-9)
        ang = []
        for z in zdirs:
            c = float(np.dot(z, zmean) / (np.linalg.norm(z) + 1e-9))
            ang.append(math.degrees(math.acos(max(-1.0, min(1.0, c)))))
        return {'n': n, 'centroid': centroid,
                'pos_std': float(dev.std()), 'pos_mean': float(dev.mean()),
                'pos_max': float(dev.max()),
                'rot_mean': float(np.mean(ang)), 'rot_max': float(np.max(ang))}

    def _report(self, tag, s):
        c = s['centroid']
        self.get_logger().info('  [%s]' % tag)
        self.get_logger().info(
            '    base<-marker 质心: [%.4f, %.4f, %.4f] m' % (c[0], c[1], c[2]))
        self.get_logger().info(
            '    位置散布: 均值 %.2f mm  最大 %.2f mm  (std %.2f mm)'
            % (s['pos_mean'], s['pos_max'], s['pos_std']))
        self.get_logger().info(
            '    姿态散布: 均值 %.3f°  最大 %.3f°' % (s['rot_mean'], s['rot_max']))

    def _solve_and_output(self):
        """override: 不标定, 用两套外参评估 base<-marker 一致性散布并对比."""
        n = len(self.R_g2b)
        if n < 2:
            self.get_logger().error('样本仅 %d 组 (<2), 无法评估散布. 多采几个姿态.' % n)
            return
        calib = self._scatter_for(self._calib_X)
        nom = self._scatter_for(self._nominal_X)

        self.get_logger().info('=' * 60)
        self.get_logger().info('手眼标定一致性验证 (%d 组姿态, base=%s)' % (n, self.base_frame))
        self.get_logger().info('-' * 60)
        self._report('标定结果 X_calib (hand_eye_result.yaml)', calib)
        self._report('URDF名义值 X_nominal (Joint_17)', nom)
        self.get_logger().info('-' * 60)
        better = '标定结果' if calib['pos_mean'] < nom['pos_mean'] else 'URDF名义值'
        self.get_logger().info(
            '结论: 位置散布更小(更自洽) = 【%s】. 散布<5mm 视为可用; 两者都大(>1cm)'
            '多半是臂零位漂移而非外参本身错.' % better)
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeVerify()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()
