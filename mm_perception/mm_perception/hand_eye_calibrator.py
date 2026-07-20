#!/usr/bin/env python3
"""手眼标定节点 (eye-in-hand): 求 Link_29(腕部) -> Link_30(深度相机) 外参.

方法 (对应参考脚本 111 的 cv2.calibrateHandEye, 棋盘格换成固定 ArUco):
  1. 预存 N 组关节角, 逐组用 FollowJointTrajectory 让臂自动摆到位;
  2. 每组到位后采一对数据:
       gripper2base  : 查 TF base_frame(Link_20) -> ee_frame(Link_29)
       target2camera : 对深度相机(Link_30)图像跑 solvePnP 得 相机->固定ArUco;
  3. cv2.calibrateHandEye(R/t_gripper2base, R/t_target2camera)
       -> camera2gripper 即 Link_29 -> Link_30 外参;
  4. 输出 x y z roll pitch yaw (6个数), 用于修正 URDF Joint_17 origin (交集成者回填).

约束: 只在 mm_perception 内; 不改 mm_description / mm_navigation / arm.
标定阶段可用打印的 static_transform_publisher 命令临时验证 (见结束输出).
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import cv2
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image, CameraInfo
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import tf2_ros

# 复用车体 ArUco 节点的字典表与纯 numpy 图像解码 (同一套约定, 避免 cv_bridge)
from mm_perception.aruco_localizer import (
    ARUCO_DICTS, imgmsg_to_bgr, enhance_gray_for_aruco, make_aruco_detector)

def quat_to_rot_matrix(x, y, z, w):
    """四元数 (x,y,z,w) -> 3x3 旋转矩阵. 用于把 TF 姿态转成 calibrateHandEye 要的 R."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def rot_matrix_to_rpy(R):
    """3x3 旋转矩阵 -> 固定轴 RPY (与 URDF <origin rpy> 同约定 R=Rz*Ry*Rx). 返回 (roll,pitch,yaw)."""
    sy = -R[2, 0]
    sy = max(-1.0, min(1.0, sy))
    pitch = math.asin(sy)
    if abs(sy) < 0.99999:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:  # 万向锁: pitch=±90°, roll/yaw 退化, 固定 roll=0
        roll = 0.0
        yaw = math.atan2(-R[0, 1], R[1, 1])
    return roll, pitch, yaw


class HandEyeCalibrator(Node):
    def __init__(self):
        super().__init__('hand_eye_calibrator')

        # ---- 参数 ----
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('image_is_best_effort', True)
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('marker_size', 0.10)
        self.declare_parameter('marker_id', 0)
        # CLAHE 局部对比度增强: 红外/低对比成像下大幅提升 ArUco 检出率. 须与 camera_check 一致.
        self.declare_parameter('equalize_clahe', True)
        self.declare_parameter('clahe_clip_limit', 3.0)
        self.declare_parameter('clahe_tile', 8)
        self.declare_parameter('base_frame', 'Link_20')
        self.declare_parameter('ee_frame', 'Link_29')
        self.declare_parameter('arm_action', '/arm_controller/follow_joint_trajectory')
        self.declare_parameter('joint_names',
                               ['Joint_11', 'Joint_12', 'Joint_13',
                                'Joint_14', 'Joint_15', 'Joint_16'])
        self.declare_parameter('move_time', 4.0)
        self.declare_parameter('settle_time', 1.5)
        # 采样模式: auto=预存姿态自动摆臂; manual=你手动把臂摆到位, 按回车逐组采样.
        self.declare_parameter('sampling_mode', 'auto')
        # 展平的一维 float 数组, 每 6 个为一组关节角 (ROS2 参数不支持嵌套数组).
        # dynamic_typing 允许空默认值 (否则空数组无法推断类型).
        self.declare_parameter('calib_poses', [],
                               ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('min_samples', 6)     # calibrateHandEye 至少需 3, 稳健建议 >=6
        self.declare_parameter('output_path', 'hand_eye_result.yaml')

        gp = self.get_parameter
        self.marker_size = float(gp('marker_size').value)
        self.marker_id = int(gp('marker_id').value)
        self.equalize_clahe = bool(gp('equalize_clahe').value)
        self.clahe_clip = float(gp('clahe_clip_limit').value)
        self.clahe_tile = int(gp('clahe_tile').value)
        self.base_frame = gp('base_frame').value
        self.ee_frame = gp('ee_frame').value
        self.joint_names = list(gp('joint_names').value)
        self.move_time = float(gp('move_time').value)
        self.settle_time = float(gp('settle_time').value)
        self.min_samples = int(gp('min_samples').value)
        self.sampling_mode = str(gp('sampling_mode').value).lower()
        if self.sampling_mode not in ('auto', 'manual'):
            raise ValueError("未知 sampling_mode='%s' (可选 auto/manual)" % self.sampling_mode)
        self.output_path = gp('output_path').value

        dict_name = gp('aruco_dictionary').value
        if dict_name not in ARUCO_DICTS:
            raise ValueError("未知 aruco_dictionary='%s'" % dict_name)
        adict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[dict_name])
        self._detector = make_aruco_detector(adict)   # 共享工厂: 发灰友好参数

        self._poses = self._load_poses()

        # ---- 相机数据缓存 ----
        self._camera_matrix = None
        self._dist_coeffs = None
        self._latest_frame = None    # 最新一帧 BGR

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

        # ---- TF 与机械臂 action ----
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._arm = ActionClient(self, FollowJointTrajectory, gp('arm_action').value)

        # 采样累积: calibrateHandEye 要的四组列表
        self.R_g2b, self.t_g2b, self.R_t2c, self.t_t2c = [], [], [], []

        self.get_logger().info(
            "hand_eye_calibrator 就绪: %d 组姿态, 标记id=%d 边长=%.3fm, base=%s ee=%s"
            % (len(self._poses), self.marker_id, self.marker_size,
               self.base_frame, self.ee_frame))

    def _load_poses(self):
        """calib_poses 参数是嵌套 list, 但 ROS2 参数只接受平面数组.
        兼容两种: 已展平的 float 列表 (每6个一组) 或字符串数组."""
        raw = self.get_parameter('calib_poses').value
        if not raw:
            return []
        flat = [float(v) for v in raw]
        if len(flat) % 6 != 0:
            raise ValueError('calib_poses 元素数 %d 不是 6 的倍数' % len(flat))
        return [flat[i:i + 6] for i in range(0, len(flat), 6)]

    def _on_image(self, msg: Image):
        try:
            self._latest_frame = imgmsg_to_bgr(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn('图像解码失败: %s' % e)

    def _on_info(self, msg: CameraInfo):
        self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if msg.d is not None and len(msg.d) > 0:
            self._dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(1, -1)
        else:
            self._dist_coeffs = np.zeros((1, 5), dtype=np.float64)

    def _lookup_gripper2base(self):
        """查 TF base_frame -> ee_frame, 返回 (R 3x3, t 3,) 或 None."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame, rclpy.time.Time())
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn('查 TF %s->%s 失败: %s'
                                   % (self.base_frame, self.ee_frame, e))
            return None
        tr = tf.transform.translation
        q = tf.transform.rotation
        R = quat_to_rot_matrix(q.x, q.y, q.z, q.w)
        t = np.array([tr.x, tr.y, tr.z])
        return R, t

    def _detect_target2camera(self):
        """对最新帧检测固定 ArUco, solvePnP 得 相机->标记, 返回 (R,t) 或 None."""
        if self._latest_frame is None or self._camera_matrix is None:
            self.get_logger().warn('尚无图像或相机内参, 跳过该姿态')
            return None
        gray = cv2.cvtColor(self._latest_frame, cv2.COLOR_BGR2GRAY)
        if self.equalize_clahe:
            gray = enhance_gray_for_aruco(gray, self.clahe_clip, self.clahe_tile)
        corners, ids, _ = self._detector.detectMarkers(gray)
        if ids is None:
            self.get_logger().warn('未检测到任何标记')
            return None
        ids = ids.flatten().tolist()
        if self.marker_id not in ids:
            self.get_logger().warn('未检测到目标标记 id=%d (看到 %s)' % (self.marker_id, ids))
            return None
        idx = ids.index(self.marker_id)
        h = self.marker_size / 2.0
        obj = np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])
        img_pts = corners[idx].reshape(-1, 2).astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(obj, img_pts, self._camera_matrix,
                                      self._dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            self.get_logger().warn('solvePnP 失败')
            return None
        R, _ = cv2.Rodrigues(rvec)
        return R, tvec.reshape(3)

    def _move_to(self, joints):
        """发 FollowJointTrajectory 让臂运动到一组关节角, 阻塞等到完成."""
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = list(self.joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in joints]
        secs = int(self.move_time)
        pt.time_from_start = Duration(sec=secs,
                                      nanosec=int((self.move_time - secs) * 1e9))
        traj.points = [pt]
        goal.trajectory = traj

        send_future = self._arm.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('轨迹目标被拒绝')
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result() is not None

    def _spin_seconds(self, seconds):
        """稳定等待期间继续处理回调 (刷新图像/TF)."""
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while self.get_clock().now().nanoseconds < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self):
        """按 sampling_mode 分派: auto=自动摆臂, manual=手动摆位+键盘采样."""
        if self.sampling_mode == 'manual':
            return self._run_manual()
        return self._run_auto()

    def _wait_for_camera(self, timeout_s=15.0):
        """等相机内参与首帧图像就绪; 超时返回 False."""
        self.get_logger().info('等待相机内参与首帧图像...')
        t0 = self.get_clock().now().nanoseconds
        while (self._camera_matrix is None or self._latest_frame is None) and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().nanoseconds - t0 > timeout_s * 1e9:
                self.get_logger().error('%.0fs 内没等到相机数据, 确认深度相机话题名与驱动.'
                                        % timeout_s)
                return False
        return True

    def _run_auto(self):
        """自动摆臂: 逐姿态摆到位 -> 采样 -> 求解 -> 输出."""
        if not self._poses:
            self.get_logger().error('未配置 calib_poses, 无法标定. 请在 yaml 填预存姿态.')
            return
        self.get_logger().info('等待机械臂 action server...')
        if not self._arm.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('连接不到 arm_controller action, 确认真机 bringup 已起.')
            return
        if not self._wait_for_camera():
            return

        for i, joints in enumerate(self._poses):
            self.get_logger().info('[%d/%d] 运动到姿态...' % (i + 1, len(self._poses)))
            if not self._move_to(joints):
                self.get_logger().warn('姿态 %d 运动失败, 跳过' % (i + 1))
                continue
            self._spin_seconds(self.settle_time)   # 稳定 + 刷新数据
            if self._take_sample():
                self.get_logger().info('姿态 %d 采样成功 (已累计 %d 组)'
                                       % (i + 1, len(self.R_g2b)))
            else:
                self.get_logger().warn('姿态 %d 采样无效, 跳过' % (i + 1))

        self._solve_and_output()

    def _take_sample(self):
        """采一对 (gripper2base, target2camera) 存入累积列表. 成功返回 True."""
        g2b = self._lookup_gripper2base()
        if g2b is None:            # TF 可能刚发布还没进 buffer, 再刷一下重试一次
            self._spin_seconds(0.5)
            g2b = self._lookup_gripper2base()
        t2c = self._detect_target2camera()
        if g2b is None or t2c is None:
            return False
        self.R_g2b.append(g2b[0]); self.t_g2b.append(g2b[1])
        self.R_t2c.append(t2c[0]); self.t_t2c.append(t2c[1])
        return True

    def _run_manual(self):
        """手动模式: 你把臂摆到位, 终端按键采样. 不发轨迹、不需 calib_poses.
        按 回车=采当前姿态, s=求解输出, q=放弃退出. 摆臂方式(手扳/GUI/示教)不限."""
        if not self._wait_for_camera():
            return
        self.get_logger().info(
            '手动采样模式: 摆好一个姿态(确保相机能看到标记)后 ->\n'
            '  [回车] 采样   [s + 回车] 求解并输出   [q + 回车] 退出')
        while rclpy.ok():
            self._spin_seconds(0.2)          # 先刷新最新图像/TF
            try:
                cmd = input('采样(回车)/求解(s)/退出(q) > ').strip().lower()
            except EOFError:
                break
            if cmd == 'q':
                self.get_logger().info('已退出, 不求解.')
                return
            if cmd == 's':
                break
            if self._take_sample():
                self.get_logger().info('采样成功 (已累计 %d 组)' % len(self.R_g2b))
            else:
                self.get_logger().warn('采样无效: TF 或标记未就绪, 换个姿态重试.')
        self._solve_and_output()

    @staticmethod
    def _homogeneous(R, t):
        M = np.eye(4)
        M[:3, :3] = R
        M[:3, 3] = np.asarray(t).reshape(3)
        return M

    def _eval_error(self, R_cg, t_cg):
        """标定一致性残差 (AX=XB): 衡量本次标定的实际精度.

        对每一对姿态 (i,j): A = 腕部相对运动 (g2b_j^-1 @ g2b_i),
        B = 标记相对相机运动 (t2c_j @ t2c_i^-1). 理想下 A@X == X@B.
        用解出的 X 算 残差 D = (A@X)^-1 @ (X@B), 其平移范数(mm)与旋转角(deg)
        即该对的偏差. 名义外参错/零位不稳/PnP 噪声都体现在这里.

        返回 dict: 平移/旋转残差的 均值 与 最大值 (mm / deg), 及对数 npairs.
        """
        X = self._homogeneous(R_cg, t_cg)
        Xinv = np.linalg.inv(X)
        g2b = [self._homogeneous(self.R_g2b[k], self.t_g2b[k])
               for k in range(len(self.R_g2b))]
        t2c = [self._homogeneous(self.R_t2c[k], self.t_t2c[k])
               for k in range(len(self.R_t2c))]

        trans_err, rot_err = [], []
        n = len(g2b)
        for i in range(n):
            for j in range(i + 1, n):
                A = np.linalg.inv(g2b[j]) @ g2b[i]     # 腕部相对运动
                B = t2c[j] @ np.linalg.inv(t2c[i])     # 标记相对相机运动
                D = np.linalg.inv(A @ X) @ (X @ B)     # 理想为单位阵
                trans_err.append(np.linalg.norm(D[:3, 3]) * 1000.0)  # mm
                # 旋转残差角: acos((tr(R)-1)/2)
                c = (np.trace(D[:3, :3]) - 1.0) / 2.0
                c = max(-1.0, min(1.0, c))
                rot_err.append(math.degrees(math.acos(c)))

        if not trans_err:
            return None
        te, re = np.array(trans_err), np.array(rot_err)
        return {'npairs': len(trans_err),
                'trans_mean': float(te.mean()), 'trans_max': float(te.max()),
                'rot_mean': float(re.mean()), 'rot_max': float(re.max())}

    def _solve_and_output(self):
        n = len(self.R_g2b)
        if n < self.min_samples:
            self.get_logger().error(
                '有效样本仅 %d 组 (<%d), 无法可靠标定. 增加姿态数或检查标记可见性.'
                % (n, self.min_samples))
            return

        # calibrateHandEye: eye-in-hand 传 gripper2base + target2camera,
        # 返回 camera2gripper (即 Link_29 -> Link_30 的外参, 契约 §3 要的).
        R_cg, t_cg = cv2.calibrateHandEye(
            self.R_g2b, self.t_g2b, self.R_t2c, self.t_t2c,
            method=cv2.CALIB_HAND_EYE_TSAI)

        t = t_cg.reshape(3)
        roll, pitch, yaw = rot_matrix_to_rpy(R_cg)

        # ---- 标定误差评估 ----
        err = self._eval_error(R_cg, t_cg)
        self.get_logger().info('=' * 56)
        self.get_logger().info('手眼标定完成 (%d 组样本). Link_29 -> Link_30 外参:' % n)
        self.get_logger().info(
            '  xyz  = [%.6f, %.6f, %.6f]  (米)' % (t[0], t[1], t[2]))
        self.get_logger().info(
            '  rpy  = [%.6f, %.6f, %.6f]  (弧度)' % (roll, pitch, yaw))
        self.get_logger().info(
            '  rpy° = [%.3f, %.3f, %.3f]  (度)'
            % (math.degrees(roll), math.degrees(pitch), math.degrees(yaw)))
        if err:
            self.get_logger().info('-' * 56)
            self.get_logger().info(
                '标定一致性残差 (AX=XB, %d 对姿态):' % err['npairs'])
            self.get_logger().info(
                '  平移: 均值 %.2f mm  最大 %.2f mm' % (err['trans_mean'], err['trans_max']))
            self.get_logger().info(
                '  旋转: 均值 %.3f°  最大 %.3f°' % (err['rot_mean'], err['rot_max']))
            verdict = ('优 (<2mm)' if err['trans_mean'] < 2.0
                       else '可用 (<5mm)' if err['trans_mean'] < 5.0
                       else '偏大, 查零位稳定性/PnP/标记尺寸 (>5mm)')
            self.get_logger().info('  评价: %s' % verdict)
        self._write_result(t, (roll, pitch, yaw), n, err)
        self.get_logger().info('=' * 56)
        self.get_logger().info(
            '交付: 用上面 xyz/rpy 替换 mm_robot.urdf 中 Joint_17 的 <origin>. (本节点不改 URDF)')

    def _write_result(self, t, rpy, n, err=None):
        """把结果写成 URDF 片段 + yaml, 交付给集成者回填 Joint_17."""
        err_block = ''
        if err:
            err_block = (
                '# 标定一致性残差 (AX=XB, %d 对姿态):\n'
                '#   平移 均值 %.2f mm / 最大 %.2f mm\n'
                '#   旋转 均值 %.3f° / 最大 %.3f°\n'
                % (err['npairs'], err['trans_mean'], err['trans_max'],
                   err['rot_mean'], err['rot_max']))
        text = (
            '# 手眼标定结果 (Link_29 -> Link_30), 样本 %d 组.\n'
            '# 用法: 替换 mm_robot.urdf 中 Joint_17 的 origin.\n'
            '%s'
            'Joint_17_origin:\n'
            '  xyz: [%.6f, %.6f, %.6f]\n'
            '  rpy: [%.6f, %.6f, %.6f]\n'
            '# 对应 URDF 写法:\n'
            '# <origin xyz="%.6f %.6f %.6f" rpy="%.6f %.6f %.6f"/>\n'
            % (n, err_block, t[0], t[1], t[2], rpy[0], rpy[1], rpy[2],
               t[0], t[1], t[2], rpy[0], rpy[1], rpy[2]))
        try:
            with open(self.output_path, 'w') as f:
                f.write(text)
            self.get_logger().info('结果已写入: %s' % self.output_path)
        except OSError as e:
            self.get_logger().warn('结果写文件失败: %s' % e)


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeCalibrator()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
