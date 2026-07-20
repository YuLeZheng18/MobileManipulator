"""抓取节点 (MVP): YOLO 位姿 -> MoveIt 粗定位 -> Cartesian 下插 -> 气泵吸取 -> 抬起.

契约 §1/§5 (MVP, 暂不含 §5 中段 moveit_servo 视觉伺服精修):
  订阅 /perception/object_pose (PoseStamped, base_link 系, 盒顶中心 xyz+yaw, YOLO 发).
  1. 预抓取(闭环 MoveIt): TCP 规划到盒顶上方 pre_height.
  2. 末段(开环 Cartesian): 从预抓取位沿 -Z 相对直插 pre_height 行程贴顶面.
     严禁末段重算绝对坐标 (契约 §5 纪律): 直插是相对预抓取位的短行程.
  3. 吸取: 发 /pump_cmd=SUCK; 抬起回预抓取高度(带载).
靠 [预抓取闭环 + 短行程末段 + 吸盘机械容差] 吃 ~18mm 零位残差(见手眼验证结论).

TCP: 规划目标用 Link_29 (SRDF 链末端), z 目标加 suction_offset(0.095) 换算到吸盘尖.
  (实测: compute_cartesian_path 不支持链外固定延伸 suction_tip 会挂死, 故用 Link_29.)
姿态: 吸盘竖直向下 = quat(0,0,sin(yaw/2),cos(yaw/2)) (契约 §1). 吸盘轴对称,
  yaw 自动挑可达腕角 (box_yaw+0/90/180/270 等效).

前提(本节点不代起): 真机 bringup(arm_controller) + move_group + yolo_box_detector + 气泵固件.
真机会真动: 执行前确认吸盘朝下、周围无遮挡、急停就位.
"""
import math
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
import rclpy.time

from pymoveit2 import MoveIt2

# 气泵命令 (对 firmware/chassis/include/config.h)
PUMP_STOP, PUMP_SUCK, PUMP_RELEASE = 0, 1, 2

ARM_JOINTS = ['Joint_11', 'Joint_12', 'Joint_13', 'Joint_14', 'Joint_15', 'Joint_16']


class GraspNode(Node):
    def __init__(self):
        super().__init__('grasp_node')
        gp = self.declare_parameter
        gp('object_pose_topic', '/perception/object_pose')
        gp('pump_topic', '/pump_cmd')
        gp('base_frame', 'Link_20')          # MoveIt 规划基座 (SRDF chain base)
        # 规划用 Link_29 (SRDF 链末端) 而非 suction_tip: compute_cartesian_path 不支持
        # 链外固定延伸(会挂死); 普通 IK 虽支持 suction_tip 但 Cartesian 步骤(下插/抬起)
        # 必须用 Link_29. 吸盘恒竖直向下 -> Link_29 在 suction_tip 正上方 suction_offset.
        gp('tcp_link', 'Link_29')            # 规划目标 link (SRDF 链末端)
        gp('suction_offset', 0.095)          # Link_29 -> suction_tip 沿 -Z 距离(米)
        gp('pre_height', 0.12)               # 预抓取: 盒顶上方高度(米)
        gp('contact_gap', 0.0)               # 末段: 吸盘距盒顶最终间隙(米, 0=贴面)
        gp('pose_timeout', 2.0)              # 位姿新鲜度阈值(秒)
        gp('max_velocity', 0.3)              # 真机保守限速
        gp('max_acceleration', 0.3)
        gp('suck_settle', 1.0)               # 吸取稳定等待(秒)
        gp('auto_grasp', False)              # 启动即抓一次(否则等 /grasp 服务)
        gp('dry_run', False)                 # 只规划不执行(无真机验证用)
        gp('move_only', False)               # 只移到预抓取位就停: 不下插/不吸/不抬(真机运动验证)
        gp('j1_tolerance', 1.0)              # J1路径约束容差(rad,±); 防基座翻180°镜像解. <=0关闭

        g = self.get_parameter
        self.pose_topic = g('object_pose_topic').value
        self.base_frame = g('base_frame').value
        self.tcp_link = g('tcp_link').value
        self.suction_offset = float(g('suction_offset').value)
        self.pre_height = float(g('pre_height').value)
        self.contact_gap = float(g('contact_gap').value)
        self.pose_timeout = float(g('pose_timeout').value)
        self.suck_settle = float(g('suck_settle').value)
        self.dry_run = bool(g('dry_run').value)
        self.move_only = bool(g('move_only').value)
        self.j1_tolerance = float(g('j1_tolerance').value)

        self._latest_pose = None             # (PoseStamped, 收到时刻 ns)
        cb = ReentrantCallbackGroup()
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(PoseStamped, self.pose_topic,
                                 self._on_pose, sensor_qos, callback_group=cb)
        self._pump_pub = self.create_publisher(
            Int8, g('pump_topic').value,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=1))

        self.moveit2 = MoveIt2(
            node=self, joint_names=ARM_JOINTS,
            base_link_name=self.base_frame, end_effector_name=self.tcp_link,
            group_name='arm', callback_group=cb)
        self.moveit2.max_velocity = float(g('max_velocity').value)
        self.moveit2.max_acceleration = float(g('max_acceleration').value)

        # 读当前腕部 yaw, 挑最近可达角(吸盘轴对称, 腕部尽量不转)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_service(Trigger, '/grasp', self._on_grasp_srv, callback_group=cb)
        self.get_logger().info(
            'grasp_node 就绪: 订阅 %s, TCP=%s, 预抓取高%.2fm, dry_run=%s. '
            '发 /grasp 触发.' % (self.pose_topic, self.tcp_link, self.pre_height,
                                 self.dry_run))

    def _on_pose(self, msg: PoseStamped):
        self._latest_pose = (msg, self.get_clock().now().nanoseconds)

    def _fresh_pose(self):
        """取新鲜位姿; 超时/无位姿返回 None."""
        if self._latest_pose is None:
            self.get_logger().warn('尚未收到 %s' % self.pose_topic)
            return None
        msg, t = self._latest_pose
        age = (self.get_clock().now().nanoseconds - t) / 1e9
        if age > self.pose_timeout:
            self.get_logger().warn('位姿过期 %.1fs (>%.1fs), 拒绝抓取' % (age, self.pose_timeout))
            return None
        return msg

    @staticmethod
    def _yaw_down_quat(yaw):
        """吸盘竖直向下 + 绕竖直轴 yaw (契约 §1). 返回 (x,y,z,w)."""
        return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))

    @staticmethod
    def _yaw_from_quat(q):
        """从盒子位姿四元数取 yaw (绕 z)."""
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _constrain_j1(self):
        """把 J1(Joint_11)路径约束在当前值附近, 防止 IK 选到基座翻 180°的镜像解
        (末端位置对但整条臂绕到反侧). 容差 j1_tolerance 够到目标又不许翻转.
        约束每次规划后被 pymoveit2 自动清, 故每次 plan/move 前都要重设."""
        if self.j1_tolerance <= 0:
            return
        js = self.moveit2.joint_state
        if js is None or 'Joint_11' not in js.name:
            return
        j1 = js.position[js.name.index('Joint_11')]
        self.moveit2.set_path_joint_constraint(
            joint_positions=[j1], joint_names=['Joint_11'],
            tolerance=self.j1_tolerance)

    def _move(self, position, quat, cartesian, frame_id):
        """规划+执行到 TCP 位姿; dry_run 只规划. 返回 True/False.
        frame_id: 目标位姿所在坐标系 (= object_pose 的 header 帧, 通常 base_link);
        MoveIt 内部按 TF 转到规划基座(Link_20). 注意 base_link≠Link_20(差90°+平移),
        故绝不能把 base_link 坐标贴 Link_20 标签."""
        if self.dry_run:
            self._constrain_j1()
            traj = self.moveit2.plan(
                position=position, quat_xyzw=quat, target_link=self.tcp_link,
                frame_id=frame_id, cartesian=cartesian)
            ok = traj is not None
            self.get_logger().info('  [dry_run] 规划%s: %s'
                                   % ('(cartesian)' if cartesian else '', '成功' if ok else '失败'))
            return ok
        self._constrain_j1()
        self.moveit2.move_to_pose(
            position=position, quat_xyzw=quat, target_link=self.tcp_link,
            frame_id=frame_id, cartesian=cartesian)
        return self.moveit2.wait_until_executed()

    def _pump(self, cmd):
        if self.dry_run:
            self.get_logger().info('  [dry_run] 气泵 %d (跳过)' % cmd)
            return
        self._pump_pub.publish(Int8(data=int(cmd)))

    def _current_wrist_yaw(self, frame_id):
        """当前 TCP(Link_29) 绕竖直轴的 yaw, 用于挑最近腕角; 查不到返回 None."""
        try:
            tf = self._tf_buffer.lookup_transform(
                frame_id, self.tcp_link, rclpy.time.Time())
            q = tf.transform.rotation
            return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        except Exception as e:
            self.get_logger().warn('查当前腕部 yaw 失败(%s), 退回盒yaw顺序' % e)
            return None

    def _pick_reachable_yaw(self, position, box_yaw, frame_id):
        """吸盘轴对称: box_yaw+0/90/180/270 物理等效, 都能吸. 优先选离当前腕部
        最近的可达角(腕部尽量不转, 避免多余/反向转动). 全不可达返回 None."""
        offs = [0.0, math.pi / 2, math.pi, -math.pi / 2]
        cur = self._current_wrist_yaw(frame_id)
        if cur is not None:
            # 按候选角与当前腕角的环形距离升序排列, 最近的先试
            def ang_dist(off):
                d = (box_yaw + off) - cur
                return abs(math.atan2(math.sin(d), math.cos(d)))
            offs.sort(key=ang_dist)
        for off in offs:
            yaw = box_yaw + off
            quat = self._yaw_down_quat(yaw)
            self._constrain_j1()
            traj = self.moveit2.plan(
                position=position, quat_xyzw=quat, target_link=self.tcp_link,
                frame_id=frame_id, cartesian=False)
            if traj is not None:
                self.get_logger().info(
                    '  选定腕部 yaw=%.1f° (盒yaw%+.0f°, 当前腕%s)'
                    % (math.degrees(yaw), math.degrees(off),
                       '%.0f°' % math.degrees(cur) if cur is not None else 'NA'))
                return quat
        return None

    def grasp_once(self):
        """执行一次抓取序列. 返回 (成功?, 说明)."""
        pose = self._fresh_pose()
        if pose is None:
            return False, '无新鲜目标位姿'
        p = pose.pose.position
        frame = pose.header.frame_id or 'base_link'   # 目标位姿所在系(base_link)
        box_yaw = self._yaw_from_quat(pose.pose.orientation)
        # Link_29 目标 = 吸盘接触点 + suction_offset(吸盘竖直下 -> Link_29 在其正上方).
        z_off = self.suction_offset
        self.get_logger().info(
            '目标盒顶中心 %s=[%.3f, %.3f, %.3f] 盒yaw=%.1f° (Link_29 目标 z+%.3f)'
            % (frame, p.x, p.y, p.z, math.degrees(box_yaw), z_off))

        # 1. 预抓取: 盒顶上方 pre_height. 吸盘轴对称->挑一个可达的腕部 yaw
        #    (box_yaw+0/90/180/270 物理等效; 某些 yaw 腕角不可达, 选第一个能规划的).
        pre = [p.x, p.y, p.z + self.pre_height + z_off]
        quat = self._pick_reachable_yaw(pre, box_yaw, frame)
        if quat is None:
            return False, '预抓取位所有等效 yaw 都不可达(位置够不着?)'
        self.get_logger().info('[1/4] 规划到预抓取位 %s' % [round(v, 3) for v in pre])
        if not self._move(pre, quat, cartesian=False, frame_id=frame):
            return False, '预抓取规划/执行失败'

        if self.move_only:
            self.get_logger().info('move_only: 已到预抓取位, 停(不下插/不吸/不抬)')
            return True, '已移到预抓取位(move_only)'

        # 2. 末段: 相对预抓取沿 -Z 直插到贴顶面 (短行程, 不重算绝对坐标)
        contact = [p.x, p.y, p.z + self.contact_gap + z_off]
        self.get_logger().info('[2/4] Cartesian 下插到 %s' % [round(v, 3) for v in contact])
        if not self._move(contact, quat, cartesian=True, frame_id=frame):
            return False, '下插失败(未吸取)'

        # 3. 吸取
        self.get_logger().info('[3/4] 气泵吸取')
        self._pump(PUMP_SUCK)
        self._sleep(self.suck_settle)

        # 4. 抬起回预抓取高度(带载)
        self.get_logger().info('[4/4] 抬起')
        if not self._move(pre, quat, cartesian=True, frame_id=frame):
            return False, '抬起失败(已吸取, 需人工处理)'
        return True, '抓取完成'

    def _sleep(self, secs):
        end = self.get_clock().now().nanoseconds + int(secs * 1e9)
        while self.get_clock().now().nanoseconds < end and rclpy.ok():
            self.create_rate(50.0).sleep()

    def _on_grasp_srv(self, req, resp):
        ok, msg = self.grasp_once()
        resp.success = ok
        resp.message = msg
        self.get_logger().info('抓取结果: %s (%s)' % (ok, msg))
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = GraspNode()
    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    t = Thread(target=executor.spin, daemon=True)
    t.start()
    node.create_rate(1.0).sleep()
    if node.get_parameter('auto_grasp').value:
        ok, msg = node.grasp_once()
        node.get_logger().info('auto_grasp: %s (%s)' % (ok, msg))
    try:
        t.join()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
