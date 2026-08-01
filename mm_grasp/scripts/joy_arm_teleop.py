#!/usr/bin/env python3
"""手柄三态遥控: 收拢 / 底盘行进 / 臂点动抓取 (路线图 D2 收尾).

一个手柄管全车, 靠三个状态互斥 —— 臂伸出来的时候车不能动, 车在走的时候臂不能动。

    joy_node -> /joy ─┬─> teleop_twist_joy -> /cmd_vel_joy ─┐
                      │                                     ├─> 本节点 -> /cmd_vel
                      └─> 本节点 (状态机 + 臂点动)  ─────────┘

状态机 (上电即 HOME, 臂收拢, 车臂都不许动):

    HOME ──START(9)──> DRIVE <──SELECT(8)──> ARM
     臂 home            臂 ready              臂 look
     车禁止             车放行                车禁止

    DRIVE: /cmd_vel_joy 原样转发到 /cmd_vel。臂停在 ready (收身, 不拖着伸出的臂走)。
    ARM:   掐掉转发并补发一帧零速 (不补的话底盘保持最后一个速度, 要等固件
           CMD_TIMEOUT_MS 500ms 才失效保护刹停), 摇杆改喷 moveit_servo。
    ARM->DRIVE 切换时先 /grasp/ready 把臂收回来才放行底盘, 顺序不能反。

R1(btn 5) 是两个状态通用的死人开关: DRIVE 下它是 teleop_twist_joy 的 enable_button
(那边 yaml 配的), ARM 下它是点动使能 —— 同一个键在互斥状态里各司其职, 不冲突。

⚠️ 安全: 臂模式下 servo 直接往 /arm_controller/joint_trajectory 喷, **绕过 MoveIt
的碰撞检查**。约束盒 (下面的 box_* 参数) 只钳位 TCP 的 x/y/z, **管不了腕姿态** ——
十字键转 roll/pitch 时没有任何几何防护, △ 键回正是唯一兜底。且盒顶/四角当前是占位
值, 第一次真机点动务必架空或把 servo_scale_* 调到很小起步, 别贴着围栏试。

⚠️ 十字键在 DragonRise 手柄上是**轴不是按键** (2026-08-01 实测: axis 5=上下,
axis 4=左右), 故 pitch/roll 按轴读。

⚠️ 本节点的 box_z_min 与 grasp_node 的 pregrasp_height 是**两份独立副本** (ROS 参数
不跨节点共享)。改了要两边一起改, 否则约束盒底与真实抓取高度对不上。
"""
import threading

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

HOME, DRIVE, ARM = 'HOME', 'DRIVE', 'ARM'


class JoyArmTeleop(Node):

    def __init__(self):
        super().__init__('joy_arm_teleop')
        cb = ReentrantCallbackGroup()

        # ---- 按键号 (DragonRise 手柄, 2026-08-01 真机 /joy 实测全部确认) ----
        self.btn_start = self.declare_parameter('btn_start', 9).value       # START: HOME->DRIVE
        self.btn_select = self.declare_parameter('btn_select', 8).value     # SELECT: DRIVE<->ARM
        self.btn_enable = self.declare_parameter('btn_enable', 5).value     # R1: 死人开关(两态通用)
        self.btn_level = self.declare_parameter('btn_level', 0).value       # △: 姿态回正+抬高
        self.btn_pick = self.declare_parameter('btn_pick', 2).value         # ✕: 抓取
        self.btn_ground = self.declare_parameter('btn_ground', 1).value     # ○: 放回地面
        self.btn_tray = self.declare_parameter('btn_tray', 3).value         # ■: 放托盘
        self.btn_unload_l = self.declare_parameter('btn_unload_l', 4).value  # L1: 卸左盘
        self.btn_unload_r = self.declare_parameter('btn_unload_r', 6).value  # L2: 卸右盘

        # ---- 轴号 (同上, 实测) ----
        self.axis_x = self.declare_parameter('axis_x', 1).value          # 左摇杆上下
        self.axis_y = self.declare_parameter('axis_y', 0).value          # 左摇杆左右
        self.axis_z = self.declare_parameter('axis_z', 3).value          # 右摇杆上下
        self.axis_yaw = self.declare_parameter('axis_yaw', 2).value      # 右摇杆左右
        self.axis_pitch = self.declare_parameter('axis_pitch', 5).value  # 十字键上下
        self.axis_roll = self.declare_parameter('axis_roll', 4).value    # 十字键左右
        self.deadzone = self.declare_parameter('deadzone', 0.15).value

        # ---- 点动速度 (speed_units: m/s 与 rad/s, servo.yaml command_in_type) ----
        # 保守起步. 约束盒是唯一防线, 快了撞上再钳位也来不及.
        # 右摇杆实测只到 0.88/0.81 (左摇杆满 1.00), 故 z/yaw 实际最大速比设定低约 15%.
        self.scale_lin = self.declare_parameter('servo_scale_linear', 0.03).value
        self.scale_rot = self.declare_parameter('servo_scale_angular', 0.20).value
        # roll/pitch 单独一档更小的: 腕一歪相机跟着歪, cam_target 标定立刻失效.
        self.scale_wrist = self.declare_parameter('servo_scale_wrist', 0.15).value

        # ---- 约束盒 (base_link 系, TCP=suction_tip 不许出这个盒) ----
        # 底: 与 grasp_node 的 pregrasp_height 同值 (抓取时臂本来就下到这个高度).
        # 顶/四周: 手动 jog 到极限实测 —— 待真机量, 现为占位.
        self.box_z_min = self.declare_parameter('box_z_min', 0.12).value
        self.box_z_max = self.declare_parameter('box_z_max', 0.40).value   # TODO 实测
        self.box_x_min = self.declare_parameter('box_x_min', -0.32).value  # TODO 实测: 四角
        self.box_x_max = self.declare_parameter('box_x_max', 0.32).value   # TODO 实测
        self.box_y_min = self.declare_parameter('box_y_min', -0.34).value  # TODO 实测
        self.box_y_max = self.declare_parameter('box_y_max', 0.10).value   # TODO 实测

        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.ee_frame = self.declare_parameter('ee_frame', 'suction_tip').value
        self.rate_hz = self.declare_parameter('publish_rate', 30.0).value
        # 上电是否自动摆 home: 真机第一次测时臂可能停在任意位置, 自动跑会突然大幅运动,
        # 故默认关. 桌面快捷方式那条命令里再打开 (那时臂位置已知).
        self.home_on_start = self.declare_parameter('home_on_start', False).value
        # 没起机械臂栈时只想遥控底盘: 置 true 则 /grasp/ready 不在线也放行 DRIVE。
        # ⚠️ 代价是"臂已收拢"这个前提没人验证过, 车可能拖着伸出的臂走 —— 只在臂确实
        # 没通电(或人眼确认已收拢)时用。臂栈在跑时保持 false, 让收臂失败拦住底盘。
        self.drive_without_arm = self.declare_parameter('drive_without_arm', False).value

        self.state = HOME
        self.last_joy = None
        self.prev_buttons = []
        self.lock = threading.Lock()
        self.busy = False           # 服务在跑, 期间不接点动也不切状态
        self.zero_sent = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel_out', 10)
        self.pub_servo = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)

        self.create_subscription(Joy, 'joy', self.on_joy, qos_profile_sensor_data,
                                 callback_group=cb)
        self.create_subscription(Twist, 'cmd_vel_in', self.on_cmd_vel, 10,
                                 callback_group=cb)

        self.cli = {n: self.create_client(Trigger, n, callback_group=cb) for n in (
            '/grasp/home', '/grasp/ready', '/grasp/look', '/grasp/level',
            '/grasp/pick_only', '/grasp/place_only', '/grasp/place_ground',
            '/grasp/unload_tray')}
        self.start_servo = self.create_client(Trigger, '/servo_node/start_servo',
                                              callback_group=cb)
        self.set_tray = self.create_client(SetParameters, '/grasp_node/set_parameters',
                                           callback_group=cb)

        self.create_timer(1.0 / self.rate_hz, self.tick, callback_group=cb)
        self.get_logger().warn(
            '手柄三态遥控就绪 [HOME] — START(%d) 进 DRIVE, SELECT(%d) 切 DRIVE/ARM, '
            '死人开关 R1(%d)' % (self.btn_start, self.btn_select, self.btn_enable))
        if self.home_on_start:
            self._run_action('上电收拢', '/grasp/home')

    # ---- 底盘: 只有 DRIVE 态转发 ----
    def on_cmd_vel(self, msg):
        if self.state == DRIVE and not self.busy:
            self.pub_cmd.publish(msg)

    def on_joy(self, msg):
        with self.lock:
            self.last_joy = msg
        pressed = self._rising_edges(msg)
        if not pressed or self.busy:
            return
        if self.btn_start in pressed and self.state == HOME:
            self._to_drive('START')
            return
        if self.btn_select in pressed:
            if self.state == DRIVE:
                self._to_arm()
            elif self.state == ARM:
                self._to_drive('SELECT')
            return
        if self.state != ARM:
            return
        if self.btn_level in pressed:
            self._run_action('姿态回正', '/grasp/level')
        elif self.btn_pick in pressed:
            # 抓完自动回正抬高: 吸着盒停在贴地姿态, 下一个点动指令容易蹭到地面.
            self._run_action('抓取', '/grasp/pick_only', then='/grasp/level')
        elif self.btn_ground in pressed:
            # 放地面前先回正: 任意姿态直插放置有概率歪着落下.
            self._run_action('放回地面', '/grasp/place_ground', first='/grasp/level')
        elif self.btn_tray in pressed:
            # 放托盘前先回 ready: 托盘在肩后死区, 从任意点动姿态规划过去失败率高.
            # place_only 内部放完自己回 ready, 这里再补一步回 look 待命.
            self._run_action('放托盘', '/grasp/place_only', first='/grasp/ready',
                             then='/grasp/look')
        elif self.btn_unload_l in pressed:
            self._run_unload('卸左盘', tray=1)
        elif self.btn_unload_r in pressed:
            self._run_unload('卸右盘', tray=0)

    def _rising_edges(self, msg):
        """只认按下那一瞬 (手柄 autorepeat 20Hz 续帧, 不去抖会连发几十次)."""
        edges = [i for i, v in enumerate(msg.buttons)
                 if v and (i >= len(self.prev_buttons) or not self.prev_buttons[i])]
        self.prev_buttons = list(msg.buttons)
        return edges

    # ---- 状态迁移 ----
    def _to_arm(self):
        # 先把车刹住再动臂, 顺序不能反.
        self._publish_zero()
        self.state = ARM
        self.get_logger().warn('[ARM] 底盘已停发, 臂去 look 位待命')
        self._run_action('去 look', '/grasp/look')
        if self.start_servo.service_is_ready():
            self.start_servo.call_async(Trigger.Request())

    def _to_drive(self, via):
        # 先把臂收回 ready 再放行底盘, 顺序不能反 —— 不然车拖着伸出的臂走.
        # 收臂期间挂在 HOME 这个过渡态: 车不放行, 臂也不接点动.
        prev, self.state = self.state, HOME
        self._publish_zero()
        self.get_logger().warn('[%s->DRIVE] 经 %s: 收臂到 ready 中, 底盘暂不放行...'
                               % (prev, via))

        def worker():
            self.busy = True
            try:
                ok, m = self._call_sync('/grasp/ready')
                if ok:
                    self.state = DRIVE
                    self.get_logger().warn('[DRIVE] 臂已收 ready — 底盘放行 (按住 R1 走)')
                elif self.drive_without_arm:
                    self.state = DRIVE
                    self.get_logger().warn(
                        '[DRIVE] 收臂失败(%s), 但 drive_without_arm=true 仍放行 '
                        '— 请自行确认臂已收拢' % m)
                else:
                    self.get_logger().error('收臂失败, 留在 HOME 不放行底盘: %s' % m)
            finally:
                self.busy = False
        threading.Thread(target=worker, daemon=True).start()

    def _publish_zero(self):
        self.pub_cmd.publish(Twist())

    # ---- 动作: 阻塞期间不接点动, 跑完回到点动待命 ----
    def _run_action(self, name, srv, first=None, then=None):
        def worker():
            self.busy = True
            try:
                if first:
                    self.get_logger().warn('%s: 先 %s' % (name, first))
                    ok, m = self._call_sync(first)
                    if not ok:
                        self.get_logger().error('%s 中止 — %s 失败: %s' % (name, first, m))
                        return
                ok, m = self._call_sync(srv)
                log = self.get_logger().warn if ok else self.get_logger().error
                log('%s -> %s %s' % (name, ok, m))
                if ok and then:
                    self.get_logger().warn('%s: 收尾 %s' % (name, then))
                    self._call_sync(then)
            finally:
                self.busy = False
        threading.Thread(target=worker, daemon=True).start()

    def _run_unload(self, name, tray):
        """卸货: 先把 grasp_node 的 unload_tray 设成目标盘号, 再调服务, 完事回 look."""
        def worker():
            self.busy = True
            try:
                if not self._set_tray_param(tray):
                    self.get_logger().error('%s 中止: 设 unload_tray=%d 失败' % (name, tray))
                    return
                ok, m = self._call_sync('/grasp/unload_tray')
                log = self.get_logger().warn if ok else self.get_logger().error
                log('%s (盘%d) -> %s %s' % (name, tray, ok, m))
                if ok:
                    self._call_sync('/grasp/look')
            finally:
                self.busy = False
        threading.Thread(target=worker, daemon=True).start()

    def _set_tray_param(self, tray):
        if not self.set_tray.service_is_ready():
            return False
        req = SetParameters.Request()
        req.parameters = [Parameter(
            name='unload_tray',
            value=ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                                 integer_value=tray))]
        fut = self.set_tray.call_async(req)
        if not self._wait(fut, 5.0):
            return False
        res = fut.result()
        return bool(res.results) and res.results[0].successful

    def _call_sync(self, srv, timeout=180.0):
        c = self.cli.get(srv)
        if not c or not c.service_is_ready():
            return False, '服务 %s 不在线' % srv
        fut = c.call_async(Trigger.Request())
        if not self._wait(fut, timeout):
            return False, '%s 超时' % srv
        return fut.result().success, fut.result().message

    def _wait(self, fut, timeout):
        end = self.get_clock().now() + Duration(seconds=timeout)
        while rclpy.ok() and not fut.done() and self.get_clock().now() < end:
            threading.Event().wait(0.05)
        return fut.done()

    # ---- 臂点动: 摇杆 -> TwistStamped, 约束盒钳位 ----
    def tick(self):
        if self.state != ARM or self.busy:
            return
        with self.lock:
            joy = self.last_joy
        if joy is None:
            return
        if not self._btn(joy, self.btn_enable):
            # 死人开关松开: servo 收不到指令会自己停, 但显式发零更干脆.
            if not self.zero_sent:
                self.pub_servo.publish(self._stamp(TwistStamped()))
                self.zero_sent = True
            return
        self.zero_sent = False

        t = TwistStamped()
        t.twist.linear.x = self._ax(joy, self.axis_x) * self.scale_lin
        t.twist.linear.y = self._ax(joy, self.axis_y) * self.scale_lin
        t.twist.linear.z = self._ax(joy, self.axis_z) * self.scale_lin
        t.twist.angular.z = self._ax(joy, self.axis_yaw) * self.scale_rot
        # 十字键是轴 (实测), 转腕. ⚠️ 约束盒管不了姿态, 这两个方向无几何防护.
        t.twist.angular.y = self._ax(joy, self.axis_pitch) * self.scale_wrist
        t.twist.angular.x = self._ax(joy, self.axis_roll) * self.scale_wrist
        self._clamp_to_box(t)
        self.pub_servo.publish(self._stamp(t))

    def _clamp_to_box(self, t):
        """出界的那个方向单独置零, 不整体停 —— 否则贴到边界就再也动不了."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame, rclpy.time.Time())
        except Exception as e:
            # 查不到 TCP 就不知道在哪, 约束盒失效 -> 一律不许动 (安全侧倒).
            t.twist = Twist()
            self.get_logger().warn('查 TF 失败, 点动已掐掉: %s' % e, throttle_duration_sec=2.0)
            return
        p = tf.transform.translation
        for val, lo, hi, attr in ((p.x, self.box_x_min, self.box_x_max, 'x'),
                                  (p.y, self.box_y_min, self.box_y_max, 'y'),
                                  (p.z, self.box_z_min, self.box_z_max, 'z')):
            v = getattr(t.twist.linear, attr)
            if (val <= lo and v < 0) or (val >= hi and v > 0):
                setattr(t.twist.linear, attr, 0.0)
                self.get_logger().warn('%s 轴到约束盒边界 (%.3f), 该方向已钳位' % (attr, val),
                                       throttle_duration_sec=1.0)

    def _stamp(self, t):
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        return t

    def _ax(self, joy, i):
        if i >= len(joy.axes):
            return 0.0
        v = joy.axes[i]
        return 0.0 if abs(v) < self.deadzone else v

    def _btn(self, joy, i):
        return i < len(joy.buttons) and bool(joy.buttons[i])


def main():
    rclpy.init()
    node = JoyArmTeleop()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
