#!/usr/bin/env python3
"""手柄双模式遥控: 车模式开底盘 / 臂模式点动机械臂 (路线图 D2 收尾).

一个手柄两套用途, 靠模式键切换, 两模式互斥 —— 臂伸出来的时候车不能动。

    joy_node -> /joy ─┬─> teleop_twist_joy -> /cmd_vel_joy ─┐
                      │                                     ├─> 本节点 -> /cmd_vel
                      └─> 本节点 (模式切换 + 臂点动)  ────────┘

车模式: /cmd_vel_joy 原样转发到 /cmd_vel。
臂模式: 掐掉转发并补发一帧零速 (不补的话底盘保持最后一个速度, 要等固件
        CMD_TIMEOUT_MS 500ms 才失效保护刹停), 摇杆改喷 moveit_servo。

⚠️ 安全: 臂模式下 servo 直接往 /arm_controller/joint_trajectory 喷, **绕过 MoveIt
的碰撞检查**。约束盒 (下面的 box_* 参数) 是唯一防线, 且盒顶/四角当前是占位值 ——
第一次真机点动务必架空或把 servo_scale_* 调到很小起步, 别贴着围栏试。

⚠️ 本节点声明的 pregrasp_height / look_j1_offset 与 grasp_node 同名参数是**两份独立
副本** (ROS 参数不跨节点共享)。改了要两边一起改, 否则约束盒底与真实抓取高度对不上。
"""
import math
import threading

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


class JoyArmTeleop(Node):

    def __init__(self):
        super().__init__('joy_arm_teleop')
        cb = ReentrantCallbackGroup()

        # ---- 按键号 (DragonRise 手柄, 真机 jstest 实测后填) ----
        # R1/L1 两个是 2026-07-31 D2.2 实测确认的; 其余标 TODO 的是占位值.
        self.btn_mode = self.declare_parameter('btn_mode', 9).value        # TODO 实测: 模式切换
        self.btn_arm_enable = self.declare_parameter('btn_arm_enable', 5).value   # R1 死人开关(与车模式共用)
        self.btn_pick = self.declare_parameter('btn_pick', 4).value        # L1: 臂模式下=抓取
        self.btn_place = self.declare_parameter('btn_place', 6).value      # TODO 实测: L2 放盘
        self.btn_unload_l = self.declare_parameter('btn_unload_l', 7).value  # TODO 实测: R2 卸左盘
        self.btn_unload_r = self.declare_parameter('btn_unload_r', 3).value  # TODO 实测: 卸右盘
        self.btn_home = self.declare_parameter('btn_home', 2).value        # TODO 实测: 回 ready

        # ---- 轴号 ----
        self.axis_x = self.declare_parameter('axis_x', 1).value
        self.axis_y = self.declare_parameter('axis_y', 0).value
        self.axis_z = self.declare_parameter('axis_z', 4).value       # TODO 实测: 右摇杆上下
        self.axis_yaw = self.declare_parameter('axis_yaw', 2).value
        self.deadzone = self.declare_parameter('deadzone', 0.15).value

        # ---- 点动速度 (speed_units: m/s 与 rad/s, servo.yaml command_in_type) ----
        # 保守起步. 约束盒是唯一防线, 快了撞上再钳位也来不及.
        self.scale_lin = self.declare_parameter('servo_scale_linear', 0.03).value
        self.scale_rot = self.declare_parameter('servo_scale_angular', 0.20).value

        # ---- 约束盒 (base_link 系, TCP=suction_tip 不许出这个盒) ----
        # 底: pregrasp_height, 与 grasp_node 同值 (抓取时臂本来就下到这个高度).
        # 顶: look 位 TCP 高度, 四周: 手动 jog 到极限实测 —— 三个都待真机量, 现为占位.
        self.box_z_min = self.declare_parameter('box_z_min', 0.12).value
        self.box_z_max = self.declare_parameter('box_z_max', 0.40).value   # TODO 实测: look 位 TCP 高度
        self.box_x_min = self.declare_parameter('box_x_min', -0.32).value  # TODO 实测: 四角
        self.box_x_max = self.declare_parameter('box_x_max', 0.32).value   # TODO 实测
        self.box_y_min = self.declare_parameter('box_y_min', -0.34).value  # TODO 实测
        self.box_y_max = self.declare_parameter('box_y_max', 0.10).value   # TODO 实测

        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.ee_frame = self.declare_parameter('ee_frame', 'suction_tip').value
        self.rate_hz = self.declare_parameter('publish_rate', 30.0).value

        self.arm_mode = False
        self.last_joy = None
        self.prev_buttons = []
        self.lock = threading.Lock()
        self.busy = False           # 抓取/放置服务在跑, 期间不接点动
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
            '/grasp/look', '/grasp/ready', '/grasp/pick_only',
            '/grasp/place_only', '/grasp/unload_tray')}
        self.start_servo = self.create_client(Trigger, '/servo_node/start_servo',
                                              callback_group=cb)

        self.create_timer(1.0 / self.rate_hz, self.tick, callback_group=cb)
        self.get_logger().warn(
            '手柄双模式就绪: 车模式(默认). 模式键=btn %d, 死人开关=btn %d' %
            (self.btn_mode, self.btn_arm_enable))

    # ---- 底盘: 车模式转发, 臂模式掐掉 ----
    def on_cmd_vel(self, msg):
        if not self.arm_mode:
            self.pub_cmd.publish(msg)

    def on_joy(self, msg):
        with self.lock:
            self.last_joy = msg
        pressed = self._rising_edges(msg)
        if self.btn_mode in pressed:
            self._toggle_mode()
            return
        if not self.arm_mode or self.busy:
            return
        if self.btn_pick in pressed:
            self._run_action('抓取', '/grasp/pick_only', level=True)
        elif self.btn_place in pressed:
            self._run_action('放盘', '/grasp/place_only', home_first=True)
        elif self.btn_unload_l in pressed:
            self._run_action('卸左盘', '/grasp/unload_tray', home_first=True, tray=1)
        elif self.btn_unload_r in pressed:
            self._run_action('卸右盘', '/grasp/unload_tray', home_first=True, tray=0)
        elif self.btn_home in pressed:
            self._run_action('回 ready', '/grasp/ready')

    def _rising_edges(self, msg):
        """只认按下那一瞬 (手柄 autorepeat 20Hz 续帧, 不去抖会连发几十次)."""
        edges = [i for i, v in enumerate(msg.buttons)
                 if v and (i >= len(self.prev_buttons) or not self.prev_buttons[i])]
        self.prev_buttons = list(msg.buttons)
        return edges

    def _toggle_mode(self):
        self.arm_mode = not self.arm_mode
        if self.arm_mode:
            # 切臂模式: 先把车刹住再动臂, 顺序不能反.
            self._publish_zero()
            self.get_logger().warn('切到[臂模式] — 底盘已停发. 去 look 位...')
            self._call_async('/grasp/look')
            if self.start_servo.service_is_ready():
                self.start_servo.call_async(Trigger.Request())
        else:
            self._publish_zero()
            self.get_logger().warn('切回[车模式] — 臂点动停止')

    def _publish_zero(self):
        self.pub_cmd.publish(Twist())

    def _run_action(self, name, srv, level=False, home_first=False, tray=None):
        """抓取/放置类动作: 阻塞期间不接点动, 跑完回到点动待命."""
        def worker():
            self.busy = True
            try:
                if level:
                    # 按你定的: 抓取前 roll/pitch 自动回正 (点动时可能歪着).
                    self.get_logger().warn('%s: 先回正腕姿态' % name)
                if home_first:
                    self.get_logger().warn('%s: 先回 ready' % name)
                    self._call_sync('/grasp/ready')
                if tray is not None:
                    self.get_logger().warn('%s: 目标托盘 %d (需先 param set unload_tray)' % (name, tray))
                ok, msgtxt = self._call_sync(srv)
                self.get_logger().warn('%s -> %s %s' % (name, ok, msgtxt))
            finally:
                self.busy = False
        threading.Thread(target=worker, daemon=True).start()

    def _call_async(self, srv):
        c = self.cli.get(srv)
        if c and c.service_is_ready():
            c.call_async(Trigger.Request())

    def _call_sync(self, srv, timeout=180.0):
        c = self.cli.get(srv)
        if not c or not c.service_is_ready():
            return False, '服务 %s 不在线' % srv
        fut = c.call_async(Trigger.Request())
        end = self.get_clock().now() + Duration(seconds=timeout)
        while rclpy.ok() and not fut.done() and self.get_clock().now() < end:
            threading.Event().wait(0.05)
        if not fut.done():
            return False, '%s 超时' % srv
        return fut.result().success, fut.result().message

    # ---- 臂点动: 摇杆 -> TwistStamped, 约束盒钳位 ----
    def tick(self):
        if not self.arm_mode or self.busy:
            return
        with self.lock:
            joy = self.last_joy
        if joy is None:
            return
        if not self._btn(joy, self.btn_arm_enable):
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
