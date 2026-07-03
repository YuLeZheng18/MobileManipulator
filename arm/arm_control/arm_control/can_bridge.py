#!/usr/bin/env python3
"""
CAN 桥接节点 (实车 ros2_control 后端).

数据流:
  MoveIt -> JTC(100Hz 五次样条插补) -> topic_based_ros2_control/TopicBasedSystem
    -> 发布 /arm_joint_commands (sensor_msgs/JointState, Joint_11~16, 弧度)
        -> [本节点] 订阅 -> 名字映射 Joint_11~16->电机1~6 -> 弧度转度 -> 0xFD 双帧发 CAN
        <- CAN 反馈(0x36 查询) -> 度转弧度 -> 发布 /arm_joint_states (Joint_11~16, 弧度)
    <- TopicBasedSystem 订阅, 填 state_interface -> JTC 闭环

协议/减速比/方向位完全复用 joint_gui.py 中已在实车验证过的实现, 仅去掉 GUI 与电机间 sleep,
改为「命令回调只更新缓存目标 + 定时器定频连续发送」, 以跟上 JTC 的稠密位置流.
"""
import os
import json
import math
import time
from threading import Lock, Thread
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState

from .can_interface import CANInterface
from .PCANBasic import PCAN_USBBUS1, PCAN_BAUD_500K


CONFIG_PATH = os.path.expanduser('~/.robot_arm_config.json')

# MoveIt/JTC 关节名 (Joint_11~16) 按顺序映射到电机 1~6
MOVEIT_JOINT_NAMES = ['Joint_11', 'Joint_12', 'Joint_13', 'Joint_14', 'Joint_15', 'Joint_16']
MOTOR_COUNT = 6


class MotorConfig:
    """与 joint_gui.py 共用 ~/.robot_arm_config.json, 保证仿真调好的参数与实车一致."""
    def __init__(self):
        self.REDUCTION_RATIOS = [50.0, 50.0, 30.0, 82.67, 62.5, 27.0]
        self.DIRECTION_MAP = [False, False, False, False, False, False]
        self.SPEEDS = [250, 250, 150, 250, 250, 135]
        self.ACCELERATIONS = [500, 500, 500, 500, 500, 500]
        self.load()

    def load(self):
        if not os.path.exists(CONFIG_PATH):
            return
        try:
            with open(CONFIG_PATH, 'r') as f:
                data = json.load(f)
            self.REDUCTION_RATIOS = data.get('reduction_ratios', self.REDUCTION_RATIOS)
            self.DIRECTION_MAP = data.get('direction_map', self.DIRECTION_MAP)
            self.SPEEDS = data.get('speeds', self.SPEEDS)
            self.ACCELERATIONS = data.get('accelerations', self.ACCELERATIONS)
        except Exception:
            pass


class CanBridge(Node):
    def __init__(self):
        super().__init__('arm_can_bridge')

        self.declare_parameter('command_topic', '/arm_joint_commands')
        self.declare_parameter('state_topic', '/arm_joint_states')
        self.declare_parameter('send_rate_hz', 100.0)
        self.declare_parameter('query_rate_hz', 100.0)
        self.declare_parameter('auto_enable', True)
        # per-motor speed 覆盖(电机内部0xFD的speed字段). 空=用~/.robot_arm_config.json的SPEEDS.
        # 用途: J3减速比最低、每帧位移最小, 默认speed=150相对太快导致"瞬冲+空等"走停, 在此调低.
        self.declare_parameter('motor_speeds', [0, 0, 0, 0, 0, 0])
        # 发送死区(度): 目标相对上次实发变化<此值的电机不重发.
        # 目的: 轨迹到位后目标静止时停发, 不再对已到位电机每帧重启梯形规划器
        # (会触发堵转保护锁死), 同时让查询帧恢复->反馈不再饿死. 对齐 joint_gui「动时发/停时静」.
        self.declare_parameter('send_deadband_deg', 0.05)

        command_topic = self.get_parameter('command_topic').value
        state_topic = self.get_parameter('state_topic').value
        speeds_override = list(self.get_parameter('motor_speeds').value)
        self._send_deadband_deg = float(self.get_parameter('send_deadband_deg').value)
        # PCAN 通道是 ctypes TPCANHandle, 不适合做 ROS 参数, 直接用常量(与 joint_gui.py 一致)
        self.can_channel = PCAN_USBBUS1
        self.send_rate = float(self.get_parameter('send_rate_hz').value)
        self.query_rate = float(self.get_parameter('query_rate_hz').value)
        self.auto_enable = bool(self.get_parameter('auto_enable').value)

        self.config = MotorConfig()
        # 应用 per-motor speed 覆盖(>0 才覆盖, 0 保留 json 配置)
        for i in range(MOTOR_COUNT):
            if i < len(speeds_override) and speeds_override[i] > 0:
                self.config.SPEEDS[i] = speeds_override[i]
        self.get_logger().info(f'电机SPEEDS={self.config.SPEEDS}')
        self.can = CANInterface()

        self._lock = Lock()
        # CAN 总线发送锁: send_loop/query_loop/enable 多线程并发写同一 PCAN 通道,
        # send_message 内部填充 TPCANMsg.DATA 非原子, 并发会拼出畸形帧 -> 电机回 00 EE(错误命令).
        # 所有 send_message 必须经此锁串行化.
        self._can_tx_lock = Lock()
        # 当前目标(度), None 表示尚未收到命令, 不主动驱动电机
        self._target_deg: Optional[list] = None
        # 上次实际发出的目标(度), None 表示还没发过; 用于死区判重, 静止目标不重发
        self._last_sent: Optional[list] = None
        # 电机反馈位置(度)
        self._motor_deg = [0.0] * MOTOR_COUNT

        self._running = True
        self._receive_running = False
        self._query_running = False
        self._query_paused = False

        qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE)
        qos_rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)

        # 订阅 JTC 稠密指令流; 回调只更新缓存目标, 不在回调里发 CAN(避免阻塞 executor)
        self._cmd_sub = self.create_subscription(
            JointState, command_topic, self._on_command, qos_be)
        # 发布关节状态给 TopicBasedSystem
        self._state_pub = self.create_publisher(JointState, state_topic, qos_rel)

        self._connect_can()

    # ---------- CAN 生命周期 ----------
    def _connect_can(self):
        ok, msg = self.can.initialize(self.can_channel, PCAN_BAUD_500K, False)
        if not ok:
            self.get_logger().error(f'CAN 初始化失败: {msg}')
            return
        self.get_logger().info('CAN 已连接')

        if self.auto_enable:
            self._enable_motors(True)

        self._start_receiving()
        self._start_query_loop()
        self._start_send_loop()
        self.get_logger().info('CAN 桥接就绪 (命令缓存+定频发送)')

    def _enable_motors(self, enable: bool):
        if not self.can.is_open:
            return
        state = 0x01 if enable else 0x00
        data = [0xF3, 0xAB, state, 0x00, 0x6B]
        for mid in range(1, MOTOR_COUNT + 1):
            can_id = 0x100 + (mid - 1) * 0x100
            self._can_send(can_id, data, True)
            time.sleep(0.002)
        self.get_logger().info(f'电机{"使能" if enable else "失能"}')

    # ---------- 指令: 订阅回调 -> 缓存目标 ----------
    def _on_command(self, msg: JointState):
        target = self._target_or_none()
        # 以名字匹配, 容忍 JTC 给的关节顺序与 MOVEIT_JOINT_NAMES 不同
        new_target = list(target) if target else [None] * MOTOR_COUNT
        for i, name in enumerate(msg.name):
            if name in MOVEIT_JOINT_NAMES and i < len(msg.position):
                idx = MOVEIT_JOINT_NAMES.index(name)
                new_target[idx] = math.degrees(msg.position[i])
        # 若首帧某些关节缺失, 用当前反馈填充, 避免发 None
        if any(v is None for v in new_target):
            fb = self.get_motor_deg()
            new_target = [new_target[i] if new_target[i] is not None else fb[i]
                          for i in range(MOTOR_COUNT)]
        with self._lock:
            self._target_deg = new_target

    def _target_or_none(self):
        with self._lock:
            return list(self._target_deg) if self._target_deg is not None else None

    # ---------- 发送循环: 定频把最新目标发 CAN ----------
    def _start_send_loop(self):
        self._send_thread = Thread(target=self._send_loop, daemon=True)
        self._send_thread.start()

    def _send_loop(self):
        period = 1.0 / self.send_rate if self.send_rate > 0 else 0.01
        while self._running:
            target = self._target_or_none()
            # 仅在目标相对上次实发有变化时才发一轮; 目标静止(轨迹已到位)则整轮跳过,
            # 既不重启已到位电机的梯形规划器(防堵转锁死), 也不占用总线(查询帧得以恢复->反馈不饿死).
            if target is not None and self.can.is_open and self._target_changed(target):
                # 帧构造与总线纪律完全不变: 发位置期间 _query_paused 挡住查询帧插入双帧中间(防 00 EE),
                # 电机间隔 1ms 防 PCAN 队列瞬时溢出.
                self._query_paused = True
                try:
                    for i in range(MOTOR_COUNT):
                        self._send_position_command(i + 1, target[i])
                        time.sleep(0.001)
                    self._last_sent = list(target)
                finally:
                    self._query_paused = False
            time.sleep(period)

    def _target_changed(self, target) -> bool:
        """任一电机目标相对上次实发变化超过死区即需要重发; 首次(未发过)必发."""
        if self._last_sent is None:
            return True
        return any(abs(target[i] - self._last_sent[i]) > self._send_deadband_deg
                   for i in range(MOTOR_COUNT))

    # ---------- 0xFD 梯形位置模式 (joint_gui 验证过的基线协议) ----------
    def _send_position_command(self, motor_id: int, position_deg: float) -> bool:
        if not self.can.is_open or not (1 <= motor_id <= MOTOR_COUNT):
            return False
        can_id = 0x100 + (motor_id - 1) * 0x100
        idx = motor_id - 1

        motor_direction = self.config.DIRECTION_MAP[idx]
        motor_speed = self.config.SPEEDS[idx] * 10
        motor_accel = self.config.ACCELERATIONS[idx]

        if motor_direction:
            direction = 0x00 if position_deg >= 0 else 0x01
        else:
            direction = 0x01 if position_deg >= 0 else 0x00

        ratio = self.config.REDUCTION_RATIOS[idx]
        if ratio > 0:
            pos_with_red = int(abs(position_deg) * 10 * ratio)
        else:
            pos_with_red = int(abs(position_deg) * 10)
        return self._send_position_frame(can_id, direction, motor_speed, motor_accel, pos_with_red)

    def _can_send(self, can_id, data, is_extended=True):
        """所有 CAN 发送的唯一出口, 经 _can_tx_lock 串行化, 杜绝并发拼帧损坏."""
        with self._can_tx_lock:
            return self.can.send_message(can_id, data, is_extended)

    def _send_position_frame(self, can_id, direction, speed, accel, position) -> bool:
        try:
            pos_bytes = position.to_bytes(4, byteorder='big')
            speed_bytes = speed.to_bytes(2, byteorder='big')
            accel_bytes = int(accel).to_bytes(2, byteorder='big')
            # 帧尾(官方ZDT_X57_V2): 相对绝对标志(0x01绝对) + 多机同步标志(0x00立即执行) + 0x6B
            abs_flag = 0x01
            sync_flag = 0x00
            data_bytes = ([0xFD, direction] + list(accel_bytes) + list(accel_bytes)
                          + list(speed_bytes) + list(pos_bytes) + [abs_flag, sync_flag, 0x6B])
            first = data_bytes[:8]
            second = data_bytes[8:]
            # 双帧必须在同一锁内连发, 防止中间被查询帧插入打断
            with self._can_tx_lock:
                ok1, _ = self.can.send_message(can_id, first, True)
                if not ok1:
                    return False
                if second:
                    ok2, _ = self.can.send_message(can_id + 1, [0xFD] + second, True)
                    if not ok2:
                        return False
            return True
        except Exception:
            return False

    # ---------- 查询循环: 0x36 请求反馈 ----------
    def _start_query_loop(self):
        self._query_running = True
        self._query_thread = Thread(target=self._query_loop, daemon=True)
        self._query_thread.start()

    def _query_loop(self):
        period = 1.0 / self.query_rate if self.query_rate > 0 else 0.01
        data = [0x36, 0x6b]
        while self._query_running:
            if self.can.is_open and not self._query_paused:
                for mid in range(1, MOTOR_COUNT + 1):
                    self._can_send(0x100 + (mid - 1) * 0x100, data, True)
                    time.sleep(0.001)
            time.sleep(period)

    # ---------- 接收循环: 解析反馈 -> 发布 joint_states ----------
    def _start_receiving(self):
        if self._receive_running:
            return
        self._receive_running = True
        self._receive_thread = Thread(target=self._receive_loop, daemon=True)
        self._receive_thread.start()

    def _receive_loop(self):
        while self._receive_running:
            try:
                ok, msg = self.can.receive_message()
                if ok:
                    self._process_message(msg)
                else:
                    time.sleep(0.002)
            except Exception:
                time.sleep(0.02)

    def _process_message(self, msg):
        try:
            if not isinstance(msg, dict):
                return
            can_id = msg.get('id')
            data = msg.get('data')
            if isinstance(data, (bytes, bytearray)):
                data = list(data)
            if not (0x100 <= can_id <= 0x600 and len(data) >= 7
                    and data[0] == 0x36 and data[-1] == 0x6b):
                return
            idx = (can_id >> 8) - 1
            if not (0 <= idx < MOTOR_COUNT):
                return

            direction = data[1]
            raw = int.from_bytes(data[2:6], byteorder='big', signed=False)
            angle = raw / 10.0
            if direction == 0x01:
                angle = -angle
            ratio = self.config.REDUCTION_RATIOS[idx]
            if ratio > 0 and ratio != 1:
                angle = angle / ratio
            if not self.config.DIRECTION_MAP[idx]:
                angle = -angle

            with self._lock:
                self._motor_deg[idx] = angle
            self._publish_state()
        except Exception:
            pass

    def get_motor_deg(self):
        with self._lock:
            return list(self._motor_deg)

    def _publish_state(self):
        deg = self.get_motor_deg()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(MOVEIT_JOINT_NAMES)
        msg.position = [math.radians(d) for d in deg]
        msg.velocity = [0.0] * MOTOR_COUNT
        msg.effort = [0.0] * MOTOR_COUNT
        self._state_pub.publish(msg)

    def destroy_node(self):
        self._running = False
        self._receive_running = False
        self._query_running = False
        time.sleep(0.1)
        if self.can.is_open:
            self.can.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CanBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
