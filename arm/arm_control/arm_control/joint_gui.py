import sys
import time
import signal
import copy
import math
import json
import os
import collections
import rclpy
import threading
from typing import Optional
from threading import Thread, Lock

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QTableWidget, QTableWidgetItem, QHeaderView, QLabel, QGroupBox,
    QPushButton, QCheckBox, QSplitter, QMessageBox, QDoubleSpinBox,
    QFormLayout, QScrollArea, QInputDialog, QLineEdit, QComboBox
)
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState

from .can_interface import CANInterface
from .PCANBasic import PCAN_USBBUS1, PCAN_BAUD_500K


CONFIG_PATH = os.path.expanduser('~/.robot_arm_config.json')


class MotorConfig:
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
            if 'reduction_ratios' in data:
                self.REDUCTION_RATIOS = data['reduction_ratios']
            if 'direction_map' in data:
                self.DIRECTION_MAP = data['direction_map']
            if 'speeds' in data:
                self.SPEEDS = data['speeds']
            if 'accelerations' in data:
                self.ACCELERATIONS = data['accelerations']
        except Exception:
            pass

    def save(self):
        data = {
            'reduction_ratios': self.REDUCTION_RATIOS,
            'direction_map': self.DIRECTION_MAP,
            'speeds': self.SPEEDS,
            'accelerations': self.ACCELERATIONS
        }
        try:
            with open(CONFIG_PATH, 'w') as f:
                json.dump(data, f, indent=2)
        except Exception:
            pass


class ArmNode(Node):
    JOINT_NAMES = ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5', 'Joint_6']
    MOTOR_COUNT = 6

    def __init__(self):
        super().__init__('arm_control_node')

        self.config = MotorConfig()
        self.can = CANInterface()
        self.motor_positions = [0.0] * self.MOTOR_COUNT
        self.motor_raw_positions = [0.0] * self.MOTOR_COUNT
        self._lock = Lock()
        self._receive_running = False
        self._receive_thread: Optional[Thread] = None
        self._query_paused = False
        self._query_running = False

        self.joint_data: Optional[dict] = None
        self._spin_thread: Optional[Thread] = None
        self._running = True
        self._homing = False

        qos_best_effort = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        qos_reliable = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            qos_best_effort
        )

        self.state_pub = self.create_publisher(
            JointState,
            '/motor_states',
            qos_reliable
        )

    def _joint_state_callback(self, msg: JointState):
        with self._lock:
            self.joint_data = {
                'names': list(msg.name),
                'positions': list(msg.position),
                'velocities': list(msg.velocity),
                'efforts': list(msg.effort)
            }

    def get_joint_data(self):
        with self._lock:
            return copy.deepcopy(self.joint_data) if self.joint_data else None

    def start_spinning(self):
        self._spin_thread = Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    def _spin(self):
        while self._running:
            rclpy.spin_once(self, timeout_sec=0.05)

    def stop_spinning(self):
        self._running = False

    def connect_can(self, channel=0x51, baudrate=PCAN_BAUD_500K, fd_mode=False):
        success, msg = self.can.initialize(channel, baudrate, fd_mode)
        if success:
            self._start_receiving()
            self._start_query_timer()
        return success, msg

    def disconnect_can(self):
        self._stop_query_timer()
        self._stop_receiving()
        if self.can.is_open:
            return self.can.close()
        return True, "already closed"

    def enable_motors(self, enable: bool):
        if not self.can.is_open:
            return False, "CAN not connected"
        self._query_paused = True
        enable_state = 0x01 if enable else 0x00
        data = [0xF3, 0xAB, enable_state, 0x00, 0x6B]
        success_all = True
        for mid in range(1, self.MOTOR_COUNT + 1):
            can_id = 0x100 + (mid - 1) * 0x100
            success, msg = self.can.send_message(can_id, data, True)
            if not success:
                success_all = False
            time.sleep(0.002)
        self._query_paused = False
        return success_all, ""

    def home_motors(self):
        if not self.can.is_open:
            return False, "CAN not connected"
        if self._homing:
            return True, ""

        current_positions = self.get_motor_positions()
        distances = [abs(p) for p in current_positions]
        max_dist = max(distances) if distances else 0

        if max_dist < 0.01:
            for i in range(self.MOTOR_COUNT):
                self._send_position_command(i + 1, 0.0)
            return True, ""

        plan = self._s_curve_plan(max_dist, v_max=90.0, a_max=180.0, j_max=900.0)
        if plan is None:
            for i in range(self.MOTOR_COUNT):
                self._send_position_command(i + 1, 0.0)
            return True, ""

        self._homing = True
        t = Thread(target=self._home_trajectory,
                   args=(current_positions, plan, 900.0, max_dist),
                   daemon=True)
        t.start()
        return True, ""

    def _home_trajectory(self, start_positions, plan, j_max, max_dist):
        dt = 0.02
        t = 0.0
        T_total = plan['T_total']

        while t <= T_total and self._homing:
            p_ref = self._s_curve_pos_at(t, plan, j_max)
            ratio = p_ref / max_dist if max_dist > 0 else 0

            for i in range(self.MOTOR_COUNT):
                target = start_positions[i] * (1.0 - ratio)
                self._send_position_command(i + 1, target)
                time.sleep(0.002)

            t += dt
            elapsed = time.time()
            time.sleep(max(0, dt - 0.005))

        if self._homing:
            for i in range(self.MOTOR_COUNT):
                self._send_position_command(i + 1, 0.0)

        self._homing = False

    def stop_homing(self):
        self._homing = False

    @staticmethod
    def _s_curve_plan(q, v_max, a_max, j_max):
        if q < 1e-6:
            return None

        T_j = a_max / j_max
        v_j = 0.5 * j_max * T_j ** 2
        v_peak_no_t2 = 2.0 * v_j

        if v_peak_no_t2 >= v_max:
            T1 = math.sqrt(v_max / j_max)
            T2 = 0.0
            a_peak = j_max * T1
            v_peak = j_max * T1 ** 2
            v_j_actual = 0.5 * j_max * T1 ** 2
        else:
            T1 = T_j
            T2 = (v_max - v_peak_no_t2) / a_max
            a_peak = a_max
            v_peak = v_max
            v_j_actual = v_j

        d1 = j_max * T1 ** 3 / 6.0
        d2 = v_j_actual * T2 + 0.5 * a_peak * T2 ** 2
        v2 = v_j_actual + a_peak * T2
        d3 = v2 * T1 + 0.5 * a_peak * T1 ** 2 - j_max * T1 ** 3 / 6.0
        q_acc = d1 + d2 + d3

        if 2.0 * q_acc > q:
            v_lo, v_hi = 0.0, v_peak
            for _ in range(64):
                v_mid = (v_lo + v_hi) / 2.0
                p = ArmNode._s_curve_plan_inner(v_mid, a_max, j_max)
                if p is None:
                    v_hi = v_mid
                    continue
                if 2.0 * p['q_acc'] > q:
                    v_hi = v_mid
                else:
                    v_lo = v_mid

            p = ArmNode._s_curve_plan_inner(v_lo, a_max, j_max)
            T1 = p['T1']
            T2 = p['T2']
            a_peak = p['a_peak']
            v_peak = v_lo
            v_j_actual = 0.5 * j_max * T1 ** 2
            q_acc = p['q_acc']

        T4 = (q - 2.0 * q_acc) / v_peak if v_peak > 1e-9 else 0.0
        T_total = 2.0 * (T1 + T2 + T1) + T4

        return {
            'T1': T1, 'T2': T2, 'T4': T4,
            'T_total': T_total,
            'v_peak': v_peak, 'a_peak': a_peak,
            'v_j': v_j_actual, 'q_acc': q_acc
        }

    @staticmethod
    def _s_curve_plan_inner(v_peak, a_max, j_max):
        T_j = a_max / j_max
        v_j_full = 0.5 * j_max * T_j ** 2
        v_peak_no_t2 = 2.0 * v_j_full

        if v_peak_no_t2 >= v_peak:
            T1 = math.sqrt(v_peak / j_max) if v_peak > 0 else 0
            T2 = 0.0
            a_peak = j_max * T1
            v_j = 0.5 * j_max * T1 ** 2
        else:
            T1 = T_j
            T2 = (v_peak - v_peak_no_t2) / a_max
            a_peak = a_max
            v_j = v_j_full

        d1 = j_max * T1 ** 3 / 6.0
        d2 = v_j * T2 + 0.5 * a_peak * T2 ** 2
        v2 = v_j + a_peak * T2
        d3 = v2 * T1 + 0.5 * a_peak * T1 ** 2 - j_max * T1 ** 3 / 6.0
        q_acc = d1 + d2 + d3

        return {'T1': T1, 'T2': T2, 'a_peak': a_peak, 'q_acc': q_acc}

    @staticmethod
    def _s_curve_pos_at(t, plan, j_max):
        T1 = plan['T1']
        T2 = plan['T2']
        T4 = plan['T4']
        a_peak = plan['a_peak']
        v_peak = plan['v_peak']
        v_j = plan['v_j']

        T3 = T1
        T5 = T1
        T6 = T2
        T7 = T1

        tb1 = T1
        tb2 = tb1 + T2
        tb3 = tb2 + T3
        tb4 = tb3 + T4
        tb5 = tb4 + T5
        tb6 = tb5 + T6
        T_total = tb6 + T7

        if t <= 0:
            return 0.0
        if t >= T_total:
            q_acc = plan['q_acc']
            return 2.0 * q_acc + v_peak * T4

        p1 = j_max * T1 ** 3 / 6.0
        v2 = v_j + a_peak * T2
        p2 = p1 + v_j * T2 + 0.5 * a_peak * T2 ** 2
        p3 = p2 + v2 * T1 + 0.5 * a_peak * T1 ** 2 - j_max * T1 ** 3 / 6.0
        p4 = p3 + v_peak * T4
        v5 = v_peak - v_j
        p5 = p4 + v_peak * T1 - j_max * T1 ** 3 / 6.0
        v6 = v_peak - v2
        p6 = p5 + v5 * T2 - 0.5 * a_peak * T2 ** 2

        if t <= tb1:
            tau = t
            return j_max * tau ** 3 / 6.0
        elif t <= tb2:
            tau = t - tb1
            return p1 + v_j * tau + 0.5 * a_peak * tau ** 2
        elif t <= tb3:
            tau = t - tb2
            return p2 + v2 * tau + 0.5 * a_peak * tau ** 2 - j_max * tau ** 3 / 6.0
        elif t <= tb4:
            tau = t - tb3
            return p3 + v_peak * tau
        elif t <= tb5:
            tau = t - tb4
            return p4 + v_peak * tau - j_max * tau ** 3 / 6.0
        elif t <= tb6:
            tau = t - tb5
            return p5 + v5 * tau - 0.5 * a_peak * tau ** 2
        else:
            tau = t - tb6
            return p6 + v6 * tau - 0.5 * a_peak * tau ** 2 + j_max * tau ** 3 / 6.0

    def sync_positions_to_motors(self, joint_data: dict):
        if not self.can.is_open:
            return False, "CAN not connected"
        names = joint_data.get('names', [])
        positions = joint_data.get('positions', [])
        for i, name in enumerate(names):
            if name in self.JOINT_NAMES and i < len(positions):
                motor_id = self.JOINT_NAMES.index(name) + 1
                position_deg = math.degrees(positions[i])
                self._send_position_command(motor_id, position_deg)
                time.sleep(0.005)
        return True, ""

    def _send_position_command(self, motor_id: int, position: float):
        if not self.can.is_open:
            return False
        if not 1 <= motor_id <= self.MOTOR_COUNT:
            return False

        can_id = 0x100 + (motor_id - 1) * 0x100
        position_value = int(position * 10)

        motor_index = motor_id - 1
        motor_direction = self.config.DIRECTION_MAP[motor_index]
        motor_speed = self.config.SPEEDS[motor_index] * 10
        motor_acceleration = self.config.ACCELERATIONS[motor_index]

        if motor_direction:
            direction = 0x00 if position >= 0 else 0x01
        else:
            direction = 0x01 if position >= 0 else 0x00

        reduction_ratio = self.config.REDUCTION_RATIOS[motor_index]
        if reduction_ratio > 0:
            position_with_reduction = int(abs(position_value) * reduction_ratio)
        else:
            position_with_reduction = abs(position_value)

        return self._send_position_frame(can_id, direction, motor_speed, motor_acceleration, position_with_reduction)

    def _send_position_frame(self, can_id: int, direction: int, speed: int, acceleration: int, position: int) -> bool:
        try:
            pos_bytes = position.to_bytes(4, byteorder='big')
            speed_bytes = speed.to_bytes(2, byteorder='big')
            accel_bytes = int(acceleration).to_bytes(2, byteorder='big')
            decel_bytes = int(acceleration).to_bytes(2, byteorder='big')
            data_bytes = [0xFD, direction] + list(accel_bytes) + list(decel_bytes) + list(speed_bytes) + list(pos_bytes) + [0x01, 0x00, 0x6B]

            first_package = data_bytes[:8]
            second_package = data_bytes[8:]

            success1, _ = self.can.send_message(can_id, first_package, True)
            if not success1:
                return False

            if len(second_package) > 0:
                second_package_with_fd = [0xFD] + second_package
                success2, _ = self.can.send_message(can_id + 1, second_package_with_fd, True)
                if not success2:
                    return False

            return True
        except Exception:
            return False

    def _send_position_query(self):
        if not self.can.is_open:
            return
        data = [0x36, 0x6b]
        for mid in range(1, self.MOTOR_COUNT + 1):
            can_id = 0x100 + (mid - 1) * 0x100
            self.can.send_message(can_id, data, True)
            time.sleep(0.002)

    def _start_query_timer(self):
        self._query_running = True
        self._query_thread = Thread(target=self._query_loop, daemon=True)
        self._query_thread.start()

    def _stop_query_timer(self):
        self._query_running = False

    def _query_loop(self):
        while self._query_running:
            if not self._query_paused and self.can.is_open:
                self._send_position_query()
            time.sleep(0.01)

    def _start_receiving(self):
        if self._receive_running:
            return
        self._receive_running = True
        self._receive_thread = Thread(target=self._receive_loop, daemon=True)
        self._receive_thread.start()

    def _stop_receiving(self):
        self._receive_running = False
        if self._receive_thread and self._receive_thread.is_alive():
            self._receive_thread.join(timeout=2.0)
        self._receive_thread = None

    def _receive_loop(self):
        while self._receive_running:
            try:
                success, msg = self.can.receive_message()
                if success:
                    self._process_message(msg)
                else:
                    time.sleep(0.005)
            except Exception:
                time.sleep(0.05)

    def _process_message(self, msg):
        try:
            if isinstance(msg, dict):
                can_id = msg.get('id')
                data = msg.get('data')
            else:
                return

            if isinstance(data, (bytes, bytearray)):
                data = list(data)

            if not (0x100 <= can_id <= 0x600 and len(data) >= 7
                    and data[0] == 0x36 and data[-1] == 0x6b):
                return

            motor_index = (can_id >> 8) - 1
            if not (0 <= motor_index < self.MOTOR_COUNT):
                return

            direction = data[1]
            raw_position = int.from_bytes(data[2:6], byteorder='big', signed=False)
            angle = raw_position / 10.0

            if direction == 0x01:
                angle = -angle

            reduction_ratio = self.config.REDUCTION_RATIOS[motor_index]
            motor_direction = self.config.DIRECTION_MAP[motor_index]

            if reduction_ratio > 0 and reduction_ratio != 1:
                angle = angle / reduction_ratio

            raw_angle = angle

            if not motor_direction:
                angle = -angle

            with self._lock:
                self.motor_positions[motor_index] = angle
                self.motor_raw_positions[motor_index] = raw_angle

            self._publish_motor_states()

        except Exception:
            pass

    def _publish_motor_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.JOINT_NAMES)

        with self._lock:
            positions_deg = list(self.motor_positions)

        msg.position = [math.radians(p) for p in positions_deg]
        msg.velocity = [0.0] * self.MOTOR_COUNT
        msg.effort = [0.0] * self.MOTOR_COUNT

        self.state_pub.publish(msg)

    def get_motor_positions(self):
        with self._lock:
            return list(self.motor_positions)

    def get_motor_raw_positions(self):
        with self._lock:
            return list(self.motor_raw_positions)

    def destroy_node(self):
        self._running = False
        self._query_running = False
        self._stop_receiving()
        if self.can.is_open:
            self.can.close()
        super().destroy_node()


class JointStateTable(QTableWidget):
    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setColumnCount(7)
        self.setHorizontalHeaderLabels([
            '关节名称', '位置 (rad/°)', '电机角度 (°/rad)', '电机原始 (°)',
            '速度 (rad/s)', '力矩 (Nm)', '偏差 (°)'
        ])
        self.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.setEditTriggers(QTableWidget.NoEditTriggers)
        self.setSelectionBehavior(QTableWidget.SelectRows)
        self.setStyleSheet("QTableWidget { gridline-color: #d0d0d0; }")


class ArmControlGUI(QMainWindow):
    JOINT_NAMES = ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5', 'Joint_6']

    def __init__(self, node: ArmNode):
        super().__init__()
        self.setWindowTitle('机械臂控制面板')
        self.setGeometry(100, 100, 1000, 650)
        self.node = node
        self._closing = False
        self._sync_enabled = False
        self._homed = False
        self._diff_too_large = False
        self._admin_unlocked = False

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        title_label = QLabel('机械臂控制面板')
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet('font-size: 18px; font-weight: bold; margin: 10px;')
        main_layout.addWidget(title_label)

        estop_layout = QHBoxLayout()
        self.btn_estop = QPushButton('⚠ 急 停 ⚠')
        self.btn_estop.setStyleSheet(
            'QPushButton {'
            '  background-color: #D32F2F; color: white; font-weight: bold; font-size: 22px;'
            '  padding: 15px; border: 3px solid #B71C1C; border-radius: 8px;'
            '  min-height: 60px;'
            '}'
            'QPushButton:hover { background-color: #F44336; }'
            'QPushButton:pressed { background-color: #B71C1C; }'
        )
        self.btn_estop.clicked.connect(self._on_estop)
        estop_layout.addWidget(self.btn_estop)
        main_layout.addLayout(estop_layout)

        motor_group = QGroupBox('电机控制')
        motor_layout = QVBoxLayout(motor_group)

        btn_row1 = QHBoxLayout()
        self.btn_can_connect = QPushButton('连接 CAN')
        self.btn_can_connect.setStyleSheet(
            'QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 8px; }'
        )
        self.btn_can_connect.clicked.connect(self._on_can_connect)

        self.btn_can_disconnect = QPushButton('断开 CAN')
        self.btn_can_disconnect.setStyleSheet(
            'QPushButton { background-color: #f44336; color: white; font-weight: bold; padding: 8px; }'
        )
        self.btn_can_disconnect.setEnabled(False)
        self.btn_can_disconnect.clicked.connect(self._on_can_disconnect)

        self.btn_enable = QPushButton('使能电机')
        self.btn_enable.setStyleSheet(
            'QPushButton { background-color: #2196F3; color: white; font-weight: bold; padding: 8px; }'
        )
        self.btn_enable.clicked.connect(self._on_enable_motors)
        self.btn_enable.setEnabled(False)

        self.btn_disable = QPushButton('失能电机')
        self.btn_disable.setStyleSheet(
            'QPushButton { background-color: #9E9E9E; color: white; font-weight: bold; padding: 8px; }'
        )
        self.btn_disable.clicked.connect(self._on_disable_motors)
        self.btn_disable.setEnabled(False)

        btn_row1.addWidget(self.btn_can_connect)
        btn_row1.addWidget(self.btn_can_disconnect)
        btn_row1.addWidget(self.btn_enable)
        btn_row1.addWidget(self.btn_disable)
        motor_layout.addLayout(btn_row1)

        btn_row2 = QHBoxLayout()
        self.btn_home = QPushButton('回零 (0°)')
        self.btn_home.setStyleSheet(
            'QPushButton { background-color: #FF9800; color: white; font-weight: bold; padding: 10px; font-size: 14px; }'
        )
        self.btn_home.clicked.connect(self._on_home)
        self.btn_home.setEnabled(False)

        self.chk_sync = QCheckBox('实时同步到电机')
        self.chk_sync.setStyleSheet('font-size: 13px; font-weight: bold;')
        self.chk_sync.stateChanged.connect(self._on_sync_changed)
        self.chk_sync.setEnabled(False)

        self.btn_sync_once = QPushButton('单次同步')
        self.btn_sync_once.setStyleSheet(
            'QPushButton { background-color: #673AB7; color: white; font-weight: bold; padding: 10px; }'
        )
        self.btn_sync_once.clicked.connect(self._on_sync_once)
        self.btn_sync_once.setEnabled(False)

        btn_row2.addWidget(self.btn_home)
        btn_row2.addStretch()
        btn_row2.addWidget(self.chk_sync)
        btn_row2.addWidget(self.btn_sync_once)
        motor_layout.addLayout(btn_row2)

        self.can_status_label = QLabel('CAN: 未连接')
        self.can_status_label.setStyleSheet('color: #f44336; font-weight: bold; font-size: 13px;')
        motor_layout.addWidget(self.can_status_label)

        main_layout.addWidget(motor_group)

        ratio_group = QGroupBox('减速比')
        ratio_layout = QVBoxLayout(ratio_group)

        ratio_top = QHBoxLayout()
        self.btn_admin = QPushButton('管理员解锁')
        self.btn_admin.setStyleSheet(
            'QPushButton { background-color: #607D8B; color: white; font-weight: bold; padding: 5px 12px; }'
        )
        self.btn_admin.clicked.connect(self._on_admin_unlock)
        self.admin_status = QLabel('🔒 已锁定')
        self.admin_status.setStyleSheet('color: #f44336; font-weight: bold; font-size: 12px;')
        ratio_top.addStretch()
        ratio_top.addWidget(self.btn_admin)
        ratio_top.addWidget(self.admin_status)
        ratio_layout.addLayout(ratio_top)

        ratio_row = QHBoxLayout()
        self.ratio_spins = []
        for i in range(6):
            col_layout = QVBoxLayout()
            name_label = QLabel(self.JOINT_NAMES[i])
            name_label.setAlignment(Qt.AlignCenter)
            name_label.setStyleSheet('font-weight: bold; font-size: 11px;')

            spin = QDoubleSpinBox()
            spin.setRange(1.0, 9999.0)
            spin.setValue(1.0)
            spin.setDecimals(4)
            spin.setSingleStep(1.0)
            spin.setAlignment(Qt.AlignCenter)
            spin.setStyleSheet('font-size: 13px;')
            spin.setEnabled(False)
            spin.valueChanged.connect(self._on_ratio_changed)

            col_layout.addWidget(name_label)
            col_layout.addWidget(spin)
            ratio_row.addLayout(col_layout)
            self.ratio_spins.append(spin)

        ratio_layout.addLayout(ratio_row)
        main_layout.addWidget(ratio_group)

        dir_group = QGroupBox('电机方向')
        dir_layout = QVBoxLayout(dir_group)

        self.dir_checks = []
        dir_row = QHBoxLayout()
        for i in range(6):
            col_layout = QVBoxLayout()
            name_label = QLabel(self.JOINT_NAMES[i])
            name_label.setAlignment(Qt.AlignCenter)
            name_label.setStyleSheet('font-weight: bold; font-size: 11px;')

            chk = QCheckBox('正向')
            chk.setChecked(True)
            chk.setStyleSheet('font-size: 12px;')
            chk.setEnabled(False)
            chk.stateChanged.connect(self._on_dir_changed)

            col_layout.addWidget(name_label)
            col_layout.addWidget(chk)
            col_layout.setAlignment(chk, Qt.AlignCenter)
            dir_row.addLayout(col_layout)
            self.dir_checks.append(chk)

        dir_layout.addLayout(dir_row)
        main_layout.addWidget(dir_group)

        self._init_config_to_ui()

        joint_group = QGroupBox('关节状态')
        joint_layout = QVBoxLayout(joint_group)

        self.table = JointStateTable()
        joint_layout.addWidget(self.table)

        main_layout.addWidget(joint_group)

        curve_group = QGroupBox('关节曲线')
        curve_layout = QHBoxLayout(curve_group)

        curve_ctrl = QVBoxLayout()
        curve_ctrl.setSpacing(2)

        ctrl_title = QLabel('显示曲线')
        ctrl_title.setAlignment(Qt.AlignCenter)
        ctrl_title.setStyleSheet('font-weight: bold; font-size: 12px; margin-bottom: 4px;')
        curve_ctrl.addWidget(ctrl_title)

        self._curve_joint_checks = []
        self._curve_motor_checks = []
        joint_colors = ['#1565C0', '#2E7D32', '#E65100', '#6A1B9A', '#C62828', '#00838F']
        motor_colors = ['#42A5F5', '#66BB6A', '#FFA726', '#AB47BC', '#EF5350', '#26C6DA']

        for i, name in enumerate(self.JOINT_NAMES):
            row = QHBoxLayout()
            row.setSpacing(2)

            j_chk = QCheckBox('关节')
            j_chk.setChecked(True)
            j_chk.setStyleSheet(f'color: {joint_colors[i]}; font-size: 10px; font-weight: bold;')
            j_chk.stateChanged.connect(self._on_curve_visibility_changed)
            row.addWidget(j_chk)
            self._curve_joint_checks.append(j_chk)

            m_chk = QCheckBox('电机')
            m_chk.setChecked(True)
            m_chk.setStyleSheet(f'color: {motor_colors[i]}; font-size: 10px; font-weight: bold;')
            m_chk.stateChanged.connect(self._on_curve_visibility_changed)
            row.addWidget(m_chk)
            self._curve_motor_checks.append(m_chk)

            name_lbl = QLabel(name.replace('Joint_', 'J').replace('Gripper', 'G'))
            name_lbl.setStyleSheet('font-size: 10px; min-width: 22px;')
            row.addWidget(name_lbl)

            curve_ctrl.addLayout(row)

        curve_ctrl.addStretch()

        self.curve_clear_btn = QPushButton('清除')
        self.curve_clear_btn.setStyleSheet('padding: 3px 8px;')
        self.curve_clear_btn.clicked.connect(self._on_clear_curve)
        curve_ctrl.addWidget(self.curve_clear_btn)

        curve_layout.addLayout(curve_ctrl)

        self.curve_widget = pg.PlotWidget()
        self.curve_widget.setBackground('w')
        self.curve_widget.setLabel('left', '角度', units='°')
        self.curve_widget.setLabel('bottom', '时间', units='s')
        self.curve_widget.addLegend(offset=(80, 10))
        self.curve_widget.showGrid(x=True, y=True, alpha=0.3)
        curve_layout.addWidget(self.curve_widget)

        self._curve_joint_items = []
        self._curve_motor_items = []
        for i in range(6):
            j_item = pg.PlotCurveItem(
                pen=pg.mkPen(color=joint_colors[i], width=2),
                name=f'{self.JOINT_NAMES[i]} 关节')
            m_item = pg.PlotCurveItem(
                pen=pg.mkPen(color=motor_colors[i], width=2, style=Qt.DashLine),
                name=f'{self.JOINT_NAMES[i]} 电机')
            self.curve_widget.addItem(j_item)
            self.curve_widget.addItem(m_item)
            self._curve_joint_items.append(j_item)
            self._curve_motor_items.append(m_item)

        self._curve_max_points = 500
        self._curve_times = collections.deque(maxlen=self._curve_max_points)
        self._curve_joint_data = [collections.deque(maxlen=self._curve_max_points) for _ in range(6)]
        self._curve_motor_data = [collections.deque(maxlen=self._curve_max_points) for _ in range(6)]
        self._curve_start_time = None

        main_layout.addWidget(curve_group)

        status_layout = QHBoxLayout()
        self.status_label = QLabel('状态：等待关节数据...')
        self.status_label.setStyleSheet('color: #666; font-style: italic;')
        status_layout.addWidget(self.status_label)
        main_layout.addLayout(status_layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.on_timer)
        self.timer.start(10)

        self.node.start_spinning()

    def _on_can_connect(self):
        success, msg = self.node.connect_can(PCAN_USBBUS1, PCAN_BAUD_500K, False)
        if success:
            self.can_status_label.setText('CAN: 已连接')
            self.can_status_label.setStyleSheet('color: #4CAF50; font-weight: bold; font-size: 13px;')
            self.btn_can_connect.setEnabled(False)
            self.btn_can_disconnect.setEnabled(True)
            self.btn_enable.setEnabled(True)
            self.btn_disable.setEnabled(True)
            self.btn_home.setEnabled(True)
            self.chk_sync.setEnabled(False)
            self.btn_sync_once.setEnabled(False)

            self.node.enable_motors(True)
            self.can_status_label.setText('CAN: 已连接 | 电机已使能')
            QMessageBox.information(self, '提示', '请先执行回零操作，再开启实时同步！')
        else:
            QMessageBox.critical(self, 'CAN 错误', f'连接失败:\n{msg}')

    def _on_estop(self):
        self._sync_enabled = False
        self.chk_sync.setChecked(False)
        self._homed = False
        self._diff_too_large = False
        self.node.stop_homing()
        if self.node.can.is_open:
            self.node.enable_motors(False)
        self.btn_can_connect.setEnabled(True)
        self.btn_can_disconnect.setEnabled(False)
        self.btn_enable.setEnabled(False)
        self.btn_disable.setEnabled(False)
        self.btn_home.setEnabled(False)
        self.chk_sync.setEnabled(False)
        self.btn_sync_once.setEnabled(False)
        self.can_status_label.setText('⚠ 急停已触发 | 电机已失能')
        self.can_status_label.setStyleSheet('color: #D32F2F; font-weight: bold; font-size: 14px;')
        self.status_label.setText('状态：急停！请重新连接并回零')
        self.status_label.setStyleSheet('color: #D32F2F; font-weight: bold;')
        QMessageBox.critical(self, '急停', '急停已触发！\n电机已失能，所有操作已复位。\n请重新连接 CAN 并回零。')

    def _on_can_disconnect(self):
        self._sync_enabled = False
        self.chk_sync.setChecked(False)
        self._homed = False
        self._diff_too_large = False
        self.node.stop_homing()
        self.node.disconnect_can()
        self.can_status_label.setText('CAN: 未连接')
        self.can_status_label.setStyleSheet('color: #f44336; font-weight: bold; font-size: 13px;')
        self.btn_can_connect.setEnabled(True)
        self.btn_can_disconnect.setEnabled(False)
        self.btn_enable.setEnabled(False)
        self.btn_disable.setEnabled(False)
        self.btn_home.setEnabled(False)
        self.chk_sync.setEnabled(False)
        self.btn_sync_once.setEnabled(False)

    def _on_enable_motors(self):
        self.node.enable_motors(True)
        self.can_status_label.setText('CAN: 已连接 | 电机已使能')

    def _on_disable_motors(self):
        self._sync_enabled = False
        self.chk_sync.setChecked(False)
        self.node.enable_motors(False)
        self.can_status_label.setText('CAN: 已连接 | 电机已失能')

    def _on_home(self):
        self._sync_enabled = False
        self.chk_sync.setChecked(False)
        self.node.home_motors()
        self.btn_home.setEnabled(False)
        self.btn_enable.setEnabled(False)
        self.btn_disable.setEnabled(False)
        self.chk_sync.setEnabled(False)
        self.btn_sync_once.setEnabled(False)
        self.can_status_label.setText('CAN: 已连接 | S曲线回零中...')
        self._home_check_timer = QTimer()
        self._home_check_timer.timeout.connect(self._check_homing_done)
        self._home_check_timer.start(10)

    def _check_homing_done(self):
        if not self.node._homing:
            self._home_check_timer.stop()
            self.btn_home.setEnabled(True)
            self.btn_enable.setEnabled(True)
            self.btn_disable.setEnabled(True)
            self._homed = True
            self.can_status_label.setText('CAN: 已连接 | 回零完成')
            if self._check_joint_motor_diff():
                self._diff_too_large = True
                self.chk_sync.setEnabled(False)
                self.btn_sync_once.setEnabled(False)
            else:
                self._diff_too_large = False
                self.chk_sync.setEnabled(True)
                self.btn_sync_once.setEnabled(True)

    def _check_joint_motor_diff(self) -> bool:
        joint_data = self.node.get_joint_data()
        if joint_data is None:
            return False
        motor_positions = self.node.get_motor_positions()
        names = joint_data.get('names', [])
        positions = joint_data.get('positions', [])
        threshold_deg = 5.0
        diff_joints = []
        for i, name in enumerate(names):
            if name in self.JOINT_NAMES and i < len(positions):
                motor_idx = self.JOINT_NAMES.index(name)
                joint_deg = math.degrees(positions[i])
                motor_deg = motor_positions[motor_idx]
                if abs(joint_deg - motor_deg) > threshold_deg:
                    diff_joints.append(
                        f'{name}: 关节={joint_deg:.1f}° 电机={motor_deg:.1f}° 差值={abs(joint_deg - motor_deg):.1f}°'
                    )
        if diff_joints:
            msg = '回零后以下关节位置与电机角度偏差过大，请复位虚拟模型！\n\n' + '\n'.join(diff_joints)
            QMessageBox.warning(self, '位置偏差警告', msg)
            return True
        return False

    def _auto_check_diff_resolved(self):
        joint_data = self.node.get_joint_data()
        if joint_data is None:
            return
        motor_positions = self.node.get_motor_positions()
        names = joint_data.get('names', [])
        positions = joint_data.get('positions', [])
        threshold_deg = 5.0
        for i, name in enumerate(names):
            if name in self.JOINT_NAMES and i < len(positions):
                motor_idx = self.JOINT_NAMES.index(name)
                joint_deg = math.degrees(positions[i])
                motor_deg = motor_positions[motor_idx]
                if abs(joint_deg - motor_deg) > threshold_deg:
                    return
        self._diff_too_large = False
        self.chk_sync.setEnabled(True)
        self.btn_sync_once.setEnabled(True)
        self.can_status_label.setText('CAN: 已连接 | 偏差已消除，可开启同步')

    def _init_config_to_ui(self):
        for i, spin in enumerate(self.ratio_spins):
            spin.blockSignals(True)
            spin.setValue(self.node.config.REDUCTION_RATIOS[i])
            spin.blockSignals(False)
        for i, chk in enumerate(self.dir_checks):
            chk.blockSignals(True)
            chk.setChecked(self.node.config.DIRECTION_MAP[i])
            chk.setText('正向' if self.node.config.DIRECTION_MAP[i] else '反向')
            chk.blockSignals(False)

    def _on_admin_unlock(self):
        if self._admin_unlocked:
            self._admin_unlocked = False
            self.btn_admin.setText('管理员解锁')
            self.admin_status.setText('🔒 已锁定')
            self.admin_status.setStyleSheet('color: #f44336; font-weight: bold; font-size: 12px;')
            for spin in self.ratio_spins:
                spin.setEnabled(False)
            for chk in self.dir_checks:
                chk.setEnabled(False)
            return

        pwd, ok = QInputDialog.getText(
            self, '管理员验证', '请输入管理员密码：',
            QLineEdit.Password, ''
        )
        if ok and pwd == 'admin':
            self._admin_unlocked = True
            self.btn_admin.setText('管理员锁定')
            self.admin_status.setText('🔓 已解锁')
            self.admin_status.setStyleSheet('color: #4CAF50; font-weight: bold; font-size: 12px;')
            if not self._sync_enabled:
                for spin in self.ratio_spins:
                    spin.setEnabled(True)
                for chk in self.dir_checks:
                    chk.setEnabled(True)
        elif ok:
            QMessageBox.warning(self, '验证失败', '密码错误！')

    def _on_ratio_changed(self):
        for i, spin in enumerate(self.ratio_spins):
            self.node.config.REDUCTION_RATIOS[i] = spin.value()
        self.node.config.save()
        self._require_rehome('减速比已修改')

    def _on_dir_changed(self):
        for i, chk in enumerate(self.dir_checks):
            self.node.config.DIRECTION_MAP[i] = chk.isChecked()
            chk.setText('正向' if chk.isChecked() else '反向')
        self.node.config.save()
        self._require_rehome('电机方向已修改')

    def _require_rehome(self, reason):
        self._sync_enabled = False
        self.chk_sync.setChecked(False)
        self._homed = False
        self._diff_too_large = False
        self.chk_sync.setEnabled(False)
        self.btn_sync_once.setEnabled(False)
        self.can_status_label.setText(f'CAN: 已连接 | {reason}，请重新回零')
        QMessageBox.warning(self, '提示', f'{reason}，请重新执行回零操作！')

    def _on_sync_changed(self, state):
        self._sync_enabled = (state == Qt.Checked)
        can_edit = not self._sync_enabled and self._admin_unlocked
        for spin in self.ratio_spins:
            spin.setEnabled(can_edit)
        for chk in self.dir_checks:
            chk.setEnabled(can_edit)
        if self._sync_enabled:
            self.can_status_label.setText('CAN: 已连接 | 同步已开启')
        else:
            self.can_status_label.setText('CAN: 已连接 | 同步已关闭')

    def _on_sync_once(self):
        data = self.node.get_joint_data()
        if data is not None:
            self.node.sync_positions_to_motors(data)
            self.can_status_label.setText('CAN: 已连接 | 已单次同步')
        else:
            self.status_label.setText('状态：无关节数据可同步')

    def on_timer(self):
        if self._closing:
            return

        if self._diff_too_large and self._homed:
            self._auto_check_diff_resolved()

        motor_positions = self.node.get_motor_positions()
        motor_raw_positions = self.node.get_motor_raw_positions()
        joint_data = self.node.get_joint_data()

        self.table.blockSignals(True)

        if joint_data is not None:
            names = joint_data['names']
            positions = joint_data['positions']
            velocities = joint_data['velocities'] or []
            efforts = joint_data['efforts'] or []

            self.table.setRowCount(len(names))

            for i, (name, pos) in enumerate(zip(names, positions)):
                vel = velocities[i] if i < len(velocities) else 0.0
                eff = efforts[i] if i < len(efforts) else 0.0
                pos_deg = math.degrees(pos)

                name_item = QTableWidgetItem(str(name))
                pos_item = QTableWidgetItem(f'{pos:.4f} / {pos_deg:.1f}°')

                if name in self.JOINT_NAMES:
                    motor_idx = self.JOINT_NAMES.index(name)
                    motor_deg = motor_positions[motor_idx]
                    motor_rad = math.radians(motor_deg)
                    motor_item = QTableWidgetItem(f'{motor_deg:.2f}° / {motor_rad:.4f}')
                    raw_deg = motor_raw_positions[motor_idx]
                    raw_item = QTableWidgetItem(f'{raw_deg:.2f}')
                    diff_deg = abs(pos_deg - motor_deg)
                    diff_item = QTableWidgetItem(f'{diff_deg:.2f}')
                    if diff_deg > 5.0:
                        diff_item.setForeground(Qt.red)
                else:
                    motor_item = QTableWidgetItem('-')
                    raw_item = QTableWidgetItem('-')
                    diff_item = QTableWidgetItem('-')

                vel_item = QTableWidgetItem(f'{vel:.4f}')
                eff_item = QTableWidgetItem(f'{eff:.4f}')

                name_item.setTextAlignment(Qt.AlignCenter)
                pos_item.setTextAlignment(Qt.AlignCenter)
                motor_item.setTextAlignment(Qt.AlignCenter)
                raw_item.setTextAlignment(Qt.AlignCenter)
                vel_item.setTextAlignment(Qt.AlignCenter)
                eff_item.setTextAlignment(Qt.AlignCenter)
                diff_item.setTextAlignment(Qt.AlignCenter)

                self.table.setItem(i, 0, name_item)
                self.table.setItem(i, 1, pos_item)
                self.table.setItem(i, 2, motor_item)
                self.table.setItem(i, 3, raw_item)
                self.table.setItem(i, 4, vel_item)
                self.table.setItem(i, 5, eff_item)
                self.table.setItem(i, 6, diff_item)

            self.status_label.setText(f'状态：正在接收 {len(names)} 个关节数据')
            self.status_label.setStyleSheet('color: green;')

            if self._sync_enabled and self.node.can.is_open:
                self.node.sync_positions_to_motors(joint_data)

        elif self.node.can.is_open:
            self.table.setRowCount(self.MOTOR_COUNT)
            for i in range(self.MOTOR_COUNT):
                motor_deg = motor_positions[i]
                motor_rad = math.radians(motor_deg)
                raw_deg = motor_raw_positions[i]

                name_item = QTableWidgetItem(self.JOINT_NAMES[i])
                pos_item = QTableWidgetItem('-')
                motor_item = QTableWidgetItem(f'{motor_deg:.2f}° / {motor_rad:.4f}')
                raw_item = QTableWidgetItem(f'{raw_deg:.2f}')
                vel_item = QTableWidgetItem('-')
                eff_item = QTableWidgetItem('-')
                diff_item = QTableWidgetItem('-')

                name_item.setTextAlignment(Qt.AlignCenter)
                pos_item.setTextAlignment(Qt.AlignCenter)
                motor_item.setTextAlignment(Qt.AlignCenter)
                raw_item.setTextAlignment(Qt.AlignCenter)
                vel_item.setTextAlignment(Qt.AlignCenter)
                eff_item.setTextAlignment(Qt.AlignCenter)
                diff_item.setTextAlignment(Qt.AlignCenter)

                self.table.setItem(i, 0, name_item)
                self.table.setItem(i, 1, pos_item)
                self.table.setItem(i, 2, motor_item)
                self.table.setItem(i, 3, raw_item)
                self.table.setItem(i, 4, vel_item)
                self.table.setItem(i, 5, eff_item)
                self.table.setItem(i, 6, diff_item)

            self.status_label.setText(f'状态：CAN 已连接，电机反馈中 ({self.MOTOR_COUNT} 个电机)')
            self.status_label.setStyleSheet('color: #2196F3;')

        self.table.blockSignals(False)
        self.table.viewport().update()

        self._update_curve(joint_data, motor_positions)

    def _update_curve(self, joint_data, motor_positions):
        now = time.time()
        if self._curve_start_time is None:
            self._curve_start_time = now
        self._curve_times.append(now - self._curve_start_time)

        joint_degs = [None] * 6
        if joint_data is not None:
            names = joint_data.get('names', [])
            positions = joint_data.get('positions', [])
            for i, (n, p) in enumerate(zip(names, positions)):
                if n in self.JOINT_NAMES:
                    joint_degs[self.JOINT_NAMES.index(n)] = math.degrees(p)

        for i in range(6):
            jd = joint_degs[i]
            md = motor_positions[i] if i < len(motor_positions) else None
            self._curve_joint_data[i].append(jd if jd is not None else 0.0)
            self._curve_motor_data[i].append(md if md is not None else 0.0)

        if len(self._curve_times) > 1:
            t = list(self._curve_times)
            for i in range(6):
                self._curve_joint_items[i].setData(t, list(self._curve_joint_data[i]))
                self._curve_motor_items[i].setData(t, list(self._curve_motor_data[i]))

    def _on_curve_visibility_changed(self):
        for i in range(6):
            self._curve_joint_items[i].setVisible(self._curve_joint_checks[i].isChecked())
            self._curve_motor_items[i].setVisible(self._curve_motor_checks[i].isChecked())

    def _on_clear_curve(self):
        self._curve_times.clear()
        for i in range(6):
            self._curve_joint_data[i].clear()
            self._curve_motor_data[i].clear()
            self._curve_joint_items[i].setData([], [])
            self._curve_motor_items[i].setData([], [])
        self._curve_start_time = None

    def closeEvent(self, event):
        self._closing = True
        self.timer.stop()
        self.node.stop_spinning()
        self.node.destroy_node()
        event.accept()


def main(args=None):
    deps_dir = '/tmp/robot_deps'
    if os.path.isdir(deps_dir) and deps_dir not in sys.path:
        sys.path.insert(0, deps_dir)

    global pg
    import pyqtgraph as pg

    rclpy.init(args=args)
    node = ArmNode()

    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    window = ArmControlGUI(node)
    window.show()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
