import sys
import time
import threading
import math
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from .can_interface import CANInterface
from .PCANBasic import PCAN_BAUD_500K


class MotorConfig:
    def __init__(self):
        self.REDUCTION_RATIOS = [1, 1, 1, 1, 1, 1]
        self.DIRECTION_MAP = [True, True, True, True, True, True]
        self.SPEEDS = [2500, 2500, 2500, 2500, 2500, 2500]


class MotorControlNode(Node):
    JOINT_NAMES = ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5', 'Joint_6']
    MOTOR_COUNT = 6

    def __init__(self):
        super().__init__('motor_control_node')

        self.declare_parameter('can_channel', 0x51)
        self.declare_parameter('can_baudrate', 500000)
        self.declare_parameter('can_fd_mode', False)
        self.declare_parameter('query_interval', 0.1)
        self.declare_parameter('auto_connect', True)

        self.config = MotorConfig()
        self.can = CANInterface()
        self.motor_positions = [0.0] * self.MOTOR_COUNT
        self._lock = threading.Lock()
        self._receive_running = False
        self._receive_thread: Optional[threading.Thread] = None
        self._query_paused = False

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.cmd_sub = self.create_subscription(
            JointState,
            '/joint_command',
            self._cmd_callback,
            qos
        )

        self.state_pub = self.create_publisher(
            JointState,
            '/motor_states',
            qos
        )

        self.enable_srv = self.create_service(
            SetBool,
            '/motor_enable',
            self._enable_callback
        )

        self._query_timer = self.create_timer(
            self.get_parameter('query_interval').value,
            self._query_timer_callback
        )

        if self.get_parameter('auto_connect').value:
            self._connect_can()

        self.get_logger().info('Motor control node initialized')

    def _connect_can(self):
        channel = self.get_parameter('can_channel').value
        baudrate = self.get_parameter('can_baudrate').value
        fd_mode = self.get_parameter('can_fd_mode').value

        from .PCANBasic import PCAN_BAUD_500K, PCAN_BAUD_1M, PCAN_BAUD_250K, PCAN_BAUD_125K
        baudrate_map = {
            1000000: PCAN_BAUD_1M,
            500000: PCAN_BAUD_500K,
            250000: PCAN_BAUD_250K,
            125000: PCAN_BAUD_125K,
        }
        pcan_baud = baudrate_map.get(baudrate, PCAN_BAUD_500K)

        success, msg = self.can.initialize(channel, pcan_baud, fd_mode)
        if success:
            self.get_logger().info(f'CAN connected: channel=0x{channel:02X}, baudrate={baudrate}')
            self._start_receiving()
            self._auto_enable_motors()
        else:
            self.get_logger().error(f'CAN connection failed: {msg}')

    def _auto_enable_motors(self):
        self.get_logger().info('Auto enabling all motors...')
        self._send_enable_command(None, True)
        self.get_logger().info('All motors enabled, sending home position (0°)...')
        time.sleep(0.1)
        for motor_id in range(1, self.MOTOR_COUNT + 1):
            self._send_position_command(motor_id, 0.0)
            time.sleep(0.01)
        self.get_logger().info('Home position commands sent')

    def _start_receiving(self):
        if self._receive_running:
            return
        self._receive_running = True
        self._receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
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
            except Exception as e:
                self.get_logger().error(f'Receive error: {str(e)}')
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

            if not motor_direction:
                angle = -angle

            with self._lock:
                self.motor_positions[motor_index] = angle

            self._publish_motor_states()

        except Exception as e:
            self.get_logger().error(f'Process message error: {str(e)}')

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

    def _cmd_callback(self, msg: JointState):
        if not self.can.is_open:
            self.get_logger().warn('CAN not connected, ignoring command')
            return

        for i, name in enumerate(msg.name):
            if name in self.JOINT_NAMES:
                motor_id = self.JOINT_NAMES.index(name) + 1
                if i < len(msg.position):
                    position_deg = math.degrees(msg.position[i])
                    self._send_position_command(motor_id, position_deg)

    def _send_position_command(self, motor_id: int, position: float):
        if not self.can.is_open:
            self.get_logger().warn('CAN not connected')
            return False

        if not 1 <= motor_id <= self.MOTOR_COUNT:
            self.get_logger().error(f'Invalid motor ID: {motor_id}')
            return False

        try:
            self._query_paused = True

            can_id = 0x100 + (motor_id - 1) * 0x100
            position_value = int(position * 10)

            motor_index = motor_id - 1
            motor_direction = self.config.DIRECTION_MAP[motor_index]
            motor_speed = self.config.SPEEDS[motor_index] * 10

            if motor_direction:
                direction = 0x00 if position >= 0 else 0x01
            else:
                direction = 0x01 if position >= 0 else 0x00

            reduction_ratio = self.config.REDUCTION_RATIOS[motor_index]
            if reduction_ratio > 0:
                position_with_reduction = int(abs(position_value) * reduction_ratio)
            else:
                position_with_reduction = abs(position_value)

            result = self._send_position_frame(can_id, direction, motor_speed, position_with_reduction)

            self._query_paused = False
            return result

        except Exception as e:
            self._query_paused = False
            self.get_logger().error(f'Send position command failed: {str(e)}')
            return False

    def _send_position_frame(self, can_id: int, direction: int, speed: int, position: int) -> bool:
        try:
            pos_bytes = position.to_bytes(4, byteorder='big')
            speed_bytes = speed.to_bytes(2, byteorder='big')

            data_bytes = [0xFB, direction] + list(speed_bytes) + list(pos_bytes) + [0x01, 0x00, 0x6B]

            first_package = data_bytes[:8]
            second_package = data_bytes[8:]

            success1, msg1 = self.can.send_message(can_id, first_package, True)
            if not success1:
                self.get_logger().error(f'First frame send failed: {msg1}')
                return False

            if len(second_package) > 0:
                second_package_with_fb = [0xFB] + second_package
                success2, msg2 = self.can.send_message(can_id + 1, second_package_with_fb, True)
                if not success2:
                    self.get_logger().error(f'Second frame send failed: {msg2}')
                    return False

            return True

        except Exception as e:
            self.get_logger().error(f'Send position frame failed: {str(e)}')
            return False

    def _send_enable_command(self, motor_id: Optional[int], enable: bool) -> bool:
        if not self.can.is_open:
            self.get_logger().warn('CAN not connected')
            return False

        try:
            self._query_paused = True

            enable_state = 0x01 if enable else 0x00
            data = [0xF3, 0xAB, enable_state, 0x00, 0x6B]

            if motor_id is None:
                motor_ids = range(1, self.MOTOR_COUNT + 1)
            else:
                if not 1 <= motor_id <= self.MOTOR_COUNT:
                    self.get_logger().error(f'Invalid motor ID: {motor_id}')
                    self._query_paused = False
                    return False
                motor_ids = [motor_id]

            success_all = True
            for mid in motor_ids:
                can_id = 0x100 + (mid - 1) * 0x100
                success, msg = self.can.send_message(can_id, data, True)
                if not success:
                    self.get_logger().error(f'Motor {mid} enable command failed: {msg}')
                    success_all = False
                else:
                    status = 'enabled' if enable else 'disabled'
                    self.get_logger().info(f'Motor {mid} {status}')
                time.sleep(0.002)

            self._query_paused = False
            return success_all

        except Exception as e:
            self._query_paused = False
            self.get_logger().error(f'Send enable command failed: {str(e)}')
            return False

    def _send_position_query(self, motor_id: Optional[int] = None):
        if not self.can.is_open:
            return False

        try:
            data = [0x36, 0x6b]

            if motor_id is None:
                motor_ids = range(1, self.MOTOR_COUNT + 1)
            else:
                if not 1 <= motor_id <= self.MOTOR_COUNT:
                    return False
                motor_ids = [motor_id]

            for mid in motor_ids:
                can_id = 0x100 + (mid - 1) * 0x100
                self.can.send_message(can_id, data, True)
                if len(list(motor_ids)) > 1:
                    time.sleep(0.002)

            return True

        except Exception as e:
            self.get_logger().error(f'Send position query failed: {str(e)}')
            return False

    def _query_timer_callback(self):
        if self._query_paused:
            return
        self._send_position_query()

    def _enable_callback(self, request: SetBool.Request, response: SetBool.Response):
        success = self._send_enable_command(None, request.data)
        response.success = success
        response.message = f'Motors {"enabled" if request.data else "disabled"}'
        return response

    def get_motor_position(self, motor_id: int) -> Optional[float]:
        if not 1 <= motor_id <= self.MOTOR_COUNT:
            return None
        with self._lock:
            return self.motor_positions[motor_id - 1]

    def disconnect_can(self):
        self._query_paused = True
        self._stop_receiving()
        if self.can.is_open:
            success, msg = self.can.close()
            if success:
                self.get_logger().info('CAN disconnected')
            else:
                self.get_logger().error(f'CAN disconnect failed: {msg}')

    def destroy_node(self):
        self.disconnect_can()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
