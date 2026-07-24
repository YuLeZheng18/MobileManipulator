#!/usr/bin/env python3
"""
J1 单关节裸测脚本 (换驱动后验证用)。

只操作 J1 (motor_id=1, can_id=0x100)，绝不广播其余轴。
协议逻辑内联自 motor_control.py，配置读 arm_control/config/robot_arm_config.json 第 0 项。

安全约定:
  - 连上后先只读一次 J1 当前角度，不自动回零、不自动奔任何绝对位置
  - 点动是相对当前实测角的小增量 (jog)，一次一小步
  - 只使能 J1；退出时失能 J1 + 关 CAN
  - 速度用下面 SPEED_OVERRIDE 保守值覆盖 config

用法:  python3 test_j1.py
"""

import os
import sys
import json
import time
import threading

# 让脚本能 import 包内的 arm_control 子模块 (按包路径导入，保留相对 import)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arm_control.can_interface import CANInterface   # noqa: E402
from arm_control.PCANBasic import PCAN_BAUD_500K      # noqa: E402

# ---- 只测 J1 ----
MOTOR_ID = 1
CAN_ID = 0x100                # J1: 0x100 + (1-1)*0x100
CAN_CHANNEL = 0x51            # PCAN_USBBUS1，与 motor_control 默认一致
CAN_BAUD = PCAN_BAUD_500K

# 保守限制
SPEED_OVERRIDE = 100          # 覆盖 config 的 250，点动用更慢的速度
DEFAULT_STEP_DEG = 5.0        # 默认点动步长
MAX_STEP_DEG = 20.0           # 单次点动上限，防手滑输入大角度


def load_j1_config():
    """读 robot_arm_config.json 第 0 项 (J1)。读不到用安全默认。"""
    cfg_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'config', 'robot_arm_config.json'
    )
    reduction, direction = 1.0, True
    try:
        with open(cfg_path) as f:
            data = json.load(f)
        reduction = float(data['reduction_ratios'][0])
        direction = bool(data['direction_map'][0])
        print(f"[config] 读到 {cfg_path}")
    except Exception as e:
        print(f"[config] 读取失败({e})，用默认 reduction=1.0 direction=True")
    print(f"[config] J1: reduction_ratio={reduction}  direction_map={direction}")
    return reduction, direction


class J1Tester:
    def __init__(self):
        self.reduction, self.direction = load_j1_config()
        self.can = CANInterface()
        self.angle_deg = None          # J1 当前实测角(减速比后、方向修正后)，单位 deg
        self._lock = threading.Lock()
        self._rx_running = False
        self._rx_thread = None

    # ---------- CAN 连接 ----------
    def connect(self):
        ok, msg = self.can.initialize(CAN_CHANNEL, CAN_BAUD, False)
        if not ok:
            print(f"[can] 初始化失败: {msg}")
            return False
        print(f"[can] 已连接 channel=0x{CAN_CHANNEL:02X} baud=500K")
        self._start_rx()
        return True

    def _start_rx(self):
        self._rx_running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def _rx_loop(self):
        while self._rx_running:
            ok, msg = self.can.receive_message()
            if ok:
                self._parse(msg)
            else:
                time.sleep(0.005)

    def _parse(self, msg):
        """解析位置反馈帧 (逻辑同 motor_control._process_message，仅认 J1)。"""
        try:
            can_id = msg.get('id')
            data = msg.get('data')
            if not (0x100 <= can_id <= 0x600 and len(data) >= 7
                    and data[0] == 0x36 and data[-1] == 0x6b):
                return
            motor_index = (can_id >> 8) - 1
            if motor_index != MOTOR_ID - 1:      # 只认 J1
                return
            direction_byte = data[1]
            raw = int.from_bytes(data[2:6], byteorder='big', signed=False)
            angle = raw / 10.0
            if direction_byte == 0x01:
                angle = -angle
            if self.reduction > 0 and self.reduction != 1:
                angle = angle / self.reduction
            if not self.direction:
                angle = -angle
            with self._lock:
                self.angle_deg = angle
        except Exception as e:
            print(f"[rx] 解析异常: {e}")

    # ---------- 发送 ----------
    def query_angle(self):
        """发查询帧，请求 J1 上报当前角度。"""
        self.can.send_message(CAN_ID, [0x36, 0x6b], True)

    def enable(self, on):
        state = 0x01 if on else 0x00
        data = [0xF3, 0xAB, state, 0x00, 0x6B]
        ok, msg = self.can.send_message(CAN_ID, data, True)
        print(f"[j1] {'使能' if on else '失能'} -> {'ok' if ok else msg}")

    def move_to(self, target_deg):
        """发绝对位置指令 (逻辑同 motor_control，仅 J1)。target_deg 为关节角。"""
        position_value = int(target_deg * 10)
        if self.direction:
            dir_byte = 0x00 if target_deg >= 0 else 0x01
        else:
            dir_byte = 0x01 if target_deg >= 0 else 0x00
        if self.reduction > 0:
            pos = int(abs(position_value) * self.reduction)
        else:
            pos = abs(position_value)
        speed = SPEED_OVERRIDE * 10

        pos_bytes = pos.to_bytes(4, byteorder='big')
        speed_bytes = speed.to_bytes(2, byteorder='big')
        frame = [0xFB, dir_byte] + list(speed_bytes) + list(pos_bytes) + [0x01, 0x00, 0x6B]

        ok1, m1 = self.can.send_message(CAN_ID, frame[:8], True)
        if not ok1:
            print(f"[j1] 第一帧失败: {m1}")
            return
        ok2, m2 = self.can.send_message(CAN_ID + 1, [0xFB] + frame[8:], True)
        if not ok2:
            print(f"[j1] 第二帧失败: {m2}")
            return
        print(f"[j1] -> 目标 {target_deg:+.1f}° (speed={SPEED_OVERRIDE})")

    def read_angle(self, timeout=1.0):
        """请求并等待一次新角度读数。"""
        with self._lock:
            self.angle_deg = None
        self.query_angle()
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self._lock:
                if self.angle_deg is not None:
                    return self.angle_deg
            time.sleep(0.02)
        return None

    def close(self):
        self._rx_running = False
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
        if self.can.is_open:
            self.can.close()
            print("[can] 已关闭")


def main():
    print("=" * 50)
    print(" J1 单关节裸测 (只操作 J1，不碰其余轴)")
    print("=" * 50)
    t = J1Tester()
    if not t.connect():
        return

    try:
        # 第一步：只读，不动
        a = t.read_angle()
        if a is None:
            print("[j1] 未收到角度反馈！检查: 电机上电? CAN 终端电阻? 波特率? 先别使能。")
        else:
            print(f"[j1] 当前角度 = {a:+.2f}°  (先看清停在哪，再决定是否使能)")

        step = DEFAULT_STEP_DEG
        enabled = False
        while True:
            print("\n命令: [r]读角度  [e]使能  [d]失能  "
                  f"[+]/[-]点动{step}°  [s]改步长  [q]退出")
            cmd = input("> ").strip().lower()

            if cmd == 'q':
                break
            elif cmd == 'r':
                a = t.read_angle()
                print(f"[j1] 角度 = {a:+.2f}°" if a is not None else "[j1] 无反馈")
            elif cmd == 'e':
                t.enable(True)
                enabled = True
            elif cmd == 'd':
                t.enable(False)
                enabled = False
            elif cmd in ('+', '-'):
                if not enabled:
                    print("[j1] 未使能，先按 e。")
                    continue
                a = t.read_angle()
                if a is None:
                    print("[j1] 读不到当前角度，放弃本次点动。")
                    continue
                target = a + (step if cmd == '+' else -step)
                t.move_to(target)
                time.sleep(0.3)
                b = t.read_angle()
                print(f"[j1] 点动后角度 = {b:+.2f}°" if b is not None else "[j1] 点动后无反馈")
            elif cmd == 's':
                try:
                    v = float(input(f"新步长(deg，<= {MAX_STEP_DEG}): ").strip())
                    step = min(abs(v), MAX_STEP_DEG)
                    print(f"[j1] 步长 = {step}°")
                except ValueError:
                    print("无效输入")
            else:
                print("未知命令")
    except KeyboardInterrupt:
        pass
    finally:
        print("\n[j1] 退出中，失能 J1...")
        try:
            t.enable(False)
        except Exception:
            pass
        t.close()


if __name__ == '__main__':
    main()
