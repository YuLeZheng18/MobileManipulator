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
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from .can_interface import CANInterface
from .PCANBasic import PCAN_USBBUS1, PCAN_BAUD_500K


# 标定以仓库内 config 为唯一真源, 团队拉取即一致; home 文件仅作缺失兜底
HOME_CONFIG_PATH = os.path.expanduser('~/.robot_arm_config.json')


def _repo_config_path():
    try:
        return os.path.join(
            get_package_share_directory('arm_control'), 'config', 'robot_arm_config.json')
    except PackageNotFoundError:
        return None

# MoveIt/JTC 关节名 (Joint_11~16) 按顺序映射到电机 1~6
MOVEIT_JOINT_NAMES = ['Joint_11', 'Joint_12', 'Joint_13', 'Joint_14', 'Joint_15', 'Joint_16']
MOTOR_COUNT = 6


class MotorConfig:
    """与 joint_gui.py 共用 ~/.robot_arm_config.json, 保证仿真调好的参数与实车一致."""
    def __init__(self):
        self.REDUCTION_RATIOS = [50.0, 50.0, 30.0, 82.67, 62.5, 27.0]
        # 电机1、2(Joint_11/12)实车转向与模型相反, 默认取反; 指令与反馈都用此标志, 翻转后闭环自洽.
        # 与 joint_gui.py 保持一致; ~/.robot_arm_config.json 存在时会覆盖这里.
        self.DIRECTION_MAP = [True, True, False, False, False, False]
        self.SPEEDS = [250, 250, 150, 250, 250, 135]
        self.ACCELERATIONS = [500, 500, 500, 500, 500, 500]
        self.load()

    def load(self):
        # 优先级: 仓库 config > home 兜底 > 硬编码默认. 读到任一份即停.
        for path in (_repo_config_path(), HOME_CONFIG_PATH):
            if not path or not os.path.exists(path):
                continue
            try:
                with open(path, 'r') as f:
                    data = json.load(f)
                self.REDUCTION_RATIOS = data.get('reduction_ratios', self.REDUCTION_RATIOS)
                self.DIRECTION_MAP = data.get('direction_map', self.DIRECTION_MAP)
                self.SPEEDS = data.get('speeds', self.SPEEDS)
                self.ACCELERATIONS = data.get('accelerations', self.ACCELERATIONS)
                return
            except Exception:
                continue


class CanBridge(Node):
    def __init__(self):
        super().__init__('arm_can_bridge')

        self.declare_parameter('command_topic', '/arm_joint_commands')
        self.declare_parameter('state_topic', '/arm_joint_states')
        self.declare_parameter('send_rate_hz', 100.0)
        self.declare_parameter('query_rate_hz', 100.0)
        self.declare_parameter('auto_enable', True)
        # per-motor 速度**上限**(0xFD speed 字段, 0.1RPM/LSB, 电机轴). 0=用 json 的 SPEEDS.
        # ⚠️ 语义已变: 以前每帧恒发这个值, 现在只当上限 —— 实发值由指令流差分前馈算出,
        # 见 _feedforward_speed(). 恒发巡航值是"冲到位再空等"的一半原因.
        self.declare_parameter('motor_speeds', [0, 0, 0, 0, 0, 0])
        # per-motor 加减速度(0xFD 的加加/减加字段, RPM/s, 电机轴). 0=用 json 的 ACCELERATIONS.
        # ⚠️ 顿挫的另一半原因就在这: 500RPM/s 下 J1 走完一帧增量(电机轴 0.044 转)是**三角
        # 剖面** —— 峰值只 36RPM、需 145ms, 而 servo 帧周期 34ms, 每帧都在加速段就被下一帧
        # 打断, 巡航段根本不存在, motor_speeds 那个值从未跑到 ⇒ 又慢又顿。
        # 拉高到"加速段 << 帧周期"才有匀速段可言.
        self.declare_parameter('motor_accels', [0, 0, 0, 0, 0, 0])
        # 落后帧数 k: 实发速度按"用 k 个帧周期走完剩余距离"算, 使电机**永远差一点没到**,
        # 下一帧目标就来了 ⇒ 不存在"走完空等"。k>1 才有这个余量, k=1 是刚好到(会短暂停).
        # ⚠️ 别用"指令速度×系数"代替: 系数>1 提前到达→空等走停(2026-08-02 实测顿挫的直接
        #    原因); 系数<1 又会无界落后(开环前馈没有回正机制)。按剩余距离算才自校正 ——
        #    慢了距离变大自动加速, 快了自动减速, 稳态落后固定 k×每帧增量, 不累积。
        #    推导: e_{n+1} = e_n(1-1/k) + Δ, 不动点 e*=kΔ, |1-1/k|<1 收敛, 且 v=Δ/T 不衰减.
        self.declare_parameter('lag_frames', 1.3)
        # 速度下限(占 motor_speeds 的比例): 兜底防算出 0 —— 速度字段为 0 电机不动.
        self.declare_parameter('min_speed_frac', 0.02)
        # 发送死区(度): 目标相对上次实发变化<此值的电机不重发.
        # 目的: 轨迹到位后目标静止时停发, 不再对已到位电机每帧重启梯形规划器
        # (会触发堵转保护锁死), 同时让查询帧恢复->反馈不再饿死. 对齐 joint_gui「动时发/停时静」.
        # ⚠️ 死区是**逐电机**判的, 不是"任一轴变了就六轴全发" —— 详见 _changed_motors().
        self.declare_parameter('send_deadband_deg', 0.05)
        # 位置模式: 'fd'=梯形曲线(基线) / 'fb'=直通限速. 详见 _send_position_command 上方注释.
        # 点动用 fb(无梯形加减速, 不会每帧"瞬冲+空等"), 跑不通时一行切回 fd.
        self.declare_parameter('position_mode', 'fd')
        # 诊断: 查询循环额外发 0x37 读电机位置误差, 发到 /arm_pos_error (度, 电机轴).
        # 用途 —— 判定"点动一顿一顿"到底顿在哪一层, 三种波形对应三个相反的处理方向:
        #   误差每周期"从大跌到0再跳大" = 电机走完就空等(瞬冲+空等), 该换控制模式;
        #   误差持续很大跟不上         = 电机限速跟不上指令, 该降 servo 增量或提 speed;
        #   误差持续接近0             = 电机忠实跟随, 顿在上游(servo 出的增量本身不匀), 改 CAN 层白费.
        # 默认关: 查询翻倍占总线, 平时不需要.
        self.declare_parameter('query_pos_error', False)
        # 反馈看门狗: 超过这么久没收到任何 CAN 反馈就持续报 ERROR.
        # 为什么必须有: 反馈断了的时候 TopicBasedSystem 会**回显命令值**当测量值, 于是
        # /joint_states 照常 100Hz、JTC 见"位置已到" 8ms 报成功、MoveIt 报动作完成 ——
        # 整条链全线自欺, 而臂一动没动。2026-08-01 因此白查五小时(三条错误假设)。
        # 这个看门狗是**唯一**会在那种情况下出声的地方, 别删。
        self.declare_parameter('feedback_timeout_sec', 1.0)

        command_topic = self.get_parameter('command_topic').value
        state_topic = self.get_parameter('state_topic').value
        speeds_override = list(self.get_parameter('motor_speeds').value)
        accels_override = list(self.get_parameter('motor_accels').value)
        self._lag_frames = max(1.05, float(self.get_parameter('lag_frames').value))
        self._min_speed_frac = float(self.get_parameter('min_speed_frac').value)
        self._send_deadband_deg = float(self.get_parameter('send_deadband_deg').value)
        self._position_mode = str(self.get_parameter('position_mode').value).lower()
        if self._position_mode not in ('fd', 'fb'):
            self.get_logger().error('position_mode=%r 非法, 回落到 fd' % self._position_mode)
            self._position_mode = 'fd'
        self._query_pos_error = bool(self.get_parameter('query_pos_error').value)
        self._pos_err_deg = [0.0] * MOTOR_COUNT
        self._fb_timeout = float(self.get_parameter('feedback_timeout_sec').value)
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
            if i < len(accels_override) and accels_override[i] > 0:
                self.config.ACCELERATIONS[i] = accels_override[i]
        self.get_logger().info('电机速度上限=%s' % self.config.SPEEDS)
        self.get_logger().info('电机加减速度=%s RPM/s (落后帧数=%.2f, 速度下限=%.0f%%)' % (
            self.config.ACCELERATIONS, self._lag_frames, self._min_speed_frac * 100))
        self.get_logger().warn('位置模式=0x%s (%s), 位置误差诊断=%s' % (
            self._position_mode.upper(),
            '直通限速, 无梯形加减速' if self._position_mode == 'fb' else '梯形曲线',
            '开(/arm_pos_error)' if self._query_pos_error else '关'))
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
        # 指令流速度(度/秒, 输出轴), 由相邻两帧命令目标差分得到 —— TopicBasedSystem 发的
        # JointState 里 velocity 是空的(command_interface 只有 position), 只能自己差分.
        self._cmd_dt = 0.0             # 实测指令帧周期(秒), 滑动平均
        self._prev_cmd_time = 0.0
        # 电机反馈位置(度)
        self._motor_deg = [0.0] * MOTOR_COUNT
        # 反馈看门狗状态: 最后一次收到有效反馈的时刻(每个电机各记一份, 便于定位是哪几个哑了)
        self._last_fb_time = [0.0] * MOTOR_COUNT
        self._fb_alarm = False      # 已进告警态, 用于恢复时打一条"已恢复"

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
        # 诊断: 电机位置误差(度, 电机轴, 未除减速比). 仅 query_pos_error=true 时有数据.
        self._err_pub = self.create_publisher(
            Float32MultiArray, '/arm_pos_error', qos_be) if self._query_pos_error else None

        # 重新使能服务: 使能帧原先只在 _connect_can() 里发一次, 驱动一旦进保护态就再没有
        # 软件侧恢复路径, 只能物理断电重上(2026-08-01 就是这么恢复的)。有了这个服务可以先
        # 试软恢复。⚠️ 不做自动重发 —— 驱动进保护态往往有物理原因(堵转/碰撞), 自动重使能
        # 等于无声地把保护解掉再撞一次, 必须由人确认现场后手动调。
        self._enable_srv = self.create_service(Trigger, '~/reenable', self._on_reenable)

        # 调参用: 这几个值原先只在 __init__ 读一次快照, 试参数就得重启 can_bridge。
        # 而重启它在臂不在零位时有风险(命令值与实测分叉), 故做成可在线改。
        self.add_on_set_parameters_callback(self._on_set_params)

        self._connect_can()

    def _on_set_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'lag_frames':
                self._lag_frames = max(1.05, float(p.value))
            elif p.name == 'min_speed_frac':
                self._min_speed_frac = float(p.value)
            elif p.name == 'send_deadband_deg':
                self._send_deadband_deg = float(p.value)
            elif p.name == 'position_mode':
                v = str(p.value).lower()
                if v not in ('fd', 'fb'):
                    return SetParametersResult(
                        successful=False, reason='position_mode 只能是 fd 或 fb')
                self._position_mode = v
            else:
                continue
            self.get_logger().warn('参数在线改: %s = %r' % (p.name, p.value))
        return SetParametersResult(successful=True)

    def _on_reenable(self, req, res):
        del req
        if not self.can.is_open:
            res.success, res.message = False, 'CAN 未连接'
            return res
        self._enable_motors(True)
        # 使能后清掉发送记账, 让下一轮把当前目标重新发一遍(保护态期间的目标已丢)
        self._last_sent = None
        res.success = True
        res.message = '已重发六个电机的使能帧; 若仍无反馈则需检查驱动/接线/供电'
        return res

    # ---------- CAN 生命周期 ----------
    def _connect_can(self):
        ok, msg = self.can.initialize(self.can_channel, PCAN_BAUD_500K, False)
        if not ok:
            self.get_logger().error(f'CAN 初始化失败: {msg}')
            return
        self.get_logger().info('CAN 已连接')

        # 连上后立刻报一次总线状态: "CAN 已连接"只代表 PCAN 句柄开了, 与总线上有没有
        # 节点应答无关。启动即 BUSOFF/BUSHEAVY 就直接指向物理层或波特率不匹配, 不必等
        # 看门狗超时后再猜。
        _, status_txt = self.can.get_status()
        self.get_logger().info('CAN 控制器状态: %s' % status_txt)
        if self.can.is_bus_off():
            ok, rmsg = self.can.reset()
            self.get_logger().error(
                '启动即 BUSOFF —— 总线上没有正常通信(常见: 波特率不匹配 / 终端电阻 / '
                '某节点拉死总线)。已尝试重置: %s' % rmsg)
            del ok

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
        # 测指令帧周期(滑动平均), 供 _feedforward_speed 定"用几个周期走完剩余距离".
        # 不用 msg.velocity: 速度已改为按剩余距离算(自校正), 不再需要指令速度本身 ——
        # 用指令速度乘系数那条路会走停或无界落后, 见 lag_frames 注释.
        now = time.time()
        if self._prev_cmd_time > 0:
            dt = now - self._prev_cmd_time
            if 0.002 <= dt <= 0.2:      # 排掉同刻双帧与点动间歇后的第一帧
                with self._lock:
                    self._cmd_dt = dt if self._cmd_dt <= 0 else self._cmd_dt * 0.9 + dt * 0.1
        self._prev_cmd_time = now

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
            # 只发**真的在动**的那几个电机; 已到位的整轮跳过, 既不重启它们的梯形规划器
            # (会触发堵转保护锁死), 也不占用总线(查询帧得以恢复->反馈不饿死).
            moving = self._changed_motors(target) if target is not None else []
            if moving and self.can.is_open:
                # 帧构造与总线纪律完全不变: 发位置期间 _query_paused 挡住查询帧插入双帧中间(防 00 EE),
                # 电机间隔 1ms 防 PCAN 队列瞬时溢出.
                self._query_paused = True
                try:
                    if self._last_sent is None:
                        self._last_sent = list(target)
                    for i in moving:
                        if self._send_position_command(i + 1, target[i]):
                            # 逐个记账: 发失败的那个不更新 _last_sent, 下一轮自然重试。
                            # 原先整批赋值, 发失败也当已发, 目标就永久丢了。
                            self._last_sent[i] = target[i]
                        else:
                            self.get_logger().error(
                                '电机%d 位置帧发送失败 (目标 %.3f°)' % (i + 1, target[i]),
                                throttle_duration_sec=1.0)
                        time.sleep(0.001)
                finally:
                    self._query_paused = False
            self._check_feedback_alive()
            time.sleep(period)

    def _changed_motors(self, target) -> list:
        """返回需要重发的电机下标(**逐个判**, 不是任一变则全发)。首次(未发过)全发。

        ⚠️ 这里原先用 `any(...)` 整体判: 一个轴动就六个轴全重发 —— 与上面 send_deadband_deg
        的注释("变化<此值的**电机**不重发")自相矛盾, 且后果严重: 点动/轨迹执行期间目标一直
        在变, 于是六个电机每 10ms 全被重启一次梯形规划器, 而注释自己写着这会触发堵转保护
        锁死。2026-08-01 六个驱动一起停止应答就是这么来的(驱动一直通着电)。
        改成逐电机后, 只有真在动的轴承压, 静止轴彻底不被打扰。
        """
        if self._last_sent is None:
            return list(range(MOTOR_COUNT))
        return [i for i in range(MOTOR_COUNT)
                if abs(target[i] - self._last_sent[i]) > self._send_deadband_deg]

    def _check_feedback_alive(self):
        """反馈看门狗: 长时间收不到 CAN 反馈就持续报 ERROR。

        必须有, 因为反馈断了整条链会自欺(见 feedback_timeout_sec 参数注释):
        TopicBasedSystem 回显命令值 -> /joint_states 照常 100Hz -> JTC 8ms 报到位 ->
        MoveIt 报成功, 而臂根本没动。**这是唯一会出声的地方。**
        """
        if not self.can.is_open:
            return
        now = time.time()
        # 还没发过任何指令时电机可能本就不该有动作, 不报(避免启动瞬间刷屏)
        if self._last_sent is None:
            return
        with self._lock:
            last = list(self._last_fb_time)
        silent = [i + 1 for i in range(MOTOR_COUNT)
                  if now - last[i] > self._fb_timeout]
        if silent:
            self._fb_alarm = True
            # 同时问控制器自己的状态位: Read() 在总线彻底静默时只报"队列空", 分不出
            # "暂时没数据"与"控制器已 bus-off"。错误类型直接指向排查方向 ——
            # BUSOFF/BUSHEAVY = 物理层或波特率; 状态 OK 而无应答 = 驱动器侧没回。
            _, status_txt = self.can.get_status()
            self.get_logger().error(
                'CAN 反馈超时 %.1fs: 电机 %s 无应答 | 控制器状态: %s。'
                '⚠️ 此时 /joint_states 仍会照常发布(TopicBasedSystem 回显命令值), '
                'MoveIt 会假报动作成功 —— 别信它。'
                % (self._fb_timeout, silent, status_txt), throttle_duration_sec=2.0)
            self._recover_bus_off()
        elif self._fb_alarm:
            self._fb_alarm = False
            self.get_logger().warn('CAN 反馈已恢复, 六个电机均在应答')

    def _recover_bus_off(self):
        """bus-off 时重置控制器。**只做 bus-off, 不碰电机使能。**

        bus-off 是 CAN 控制器错误计数超限后的自我隔离, 不 reset 则永远不再收发,
        表现为 read 冻住而 write 照涨 —— 断电重上能好正是因为那样才重新初始化了控制器。
        reset 是恢复通信的必要条件, 但**不足以让电机动**: 驱动器若因堵转进了保护态,
        还需人工确认现场后调 ~/reenable(刻意不自动)。
        """
        if not self.can.is_bus_off():
            return
        ok, msg = self.can.reset()
        self.get_logger().error(
            'CAN 处于 BUSOFF, 已尝试重置控制器: %s。'
            '⚠️ 重置只恢复通信, 若电机因保护态不动仍需手动调 ~/reenable' % msg,
            throttle_duration_sec=5.0)
        if ok:
            self._last_sent = None   # 保护期间的目标已丢, 让下一轮重发一遍

    # ---------- 位置模式: 0xFD 梯形 / 0xFB 直通限速 ----------
    # 0xFD 每帧语义="按给定加减速与最大速度, **加速→减速→停在**目标位置"。
    # 2026-08-02 关于顿挫查到这里:
    #   ① 速度字段原先恒发巡航值(J1 恒 382RPM): 不论增量多小都命令"冲过去", 走完空等下一帧。
    #      已由 _feedforward_speed() 按 JTC 给的 velocity 逐帧算, 替代恒定值。
    #   ② 加速度**不是**可调的出路 —— 从 500 提到 20000 当场剧烈抖动 + 异响。因为每帧都重发
    #      一次梯形规划, 加速度方向每 10ms 反向一次, 提高它等于把 100Hz 强迫激励加重, 正落
    #      在步进共振带。500 之所以安静只是它连加速段都跑不完、从没那次减速反向。
    #   ⇒ 真正要去掉的是**"每帧减速到停"**这个语义本身, 即换 0xFB 直通限速(见下), 而不是
    #      在 0xFD 里调参数。这条别再试第二遍。
    # ⚠️ 别再回去查 CAN 帧格式: 0x37 位置误差实测 ±0.1~0.4°(电机轴)、指令与实测行程差
    #    0.02%, 电机忠实跟随, 换帧格式救不了这两个字段填错。
    # 0xFB 直通限速: 同样是位置目标(可照用绝对标志), 但**没有梯形加减速**, 直奔目标。
    # 帧短 4 字节(去掉两组加速度)。留作备用: 若 ① 拉高加速度后仍有异响/丢步, 它是另一条路。
    # ⚠️ 没选 0xF6 速度模式: 它只给速度不给位置, 电机会一直转到下一条指令为止。而 servo
    # 的停止方式是**停发**(见 joy_arm_teleop 里 zero-twist 那段), 停发就等于电机保持
    # 最后速度一直转出约束盒 —— 那是把安全模型整个反过来, 要另做失效保护才敢用。
    def _send_position_command(self, motor_id: int, position_deg: float) -> bool:
        if not self.can.is_open or not (1 <= motor_id <= MOTOR_COUNT):
            return False
        can_id = 0x100 + (motor_id - 1) * 0x100
        idx = motor_id - 1

        motor_direction = self.config.DIRECTION_MAP[idx]
        motor_speed = self._feedforward_speed(idx, position_deg)
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

    def _feedforward_speed(self, idx: int, target_deg: float) -> int:
        """速度字段(0.1RPM/LSB, 电机轴): 按"用 lag_frames 个帧周期走完**剩余距离**"算。

        为什么不是"指令速度 × 余量":
          余量>1 ⇒ 提前到达目标, 剩下的时间电机停着等下一帧 = 走停顿挫;
          余量<1 ⇒ 每帧都少走一点, 开环前馈没有回正机制, 会无界地越落越远(即"衰减")。
        按剩余距离算则自校正: 落后多了距离变大 ⇒ 自动加速; 跑快了距离变小 ⇒ 自动减速。
        稳态落后固定在 lag_frames × 每帧增量(J1 约 0.42°), 不累积, 且此时实发速度恰好
        等于指令速度(推导见 lag_frames 参数注释)。电机永远差一点没走到, 所以不会停。

        ⚠️ 必须用**实测反馈**算剩余距离, 不能用上一帧指令 —— 用指令就退化成开环, 自校正
           那一半没了。反馈的量化噪声不影响: 它只抬高速度上限, 不改变位置目标。
        """
        ratio = self.config.REDUCTION_RATIOS[idx] or 1.0
        cap = self.config.SPEEDS[idx] * 10           # 上限, 0.1RPM
        with self._lock:
            cur = self._motor_deg[idx]
        remain_deg = abs(target_deg - cur)           # 输出轴
        horizon = self._lag_frames * self._cmd_period()
        dps = remain_deg / horizon if horizon > 0 else 0.0
        # 输出轴 deg/s -> 电机轴 0.1RPM: rpm = dps/360*60*ratio = dps*ratio/6
        val = int(dps * ratio / 6.0 * 10)
        return max(int(cap * self._min_speed_frac), min(val, cap))

    def _cmd_period(self) -> float:
        """指令帧周期(秒), 实测滑动平均。不写死 0.01 是因为它由上游决定:
        MoveIt 规划执行时是 JTC 的 100Hz, servo 点动时取决于 publish_period(现 34ms),
        写死会让 lag_frames 的物理含义在两种场景下不一致。"""
        with self._lock:
            return self._cmd_dt if self._cmd_dt > 0 else 0.01

    def _can_send(self, can_id, data, is_extended=True):
        """所有 CAN 发送的唯一出口, 经 _can_tx_lock 串行化, 杜绝并发拼帧损坏."""
        with self._can_tx_lock:
            return self.can.send_message(can_id, data, is_extended)

    def _send_position_frame(self, can_id, direction, speed, accel, position) -> bool:
        try:
            pos_bytes = position.to_bytes(4, byteorder='big')
            speed_bytes = speed.to_bytes(2, byteorder='big')
            # 帧尾(官方ZDT_X57_V2): 相对绝对标志(0x01绝对) + 多机同步标志(0x00立即执行) + 0x6B
            abs_flag = 0x01
            sync_flag = 0x00
            if self._position_mode == 'fb':
                # 0xFB: FB + 符号 + 速度(2) + 位置(4) + abs + sync + 6B = 11 字节
                data_bytes = ([0xFB, direction] + list(speed_bytes) + list(pos_bytes)
                              + [abs_flag, sync_flag, 0x6B])
            else:
                accel_bytes = int(accel).to_bytes(2, byteorder='big')
                # 0xFD: FD + 符号 + 加加(2) + 减加(2) + 速度(2) + 位置(4) + abs + sync + 6B = 15 字节
                data_bytes = ([0xFD, direction] + list(accel_bytes) + list(accel_bytes)
                              + list(speed_bytes) + list(pos_bytes) + [abs_flag, sync_flag, 0x6B])
            func_code = data_bytes[0]
            # 分包规则(协议 7.2): >8 字节要拆包, 帧 ID = (地址<<8) + 包号,
            # 且**每包首字节都是功能码**。故第 1 包要重新带一次功能码。
            first = data_bytes[:8]
            second = data_bytes[8:]
            # 双帧必须在同一锁内连发, 防止中间被查询帧插入打断
            with self._can_tx_lock:
                ok1, _ = self.can.send_message(can_id, first, True)
                if not ok1:
                    return False
                if second:
                    ok2, _ = self.can.send_message(can_id + 1, [func_code] + second, True)
                    if not ok2:
                        return False
            return True
        except Exception as e:
            self.get_logger().error('构造/发送位置帧异常: %r' % e, throttle_duration_sec=2.0)
            return False

    # ---------- 查询循环: 0x36 请求反馈 ----------
    def _start_query_loop(self):
        self._query_running = True
        self._query_thread = Thread(target=self._query_loop, daemon=True)
        self._query_thread.start()

    def _query_loop(self):
        period = 1.0 / self.query_rate if self.query_rate > 0 else 0.01
        data = [0x36, 0x6b]
        err_data = [0x37, 0x6b]
        while self._query_running:
            if self.can.is_open and not self._query_paused:
                for mid in range(1, MOTOR_COUNT + 1):
                    can_id = 0x100 + (mid - 1) * 0x100
                    self._can_send(can_id, data, True)
                    time.sleep(0.001)
                    # 位置误差紧跟位置查, 同一轮里问同一个电机 —— 两个量时间上对齐才好对照
                    if self._query_pos_error:
                        self._can_send(can_id, err_data, True)
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
                    # receive_message 把"队列空"(正常, 每秒几百次)与真错误都返回 False,
                    # 只能靠文案区分。队列空静默跳过, 其余(总线错误/句柄失效)必须出声 ——
                    # 原先一律静默, 反馈死了五小时日志里一个字都没有。
                    if isinstance(msg, str) and '队列为空' not in msg:
                        self.get_logger().error('CAN 接收错误: %s' % msg,
                                                throttle_duration_sec=2.0)
                    time.sleep(0.002)
            except Exception as e:
                self.get_logger().error('接收循环异常: %r' % e, throttle_duration_sec=2.0)
                time.sleep(0.02)

    def _process_message(self, msg):
        try:
            if not isinstance(msg, dict):
                return
            can_id = msg.get('id')
            data = msg.get('data')
            if isinstance(data, (bytes, bytearray)):
                data = list(data)
            if not (0x100 <= can_id <= 0x600 and len(data) >= 7 and data[-1] == 0x6b):
                return
            idx = (can_id >> 8) - 1
            if not (0 <= idx < MOTOR_COUNT):
                return
            # 0x37 位置误差(诊断用): 符号 + 4 字节, ×100 放大.
            # ⚠️ 刻意**不喂看门狗** —— 看门狗只该认位置帧(0x36), 否则误差帧照常来
            # 就能掩盖位置反馈断流, 而那正是 TopicBasedSystem 回显陷阱的触发条件.
            if data[0] == 0x37:
                if self._err_pub is not None:
                    e = int.from_bytes(data[2:6], byteorder='big', signed=False) / 100.0
                    self._pos_err_deg[idx] = -e if data[1] == 0x01 else e
                    m = Float32MultiArray()
                    m.data = [float(v) for v in self._pos_err_deg]
                    self._err_pub.publish(m)
                return
            if data[0] != 0x36:
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
                self._last_fb_time[idx] = time.time()   # 喂看门狗
            self._publish_state()
        except Exception as e:
            self.get_logger().error('解析反馈帧失败: %r' % e, throttle_duration_sec=2.0)

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
