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
      ↑                   │                    │
      └────── START(9) 停机: 经 ready 回零位 ───┘

START 是**总开关**: HOME 按它启动进 DRIVE; DRIVE/ARM 按它停机 —— 车停 + 臂经 ready
回零位。**零位必须真回到**: 臂是增量编码器无 homing, 上电即认当前位置为零, 停在别处
再上电则零位基准错, 之后所有 base_link 系标定值跟着偏。
故 HOME 态下 START 的去向由 `arm_homed` 决定: 真=启动进 DRIVE, 假(上次收臂中途失败,
臂停在半途)=继续重试收臂。没有这个区分则失败后再按 START 会去启动底盘, 车拖着伸出的臂走,
且零位永远回不去。

    DRIVE: /cmd_vel_joy 原样转发到 /cmd_vel。臂停在 ready (收身, 不拖着伸出的臂走)。
    ARM:   掐掉转发并补发一帧零速 (不补的话底盘保持最后一个速度, 要等固件
           CMD_TIMEOUT_MS 500ms 才失效保护刹停), 摇杆改喷 moveit_servo。
    ARM->DRIVE 切换时先 /grasp/ready 把臂收回来才放行底盘, 顺序不能反。

R1(btn 5) 是两个状态通用的死人开关: DRIVE 下它是 teleop_twist_joy 的 enable_button
(那边 yaml 配的), ARM 下它是点动使能 —— 同一个键在互斥状态里各司其职, 不冲突。
⚠️ ARM 态的死人开关**还包含"joy 帧未过期"这一半**(`joy_stale_sec`): tick() 读的是缓存帧,
joy_node 崩掉/手柄被拔时缓存会永久停在最后一帧, 不检查过期则臂照着冻结的摇杆位置一直走。
DRIVE 态不需要(joy 死了 teleop_twist_joy 自己停发, 固件 500ms failsafe 兜住)。

臂模式下有**两套互不相干的运动路径**, 别混:
  ① 摇杆/十字键点动 -> /servo_node/delta_twist_cmds, 受下面的约束盒 (box_* 参数) 钳位。
  ② 按键动作 (✕抓取 ■放托盘 ○放地面 L1/L2卸盘各一个 △回正) -> 调 /grasp/* 服务, 走 grasp_node
     放托盘不选左右: 盘号由盒子类别定 (place.yaml category_tray, 类别1/2->左盘1, 3/4->右盘0).
     L1/L2 是卸货, 每按一次只卸栈顶一个 (/grasp/unload_one); 装货卸货完都回 look 待命。
     动作执行期间 busy=True, 点动与状态切换全部拦住, 必须等这一轮跑完。
     的 MoveIt plan()+execute() 带碰撞检查, **约束盒不参与**。放托盘/卸货的可达性与避障
     由规划器负责, 与本节点无关。

⚠️ 安全: 三层防护各管一段, 别互相当替补 ——
  - **servo 自碰撞检查**: servo.yaml `check_collisions: true` (10Hz)。托盘围栏在
    Link_11.stl 里且该 mesh 同时作 collision geometry, 故转腕把相机往托盘上怼**能拦**。
  - **约束盒**: 管**planning scene 里没有的东西** —— 场景只有机器人本体 + placed_* 已落盒,
    货架/外部围栏/墙**一个都不在**, servo 与规划器都看不见, 只有盒能拦。故盒的四角/盒顶
    必须手动 jog 实测, 标到"盒内任意末端姿态都不碰外部环境"为止 (物理保证, 比在代码里
    加角度限位可靠 —— 限位值本身也是猜的)。
  - **touch_links 豁免**: 吸着盒子时盒↔托盘碰撞**全程豁免**, 低于围栏顶横穿会蹭侧壁而
    规划器不报、执行不停 (详见 grasp_node.cpp 里 transit_z 那段注释)。**两条路径都有这个
    缺口**, 不是遥控独有。
约束盒只钳 TCP 的 x/y/z, 不钳姿态: 十字键转 roll/pitch 靠上面第一层兜, △ 键可随时回正。
盒的六个值已于 2026-08-01 真机实标 + IK 扫描复核 (见 box_* 参数处注释), 但标定环境是
**空台架无货架围栏**, 故它保证的是"臂自身可达", 不是"不碰环境" —— 真上货架后要重标。

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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
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
        # joy 回调**必须**单独一个互斥组: /joy 是 20Hz autorepeat 续帧, 留在 Reentrant 组里
        # 则多帧在 MultiThreadedExecutor 下真并发, `if self.busy: return` 与 `busy=True`
        # 之间隔着 _rising_edges/日志/_publish_zero 几步, 两帧都读到 busy==False 就双双放行
        # ⇒ 起两个 worker, 各自往 MoveIt 发 action。
        # 2026-08-01 实测症状: 按 START 后臂朝零位走到一半又被拽回 —— grasp_node 侧服务是
        # 互斥的, 两个 worker 的 ready→home 交错成 ready→home→ready→home。
        # 互斥组让 on_joy 串行, busy 的检查与占住、prev_buttons 的读改自然原子, 不必加锁。
        joy_cb = MutuallyExclusiveCallbackGroup()

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
        # 右摇杆实测只到 0.88/0.81 (左摇杆满 1.00), 故 z/yaw 实际最大速比设定低约 15%.
        #
        # 0.08 而非原来的 0.03: 3cm/s 走完盒 y 向(16cm)要 5 秒多, 太慢。
        # 上限由**钳位是反应式的**决定 —— _clamp_to_box 是越界后才把该方向置零, 没有提前
        # 减速, 所以越界深度 ≈ 速度 × 反应延迟。延迟链: tick 30Hz(33ms) + servo
        # publish_period(34ms) + CAN 落后 lag_frames 1.3 帧(44ms) ≈ 111ms, 另有 TF 滞后未计。
        # 0.08 对应越界约 9mm; 0.15 就要 17mm。
        # 2026-08-03 从 0.08 提到 0.35: 提速本身**减轻**了点动粗糙, 这是实测因果不是副作用。
        # 机理(用户提出, 已测量证实): can_bridge 只在位移超 send_deadband_deg(0.05°) 才发
        # CAN 帧。0.08 下从动轴每帧位移远小于死区 —— 实测 J14 帧位移 0.0043°(死区的 0.09 倍)
        # 要攒 97 帧≈1s 才发一次, J15 攒 176 帧 —— 即从动轴在做**秒级阶跃**而非连续跟随,
        # 六轴发帧节奏互不同步, 合成末端轨迹带横向抽动。提速把更多轴推过死区。
        # ⚠️ 别指望调 send_deadband_deg 解决: 降到 0.005 实测无改善且手感更差(从动轴变成
        #    100Hz 微幅斜坡重启, 激励频率进可感带), 两头都不舒服 —— 根子在 0xFD 每帧重启
        #    梯形斜坡的语义, 不在死区取值。
        # ⚠️⚠️ 安全代价, 装货架前必读: 约束盒是**软**钳位(超界才减速), 越界深度 ≈ 速度 ×
        #    反应延迟(tick 33ms + servo publish_period 40ms + CAN lag_frames 1.3 帧 44ms
        #    ≈ 117ms, 另有 TF 滞后未计)。0.08 越界约 9mm, **0.35 越界约 41mm**。
        #    而六个 box_ 值是**空台架无货架围栏**标的(见下方 box_ 注释), 只反映臂自身可达
        #    包络, 不含"不碰环境"。上货架/围栏后必须重标 box_, 或把此值降回 0.15(越界 18mm)。
        self.scale_lin = self.declare_parameter('servo_scale_linear', 0.35).value
        # 0.20 -> 0.5 -> 0.8 rad/s: yaw 只转腕不平移 TCP, 不受约束盒制约, 故可比 linear 激进.
        self.scale_rot = self.declare_parameter('servo_scale_angular', 0.8).value
        # roll/pitch 单独一档更小的: 腕一歪相机跟着歪, cam_target 标定立刻失效.
        # 提得比 yaw 保守, 且**约束盒钳不了腕姿态** —— 腕一歪就能怼地(见 box_z_min 注释).
        self.scale_wrist = self.declare_parameter('servo_scale_wrist', 0.5).value

        # ---- 约束盒 (base_link 系, TCP=suction_tip 不许出这个盒) ----
        # 2026-08-01 真机实标 (RViz 拖臂走四角读 base_link->suction_tip) + /compute_ik
        # 网格扫描复核 (吸盘朝下 yaw=-90°, 924 点 83% 有解).
        # ⚠️ 标定环境是**空台架, 无货架围栏** —— 这组值反映的是**臂自身可达包络**,
        #    不是"不碰环境". 真上货架/围栏后六个值全要重标.
        #
        # 底 0.12: 与 grasp_node 的 pregrasp_height 同值 (抓取时臂本来就下到这个高度).
        #   实测最低可走到 z≈0.017, 但没采纳 —— 那只离地 6cm(地面 z=-0.0476), 而约束盒
        #   钳不了腕姿态, 腕一歪就能怼地。
        self.box_z_min = self.declare_parameter('box_z_min', 0.12).value
        # 顶 0.29: look 位实测 TCP z=0.295 (点动的起点, 盒必须容纳它).
        self.box_z_max = self.declare_parameter('box_z_max', 0.29).value
        self.box_x_min = self.declare_parameter('box_x_min', -0.17).value  # 实测 -0.167
        # x_max 实测 0.146, 且扫描显示 x=+0.18 几乎全无解 —— 这是真的可达边缘, 不是保守值.
        self.box_x_max = self.declare_parameter('box_x_max', 0.14).value
        self.box_y_min = self.declare_parameter('box_y_min', -0.35).value  # 实测 -0.350
        # y_max **刻意比实测的 -0.267 放宽**: look 位 TCP y=-0.201 在那之外, 照实测填则切进
        # ARM 态时起点就在盒外, y 正向被永久钳住只能单向走回来。-0.19 给 look 起点留 10mm.
        # 扫描确认 y∈[-0.21,-0.35] 全 z 全 x 干净; 再往上(y=-0.19)低 z 段中间 x 有空洞,
        # 那是臂自身基座区, 归 servo 的 check_collisions 管 (第一层防护), 盒不重复兜.
        self.box_y_max = self.declare_parameter('box_y_max', -0.19).value

        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.ee_frame = self.declare_parameter('ee_frame', 'suction_tip').value
        # 25Hz = 40ms, 必须与 servo.yaml 的 publish_period 一致: low_latency_mode 下
        # servo 只在收到 twist 时出帧, 出帧节奏由这里决定, 只改 servo 那边不生效。
        # 40ms 是控制周期 10ms 的整数倍, 理由见 servo.yaml publish_period 上方注释。
        self.rate_hz = self.declare_parameter('publish_rate', 25.0).value
        # 上电是否自动摆 home: 真机第一次测时臂可能停在任意位置, 自动跑会突然大幅运动,
        # 故默认关. 桌面快捷方式那条命令里再打开 (那时臂位置已知).
        self.home_on_start = self.declare_parameter('home_on_start', False).value
        # 没起机械臂栈时只想遥控底盘: 置 true 则 /grasp/ready 不在线也放行 DRIVE。
        # ⚠️ 代价是"臂已收拢"这个前提没人验证过, 车可能拖着伸出的臂走 —— 只在臂确实
        # 没通电(或人眼确认已收拢)时用。臂栈在跑时保持 false, 让收臂失败拦住底盘。
        self.drive_without_arm = self.declare_parameter('drive_without_arm', False).value

        # /joy 帧过期阈值: 超过这么久没收到新帧就当手柄不在, 停发点动。
        # ⚠️ 这条是死人开关的**必要组成部分**, 不是优化 —— tick() 读的是缓存的 last_joy,
        # 若 joy_node 崩了/手柄 USB 被拔, 而当时 R1 正按着摇杆正推着, 缓存就永久停在那一帧,
        # tick() 会照着冻结的摇杆位置一直 30Hz 发点动, 臂走到撞上约束盒边界才停。
        # DRIVE 态不需要这个(joy 死了 teleop_twist_joy 自己停发, 固件 500ms failsafe 兜住),
        # **只有 ARM 态敞口**, 因为点动帧是本节点自己造的、永远新鲜。
        # 取 0.4s: yaml 配 autorepeat 20Hz(50ms), 但 2026-08-01 实测 /joy 只有 15~16Hz 且
        # **最大帧间隔 101ms** —— 阈值必须按实测的最坏间隔留余量, 不是按标称 50ms 算。
        # 0.4s ≈ 4 个最坏间隔; 再小则偶发丢帧会误判掉线, 点动一顿一顿。
        # 上限也别放太宽: 这是安全阈值, 手柄真掉了要在 0.4s 内停发, 期间臂最多多走
        # 0.4s × 0.03m/s ≈ 12mm, 在约束盒的容差内。
        self.joy_stale_sec = self.declare_parameter('joy_stale_sec', 0.4).value

        self.state = HOME
        self.last_joy = None
        self.last_joy_time = None
        self.prev_buttons = []
        self.lock = threading.Lock()
        self.busy = False           # 服务在跑, 期间不接点动也不切状态
        # 臂是否**确认**在零位。初值 True 是对的: 臂是增量编码器无 homing, 上电即认当前位置
        # 为零, 所以"刚上电"与"在零位"是同一件事。
        # 为什么需要这个标志: HOME 既是"臂在零位"也是各种失败后的兜底态, 两者不能混。
        # 停机时 ready 或 home 失败, 臂停在半途而 state 已是 HOME ⇒ 再按 START 会走
        # HOME->DRIVE(启动), 用户**永远没法用 START 重试收臂**。而零位必须真回到 ——
        # 停在别处断电则下次上电零位基准错, 之后所有 base_link 系标定值跟着偏。
        # 有了它, START 在 HOME 态就能分辨"该启动"还是"该重试停机"。
        self.arm_homed = True

        # 点动速度要在线扫: 上面三个 scale_* 原先只在 __init__ 读一次快照, 改一档就得重启
        # 遥控栈(会连带 joy_node/teleop_twist_joy 一起重起, 且残留进程要手动清)。
        # 2026-08-03 因此白测一轮: ros2 param set 报成功而实际没生效, 拿到的是旧速度的数据。
        self.add_on_set_parameters_callback(self._on_set_params)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel_out', 10)
        self.pub_servo = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)

        self.create_subscription(Joy, 'joy', self.on_joy, qos_profile_sensor_data,
                                 callback_group=joy_cb)
        self.create_subscription(Twist, 'cmd_vel_in', self.on_cmd_vel, 10,
                                 callback_group=cb)

        self.cli = {n: self.create_client(Trigger, n, callback_group=cb) for n in (
            '/grasp/home', '/grasp/ready', '/grasp/look', '/grasp/level',
            '/grasp/pick_only', '/grasp/place_only', '/grasp/place_ground',
            '/grasp/unload_one')}
        self.start_servo = self.create_client(Trigger, '/servo_node/start_servo',
                                              callback_group=cb)
        self.set_tray = self.create_client(SetParameters, '/grasp_node/set_parameters',
                                           callback_group=cb)

        # tick 与 on_joy 同组: 点动发帧与状态迁移从此串行, 不会交错 ——
        # 这也根治了原先靠"busy=True 必须写在 state=ARM 之前"缓解的那个瞬间窗口。
        # 两者都不阻塞(tick 只查 TF, on_joy 只起线程), 放一个互斥组不会互相饿死。
        self.create_timer(1.0 / self.rate_hz, self.tick, callback_group=joy_cb)
        self.get_logger().warn(
            '手柄三态遥控就绪 [HOME] — START(%d) 进 DRIVE, SELECT(%d) 切 DRIVE/ARM, '
            '死人开关 R1(%d)' % (self.btn_start, self.btn_select, self.btn_enable))
        if self.home_on_start:
            # after 给成空函数而不是默认的 _restart_servo: 这里是 HOME 态, servo 基准要等
            # 切进 ARM 时才锁(_to_arm 里做), 此刻重锁毫无意义。
            # 收拢期间 arm_homed 先置假 —— 臂上电位置未知才需要跑这一步, 只有**成功**才算
            # 确认在零位, 故走 on_success 而不是 after(after 失败也会执行)。
            self.arm_homed = False
            self._run_action('上电收拢', '/grasp/home',
                             after=lambda: None, on_success=self._mark_homed)

    def _on_set_params(self, params):
        """只放行点动速度三档。约束盒与按键号刻意不可在线改 —— 盒是安全边界,
        改错了没有第二层兜(planning scene 里没有货架/围栏, 只有盒能拦)。"""
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'servo_scale_linear':
                self.scale_lin = float(p.value)
            elif p.name == 'servo_scale_angular':
                self.scale_rot = float(p.value)
            elif p.name == 'servo_scale_wrist':
                self.scale_wrist = float(p.value)
            else:
                continue
            self.get_logger().warn('参数在线改: %s = %r' % (p.name, p.value))
        return SetParametersResult(successful=True)

    def _mark_homed(self):
        self.arm_homed = True

    # ---- 底盘: 只有 DRIVE 态转发 ----
    def on_cmd_vel(self, msg):
        if self.state == DRIVE and not self.busy:
            self.pub_cmd.publish(msg)

    def on_joy(self, msg):
        with self.lock:
            self.last_joy = msg
            # 用本地时钟而不是 msg.header.stamp: 跨机时 stamp 受 NTP 对时影响,
            # 而这里只关心"这一帧多久前到的我手上", 本地单调时间才是对的量。
            self.last_joy_time = self.get_clock().now()
        pressed = self._rising_edges(msg)
        if pressed:
            # 诊断: 每个上升沿都记, 含被 busy 丢弃的 —— 查"某键触发了意外动作"必须
            # 能区分"键没按到"与"键按到了但被 busy 吃掉"。
            self.get_logger().warn('[joy] 上升沿 %s state=%s busy=%s'
                                   % (pressed, self.state, self.busy))
        if not pressed or self.busy:
            return
        if self.btn_start in pressed:
            # START 是总开关: HOME 且臂确认在零位时启动进 DRIVE, 其余一律停机回零位.
            # arm_homed 为假说明上次停机没收成(臂停在半途), 此时 START 继续重试收臂 ——
            # 不能去启动底盘, 否则车拖着伸出的臂走, 且零位永远回不去。理由见 arm_homed 定义。
            if self.state == HOME and self.arm_homed:
                self._to_drive('START')
            else:
                self._to_shutdown()
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
        # busy 必须在 state=ARM **之前**占住: 反了则 tick() 有一瞬看到
        # state==ARM and busy==False, 按住 R1 就会让 servo 与 look 轨迹抢控制器.
        self.busy = True
        self.state = ARM
        self.get_logger().warn('[ARM] 底盘已停发, 臂去 look 位待命')
        # start_servo 必须在 look **到位之后**才调: ServoCalcs::start() 会把当时的关节位姿
        # 锁存进 last_sent_command_(servo_calcs.cpp:238), 之后任何"全零但新鲜"的指令都会把
        # 那份位姿原样重发。并发调用则锁存的是 look 之前的 ready 位 ⇒ 一点动就跳回 ready。
        # 重复调 start_servo 是安全的: 它内部先 stop() 再按当前实测位姿重新锁存。
        self._run_action('去 look', '/grasp/look')   # 跑完自动重锁 servo, 见 _run_action

    def _restart_servo(self):
        """(重新)启动 servo, 让它按**当前**实测位姿锁存基准。每次动作后都要重调 ——
        MoveIt 执行过轨迹之后 servo 那份锁存位姿必然已经过时。
        pause/unpause 不行: 它们不重新锁存(servo_node.cpp:135)。"""
        if self.start_servo.service_is_ready():
            self.start_servo.call_async(Trigger.Request())
        else:
            self.get_logger().warn('/servo_node/start_servo 不在线, 点动可能跳回旧位姿')

    def _to_drive(self, via):
        # 先把臂收回 ready 再放行底盘, 顺序不能反 —— 不然车拖着伸出的臂走.
        # 收臂期间挂在 HOME 这个过渡态: 车不放行, 臂也不接点动.
        prev, self.state = self.state, HOME
        # 一旦开始往 ready 走就不再"确认在零位"了。必须在这里清而不是在 worker 里 ——
        # 收臂失败时臂已离开零位停在半途, 那种情况下 START 必须走重试停机而非启动底盘。
        self.arm_homed = False
        self._publish_zero()
        self.get_logger().warn('[%s->DRIVE] 经 %s: 收臂到 ready 中, 底盘暂不放行...'
                               % (prev, via))
        self.busy = True   # 同步占住, 理由见 _run_action

        def worker():
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

    def _to_shutdown(self):
        """START 停机: 车停 + 臂经 ready 回零位。
        零位必须真回到 —— 臂是增量编码器无 homing, 上电即认当前位置为零, 停在别处
        再上电零位基准就错了, 之后所有 base_link 系标定值跟着偏。
        分两段(先 ready 再 home)而不是从 look 直接规划回零: 跨度小, 规划失败率低。
        """
        prev, self.state = self.state, HOME
        self.arm_homed = False   # 收成之前一律当"没在零位", 中途失败则 START 会继续重试
        self._publish_zero()
        self.get_logger().warn('[%s->HOME] 经 START 停机: 车已停, 臂回零位中...' % prev)
        self.busy = True   # 同步占住, 理由见 _run_action

        def worker():
            try:
                ok, m = self._call_sync('/grasp/ready')
                if not ok:
                    self.get_logger().error('停机中止 — 收 ready 失败: %s。'
                                            '⚠️ 臂未回零位, **再按一次 START 可重试**; '
                                            '断电前必须确认已回零' % m)
                    return
                ok, m = self._call_sync('/grasp/home')
                if ok:
                    self.arm_homed = True
                    self.get_logger().warn('[HOME] 臂已回零位, 车已停 — 可断电')
                else:
                    self.get_logger().error('回零位失败: %s。⚠️ **再按一次 START 可重试**。'
                                            '断电前必须手动回零, 否则下次上电零位基准是错的'
                                            % m)
            finally:
                self.busy = False
        threading.Thread(target=worker, daemon=True).start()

    def _publish_zero(self):
        self.pub_cmd.publish(Twist())

    def _log_result(self, text, ok):
        """按成功/失败选日志级别。**不能**写成 `log = warn if ok else error` 再调 ——
        rclpy 按"文件+行号"缓存 logger 配置, 同一行先后用两个级别会抛
        ValueError: Logger severity cannot be changed between calls, 而这个异常会直接
        掀掉 worker 线程: 2026-08-03 卸货成功后那句"回 look"因此从未执行过 (finally 里
        仍清了 busy, 所以看起来不像崩)。分成两个调用点, 两个级别各占自己的行号。"""
        if ok:
            self.get_logger().warn(text)
        else:
            self.get_logger().error(text)

    # ---- 动作: 阻塞期间不接点动, 跑完回到点动待命 ----
    def _run_action(self, name, srv, first=None, then=None, after=None, on_success=None):
        # after 默认 = 重锁 servo 基准. 本方法只在 ARM 态被调(点动态), 而每个动作都动了臂,
        # 所以"跑完必须重锁"是无例外的规则 —— 设成默认值而不是逐个调用点去传, 免得漏。
        # after 在 finally 里**无条件**跑(失败也得重锁基准); 只该在成功时做的事走 on_success。
        if after is None:
            after = self._restart_servo
        # busy 必须在起线程**之前**同步占住: 若留给 worker 去设, 从这里返回到 worker 被
        # 调度上有个窗口, 期间 tick() 看到 state==ARM and busy==False 就放行点动 ——
        # servo 与正在执行的 MoveIt 轨迹同时往 /arm_controller/joint_trajectory 发,
        # 后发覆盖先发。2026-08-01 实测: 切进 ARM 后按住 R1, look 轨迹被 30Hz 的 servo
        # 指令覆盖, 臂停在 ready 附近而 MoveIt 仍报"已到看货姿势"。
        self.busy = True

        def worker():
            try:
                if first:
                    self.get_logger().warn('%s: 先 %s' % (name, first))
                    ok, m = self._call_sync(first)
                    if not ok:
                        self.get_logger().error('%s 中止 — %s 失败: %s' % (name, first, m))
                        return
                ok, m = self._call_sync(srv)
                self._log_result('%s -> %s %s' % (name, ok, m), ok)
                if ok and then:
                    self.get_logger().warn('%s: 收尾 %s' % (name, then))
                    self._call_sync(then)
                if ok and on_success:
                    on_success()
            finally:
                # 先重锁 servo 基准再放行点动, 顺序不能反 —— 任何 MoveIt 轨迹跑完,
                # servo 里那份 last_sent_command_ 都已过时, 不重锁则第一帧点动会跳回旧位姿.
                if after:
                    after()
                self.busy = False
        threading.Thread(target=worker, daemon=True).start()

    def _run_unload(self, name, tray):
        """卸货: 先把 grasp_node 的 unload_tray 设成目标盘号, 再调服务, 完事回 look.

        调 /grasp/unload_one (每次只卸栈顶一个), 不是 /grasp/unload_tray (整盘).
        整盘那条留给 mm_task 状态机, 遥控要的是一按一个、人跟着取盒。
        """
        self.busy = True   # 同步占住, 理由见 _run_action

        def worker():
            try:
                if not self._set_tray_param(tray):
                    self.get_logger().error('%s 中止: 设 unload_tray=%d 失败' % (name, tray))
                    return
                ok, m = self._call_sync('/grasp/unload_one')
                self._log_result('%s (盘%d) -> %s %s' % (name, tray, ok, m), ok)
                if ok:
                    self._call_sync('/grasp/look')
            finally:
                self._restart_servo()   # 同 _run_action: 动过臂就得重锁基准
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

    # 30 而不是原来的 180: 超时**不是**"动作还没跑完"的正常等待, 而是 response 丢了。
    # 2026-08-02 实测: grasp_node 正常完成 look 并返回(日志"已到看货姿势"), teleop 侧却
    # 到 180s 才报超时 = 请求发出 + 整个 timeout, 那个 response 从头到尾没到客户端。
    # 同一份日志里 servo_node 有一模一样的签名 (failed to send response ...
    # client will not receive response), 两个互不相干的服务端都是"回调返回了、response
    # 投不出去" ⇒ rmw_fastrtps 传输层, 疑似 /dev/shm 孤儿段(见记忆里 matched=0 那条)。
    # 期间 busy 一直是 True, 所有按键被丢弃, 看起来像死锁 —— 其实超时一到 finally 就放行。
    # 取 30s: MoveIt 最长一段执行实测约 8s, 30s 够宽; 丢包时只白等 30s 而不是 3 分钟。
    def _call_sync(self, srv, timeout=30.0):
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
            jt = self.last_joy_time
        if joy is None:
            return
        # joy 帧过期 = 手柄/joy_node 没了 -> 停发(等同松开死人开关)。理由见 joy_stale_sec。
        if jt is None or (self.get_clock().now() - jt) > Duration(seconds=self.joy_stale_sec):
            self.get_logger().error(
                '/joy 超过 %.1fs 无新帧 — 手柄或 joy_node 掉了, 点动已停发'
                % self.joy_stale_sec, throttle_duration_sec=2.0)
            return
        if not self._btn(joy, self.btn_enable):
            # 死人开关松开: **停发**, 绝不发全零帧。
            # ⚠️ moveit_servo 里"全零但新鲜"与"过期"是两条完全不同的路径:
            #   - 停发 -> incoming_command_timeout(0.1s) 到 -> filteredHalt(), 它用**实测**
            #     关节位姿刹停, 安全。
            #   - 发全零 -> 命令永不过期 -> servo_calcs.cpp:413 走
            #     `*joint_trajectory = *last_sent_command_`, 把 **start_servo 那一刻锁存的
            #     旧位姿**原样重发, 且 stamp=0 表示立即执行 ⇒ 臂会"快速跳回"那个旧位姿。
            # 2026-08-01 实测: start_servo 与 /grasp/look 并发调用, 锁存的是 look 之前的
            # ready 位; 到 look 后单按 R1(摇杆居中)臂即快速回 ready。推摇杆则走正常雅可比
            # 分支(每周期从实测状态重新起算)故能"打断并点动" —— 两个症状同一个根因。
            return

        t = TwistStamped()
        # x/y 与下面的 roll/pitch 取负: 2026-08-01 首次真机点动实测这四个方向与摇杆/十字键
        # 推向相反, 而 z/yaw 正确 —— 不是整体系差, 逐轴实测定的. 手柄轴正负与 base_link
        # 轴正负本是两套无关约定, 推不出来, 只能以实测为准.
        t.twist.linear.x = -self._ax(joy, self.axis_x) * self.scale_lin
        t.twist.linear.y = -self._ax(joy, self.axis_y) * self.scale_lin
        t.twist.linear.z = self._ax(joy, self.axis_z) * self.scale_lin
        t.twist.angular.z = self._ax(joy, self.axis_yaw) * self.scale_rot
        # 十字键是轴 (实测), 转腕. ⚠️ 约束盒管不了姿态, 这两个方向无几何防护.
        t.twist.angular.y = -self._ax(joy, self.axis_pitch) * self.scale_wrist
        t.twist.angular.x = -self._ax(joy, self.axis_roll) * self.scale_wrist
        self._clamp_to_box(t)
        # 钳位后也可能变成全零(贴着边界往外推). 全零一律不发, 理由同上 —— 按住死人开关
        # 但摇杆居中同样会触发 servo 重发陈旧位姿.
        if self._is_zero(t):
            return
        self.pub_servo.publish(self._stamp(t))

    @staticmethod
    def _is_zero(t, eps=1e-9):
        v, w = t.twist.linear, t.twist.angular
        return all(abs(c) < eps for c in (v.x, v.y, v.z, w.x, w.y, w.z))

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
