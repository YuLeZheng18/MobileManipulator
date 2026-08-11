#!/usr/bin/env python3
"""
mm_task / mission_manager — 顶层任务状态机 S0–S5 (架构 §7.2)

只做"编排", 不重算几何: 导航交 lane_navigator, 抓取/卸货整段交 grasp_node 的
/grasp/execute /grasp/unload_tray (三段抓取 + 末段相对直插纪律都在 grasp_node 内).

状态流 (worker 线程 run_mission):
  S0 INIT    发 /initialpose 给 AMCL 初值 -> 等收敛 -> /grasp/reset_stack -> /grasp/ready
  S1 NAV     发 /go_to=<nav_target>, 等 /lane_navigator/status "<target>:SUCCEEDED"
  S2 ALIGN   ArUco 精对位 (本轮 no-op, 直接放行)
  S3 DETECT  仅 action==grasp 时: 等 /perception/object_pose 新鲜且够得着
  S4 GRASP/UNLOAD  action 分派:
               grasp  -> S3a /grasp/look 读可抓数 -> 循环 detect+/grasp/execute 连抓
                         (抓空或托盘满即收工) -> /grasp/ready 收身
               unload -> 设参数 unload_tray=<tray> -> /grasp/unload_tray 整盘卸到地面
               none   -> 仅导航
  S5 LOOP    取任务列表下一项回 S1; 跑完→DONE

执行结构: MultiThreadedExecutor 主线程 spin; 订阅/服务客户端在 ReentrantCallbackGroup;
主流程跑在 worker 线程, 服务用 call_async + 轮询 future.done() (响应由主线程 executor 处理).
"""
import threading

import yaml

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener


def yaw_to_quat(yaw):
    import math
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        self.declare_parameter('mission_file', '')
        mission_path = self.get_parameter('mission_file').get_parameter_value().string_value
        if not mission_path:
            from ament_index_python.packages import get_package_share_directory
            mission_path = get_package_share_directory('mm_task') + '/config/mission.yaml'
        self.get_logger().info(f'Loading mission: {mission_path}')
        self.load_mission(mission_path)

        cbg = ReentrantCallbackGroup()

        # latched: 晚起的 AMCL / 本节点晚订阅也能拿到最后一条
        latched = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.initpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', latched)
        self.goto_pub = self.create_publisher(String, '/go_to', 10)

        # 最近一条终态, 存成 (seq, "<target>:SUCCEEDED|FAILED"); seq 为 None = 对方无序号旧版
        self._nav_status = None
        self._seq_floor = 0          # stage_nav 下发目标前的 seq 门限, 用来滤掉上一轮旧终态
        self.create_subscription(
            String, '/lane_navigator/status', self.on_nav_status, 10,
            callback_group=cbg)

        self._last_trigger_msg = ''  # 最近一次 call_trigger 的 resp.message
        self._last_obj = None        # (stamp_sec, PoseStamped)
        self.create_subscription(
            PoseStamped, '/perception/object_pose', self.on_object, 10,
            callback_group=cbg)

        self.grasp_cli = self.create_client(Trigger, '/grasp/execute', callback_group=cbg)
        # 卸货走 /grasp/unload_tray 而不是老的 /grasp/unload: 后者靠视觉从托盘上重新识别盒,
        # 而托盘上的盒高于感知端 pick_z_max 会被整批滤掉, 真机根本拿不到目标.
        # unload_tray 不用视觉 —— 取盒目标就是当初放它时的吸盘位姿(grasp_node 的 placed_release_),
        # 取放对称, 一次调用把该托盘上的盒全卸到地面两个卸货点.
        self.unload_cli = self.create_client(Trigger, '/grasp/unload_tray', callback_group=cbg)
        # 卸哪个托盘: Trigger 带不了数值, 由参数传 (grasp_node 用普通 declare_parameter, 可热设)
        self.set_param_cli = self.create_client(
            SetParameters, '/grasp_node/set_parameters', callback_group=cbg)
        # S0 开工前清一次堆叠状态 + 残留碰撞体: 上一轮跑完 grasp_node 里还记着盒在托盘上,
        # 不清则这一轮 TRAY_FULL 立刻触发, 且场景里的 placed_* 会挡住直下段.
        self.reset_cli = self.create_client(Trigger, '/grasp/reset_stack', callback_group=cbg)
        # S0 底盘行进前摆臂 ready; grasp 任务识别前摆看货姿势 (ready+J1+90°, 供视觉看见)
        self.ready_cli = self.create_client(Trigger, '/grasp/ready', callback_group=cbg)
        self.look_cli = self.create_client(Trigger, '/grasp/look', callback_group=cbg)

        # 等 AMCL 收敛用: S0 发完 initialpose 后阻塞等 map->base_link 出现再进 S1
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            f'mission_manager 就绪: {len(self.tasks)} 个任务, '
            f'initial_pose=({self.init_x:.2f},{self.init_y:.2f},{self.init_yaw:.2f})')

        self._worker = threading.Thread(target=self.run_mission, daemon=True)
        self._worker.start()

    # ---- 配置 ----
    def load_mission(self, path):
        with open(path, 'r') as f:
            cfg = yaml.safe_load(f)
        ip = cfg.get('initial_pose', {})
        self.init_x = float(ip.get('x', 0.0))
        self.init_y = float(ip.get('y', 0.0))
        self.init_yaw = float(ip.get('yaw', 0.0))
        to = cfg.get('timeouts', {})
        self.t_nav = float(to.get('nav', 120.0))
        self.t_detect = float(to.get('detect', 10.0))
        self.t_grasp = float(to.get('grasp', 150.0))
        self.t_localize = float(to.get('localize', 20.0))
        # 单货架连抓上限: 只是防死循环的兜底, 正常终止靠 pickable=0 或 TRAY_FULL.
        # 默认 4 = 一个货架最多放 4 个盒 (托盘总容量也是 4).
        self.max_picks_per_shelf = int(cfg.get('max_picks_per_shelf', 4))
        self.tasks = cfg.get('tasks', [])

    # ---- 订阅回调 ----
    def on_nav_status(self, msg):
        """lane_navigator 终态回报: "<seq> <target>:SUCCEEDED|FAILED".

        seq 单调递增, 只用来把"本轮新终态"与"上一轮的旧终态"区分开 —— 故这里只存,
        由 stage_nav 比对。兼容无 seq 的旧格式(seq 记 None)。"""
        raw = msg.data.strip()
        seq, _, rest = raw.partition(' ')
        if seq.isdigit() and rest:
            self._nav_status = (int(seq), rest.strip())
        else:
            self._nav_status = (None, raw)

    def on_object(self, msg):
        # 存"收到时的 monotonic 墙钟时刻", 与 stage_detect 的 self.now() 同基准.
        # 不能用 msg.header.stamp: 那是 sim 时间, 和 monotonic 混算 age 会得到垃圾值.
        self._last_obj = (self.now(), msg)

    # ---- 主流程 ----
    def run_mission(self):
        if not self.stage_init():
            self.get_logger().error('S0 初始化定位失败, 任务中止')
            return
        for i, task in enumerate(self.tasks):
            target = task.get('nav_target')
            action = task.get('action', 'none')
            tray = int(task.get('tray', 0))
            self.get_logger().info(
                f'==== 任务 {i + 1}/{len(self.tasks)}: nav={target} action={action}'
                f'{f" tray={tray}" if action == "unload" else ""} ====')
            if not self.run_task(target, action, tray):
                self.get_logger().error(f'任务 {i + 1} 失败, 任务序列中止')
                return
        self.get_logger().info('==== 全部任务完成 DONE ====')

    def run_task(self, target, action, tray):
        if not self.stage_nav(target):
            return False
        self.stage_align(target)
        if action == 'grasp':
            return self.stage_grasp_shelf()
        if action == 'unload':
            return self.stage_unload_tray(tray)
        if action == 'none':
            self.get_logger().info('action=none: 仅导航, 跳过抓取')
            return True
        self.get_logger().error(f'未知 action="{action}"')
        return False

    # ---- S4 卸货: 把一个托盘整盘卸到地面 ----
    # 空盘不算失败: 一条 mission 里两个盘各一条卸货任务, 而这一轮装了几个盘取决于货架上
    # 有几个盒 —— 只装了右盘时左盘那条任务照样会跑到, 它该跳过而不是把整条序列判失败.
    def stage_unload_tray(self, tray):
        self.get_logger().info(f'==== S4 卸 {tray} 号托盘 ====')
        if not self.set_unload_tray(tray):
            return False
        ok, msg = self.stage_grasp_msg(self.unload_cli, '/grasp/unload_tray')
        if not ok and '空的' in msg:
            self.get_logger().info(f'{tray} 号托盘本轮没装货, 跳过卸货')
            return True
        return ok

    def set_unload_tray(self, tray):
        if not self.set_param_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/grasp_node/set_parameters 不可用')
            return False
        req = SetParameters.Request()
        req.parameters = [Parameter(
            name='unload_tray',
            value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=int(tray)))]
        future = self.set_param_cli.call_async(req)
        deadline = self.now() + 10.0
        while rclpy.ok() and self.now() < deadline:
            if future.done():
                results = future.result().results
                if results and results[0].successful:
                    return True
                reason = results[0].reason if results else 'no result'
                self.get_logger().error(f'设 unload_tray={tray} 失败: {reason}')
                return False
            self.sleep(0.1)
        self.get_logger().error(f'设 unload_tray={tray} 超时')
        return False

    # ---- S3/S4 同一货架连抓 (2026-07-31 加) ----
    # 一个货架上有 1~4 个盒, 具体几个只有识别了才知道, 故不能在 mission.yaml 里按盒数写死
    # 几条 grasp 任务. 循环 detect->execute 直到两个终止条件之一:
    #   ① pickable 归 0 —— 这个货架抓空了;
    #   ② 托盘装满 (execute 返回 TRAY_FULL) —— 该去卸货了, 不是故障.
    # 两者都算本任务成功, 后续的卸货任务照常跑; 只有真故障(规划/执行失败)才中止整条序列.
    #
    # 循环体只有 detect+execute, 不重跑 S1 导航与 S3a look: 车没动, 且 grasp_node 的
    # /grasp/execute 收尾已经把臂停在 look 位 (2026-07-31 改), 下一轮直接就能识别.
    # 离开货架前才显式调 /grasp/ready —— 臂收回身前底盘才好走, 而这是"要不要走"这件事,
    # 只有状态机知道.
    def stage_grasp_shelf(self):
        if not self.stage_look():
            return False
        n = self._pickable_from(self._last_trigger_msg)
        if n == 0:
            self.get_logger().warn('S3a look 报可抓 0 个: 这个货架没有可抓的盒, 跳过抓取')
            return self.leave_shelf()
        self.get_logger().info(f'==== 本货架识别到 {n} 个可抓盒, 开始连抓 ====')

        picked = 0
        for _ in range(self.max_picks_per_shelf):
            if not self.stage_detect():
                # 识别不到新鲜可达帧: 上一轮已抓空是最常见的原因 (execute 报的 pickable 已归 0
                # 时压根不会走到这里, 但感知侧偶发丢帧也会落到这条). 抓过至少一个就算这个货架
                # 做完了, 一个都没抓到才是真失败.
                if picked > 0:
                    self.get_logger().info(f'S3 没有更多可抓盒, 本货架收工 (已抓 {picked} 个)')
                    break
                return False
            ok, msg = self.stage_grasp_msg(self.grasp_cli, '/grasp/execute')
            if not ok:
                if 'TRAY_FULL' in msg:
                    self.get_logger().warn(
                        f'S4 托盘已满, 停止本货架抓取 (已抓 {picked} 个), 去卸货: {msg}')
                    break
                return False
            picked += 1
            left = self._pickable_from(msg)
            free_slots = self._tray_free_from(msg)
            self.get_logger().info(
                f'S4 本货架已抓 {picked} 个, 还剩 {left} 个可抓, 托盘余位 {free_slots}')
            if left == 0:
                self.get_logger().info(f'本货架抓空 (共 {picked} 个)')
                break
            if free_slots == 0:
                self.get_logger().warn(f'托盘已无余位 (已抓 {picked} 个), 去卸货')
                break
        else:
            self.get_logger().warn(
                f'达到单货架抓取上限 {self.max_picks_per_shelf}, 停止本货架')
        return self.leave_shelf()

    # 离开货架前把臂摆回 ready: 底盘不拖着伸出的臂走. grasp_node 的 execute 收尾停在 look,
    # 那是为了下一轮少走一趟空行程, 收身这件事由状态机在真要走时才做.
    def leave_shelf(self):
        self.get_logger().info('==== 离开货架前摆臂回 ready ====')
        if not self.call_trigger(self.ready_cli, '/grasp/ready', 30.0):
            self.get_logger().error('离开货架前回 ready 失败, 任务中止 (底盘不能拖着伸出的臂走)')
            return False
        return True

    # grasp_node 在 /grasp/look 与 /grasp/execute 的 message 里带 "pickable=N, tray_free=M".
    # 解析不出(老版本 grasp_node / 消息格式变了)返回 -1: 调用方按"未知"处理而不是按 0 ——
    # 报 0 会让循环立刻收工, 明明还有盒没抓.
    @staticmethod
    def _parse_kv_int(msg, key):
        import re
        m = re.search(rf'{key}=(-?\d+)', msg or '')
        return int(m.group(1)) if m else -1

    def _pickable_from(self, msg):
        return self._parse_kv_int(msg, 'pickable')

    def _tray_free_from(self, msg):
        return self._parse_kv_int(msg, 'tray_free')

    # ---- S0 INIT ----
    def stage_init(self):
        self.get_logger().info(
            f'==== S0 初始化定位: 发 /initialpose ({self.init_x:.2f},'
            f'{self.init_y:.2f},yaw={self.init_yaw:.2f}) ====')
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = self.init_x
        msg.pose.pose.position.y = self.init_y
        qz, qw = yaw_to_quat(self.init_yaw)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # 小协方差: 告诉 AMCL 这是较可信初值 (x,y ~0.25m, yaw ~0.07rad)
        cov = [0.0] * 36
        cov[0] = 0.25 * 0.25
        cov[7] = 0.25 * 0.25
        cov[35] = 0.068 * 0.068
        msg.pose.covariance = cov
        self.get_logger().info('S0 循环补发 /initialpose 并等 AMCL 收敛 ...')
        if not self.wait_for_localization(msg):
            return False
        # 清 grasp_node 的堆叠状态与残留 placed_* 碰撞体: 不清则上一轮记的"盒还在托盘上"
        # 会让本轮一开抓就 TRAY_FULL, 场景里的残留碰撞体还会挡住放置直下段.
        # 失败不中止: 首次启动本来就是干净的, reset 只是保险 (服务没起也不该拦住整条任务).
        self.get_logger().info('S0 清抓取堆叠状态 (/grasp/reset_stack)')
        if not self.call_trigger(self.reset_cli, '/grasp/reset_stack', 15.0):
            self.get_logger().warn('S0 reset_stack 失败, 继续 (若上一轮有残留可能影响放置)')
        # 底盘行进前先把机械臂摆回 ready 位 (臂收身前, 底盘不拖着伸出的臂走)
        self.get_logger().info('S0 定位就绪, 底盘行进前摆臂回 ready')
        if not self.call_trigger(self.ready_cli, '/grasp/ready', 30.0):
            self.get_logger().error('S0 机械臂回 ready 失败, 任务中止')
            return False
        return True

    def wait_for_localization(self, msg):
        # AMCL 的 /initialpose 订阅是 VOLATILE: 建 publisher 后立即连发会赶在发现完成前,
        # 消息被丢. 故把"发布"并进等待循环, 每 0.5s 补发一次, 直到 map->base_link 出现.
        # AMCL 收敛后才发 map->odom, map 帧才存在; 没等到就进 S1 会被 lane_navigator
        # 的 map<-base_link 查询判 None -> 整轮误判 FAILED.
        deadline = self.now() + self.t_localize
        n = 0
        while rclpy.ok() and self.now() < deadline:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.initpose_pub.publish(msg)
            n += 1
            if self.tf_buffer.can_transform('map', 'base_link', Time()):
                self.get_logger().info(f'S0 map->base_link 可用, 定位就绪 (发了 {n} 次 initialpose)')
                return True
            self.sleep(0.5)
        self.get_logger().error(
            f'S0 定位超时 ({self.t_localize:.0f}s): map->base_link 不可用 (AMCL 未收敛?)')
        return False

    # ---- S1 NAV ----
    def stage_nav(self, target):
        self.get_logger().info(f'==== S1 导航到 {target} ====')
        # 记下发目标**之前**已见的最大 seq: 只有 seq 比它大的终态才是本轮的回报。
        # ⚠️ 这道判据不能省成"清 None 再等" —— 本节点若在 lane_navigator 之后启动,
        # 且对方话题将来又被改回 latched, 一订上就会收到上一轮的旧终态, S1 会瞬间假成功
        # (车压根没动就进 S3 抓取)。seq 让旧终态自然落在门限之下被忽略。
        seen = self._nav_status[0] if self._nav_status else None
        self._seq_floor = seen if seen is not None else 0
        self._nav_status = None
        self.goto_pub.publish(String(data=target))
        deadline = self.now() + self.t_nav
        want_ok = f'{target}:SUCCEEDED'
        want_fail = f'{target}:FAILED'
        while rclpy.ok() and self.now() < deadline:
            st = self._nav_status
            if st is not None:
                seq, verdict = st
                # seq 为 None = 对方是无序号旧版, 退回"只比字符串"(行为与改动前一致)
                fresh = seq is None or seq > self._seq_floor
                if fresh and verdict == want_ok:
                    self.get_logger().info(f'S1 到达 {target}')
                    return True
                if fresh and verdict == want_fail:
                    self.get_logger().error(f'S1 导航失败: {target}')
                    return False
            self.sleep(0.1)
        self.get_logger().error(f'S1 导航超时 ({self.t_nav:.0f}s): {target}')
        return False

    # ---- S2 ALIGN ----
    def stage_align(self, target):
        self.get_logger().info(f'==== S2 精对位 {target}: ArUco 伺服 TODO (本轮 no-op, 直接放行) ====')

    # ---- S3a LOOK (仅 grasp): 摆看货姿势, ready+J1+90° 让相机转向货物再识别 ----
    def stage_look(self):
        self.get_logger().info('==== S3a 摆看货姿势 (ready+J1+90°, 供视觉识别) ====')
        return self.call_trigger(self.look_cli, '/grasp/look', 30.0)

    # ---- S3 DETECT ----
    def stage_detect(self):
        # 只认"够得着范围内的新鲜帧". 仿真里 place_box_helper 也订 /lane_navigator/status,
        # 到位后才把盒子瞬移到车右侧可达点; 二者与本 S3 存在竞态, 若抓第一帧可能读到尚未挪走
        # 的远盒 -> 粗定位残差大 -> 精修 xy 收不回. 故等盒子落进 base_link 系可达范围
        # (|x|<0.5, |y|<0.6) 的新鲜帧再放行, 彻底避开旧远盒.
        self.get_logger().info('==== S3 识别货物: 等 /perception/object_pose 够得着的新鲜帧 ====')
        deadline = self.now() + self.t_detect
        while rclpy.ok() and self.now() < deadline:
            obj = self._last_obj
            if obj is not None:
                age = self.now() - obj[0]
                p = obj[1].pose.position
                if age < 1.0 and abs(p.x) < 0.5 and abs(p.y) < 0.6:
                    self.get_logger().info(
                        f'S3 拿到 object_pose ({p.x:.3f},{p.y:.3f},{p.z:.3f}) age={age:.2f}s')
                    return True
                self.get_logger().info(
                    f'S3 等待可达帧: obj=({p.x:.3f},{p.y:.3f}) age={age:.2f}s (需 |x|<0.5 |y|<0.6 age<1.0)',
                    throttle_duration_sec=1.0)
            self.sleep(0.1)
        self.get_logger().error(f'S3 识别超时 ({self.t_detect:.0f}s): 无够得着的新鲜 object_pose')
        return False

    # ---- S4 GRASP / UNLOAD ----
    # 返回 (success, message): 同货架连抓要从 message 里读 "pickable=N, tray_free=M" 和
    # TRAY_FULL 标记, 光一个 bool 分不出"托盘满了该去卸货"和"规划失败"这两种 success=false.
    def stage_grasp_msg(self, cli, name):
        self.get_logger().info(f'==== S4 调 {name} ====')
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'S4 服务不可用: {name}')
            return False, ''
        future = cli.call_async(Trigger.Request())
        deadline = self.now() + self.t_grasp
        while rclpy.ok() and self.now() < deadline:
            if future.done():
                resp = future.result()
                if resp.success:
                    self.get_logger().info(f'S4 {name} 成功: {resp.message}')
                else:
                    self.get_logger().error(f'S4 {name} 失败: {resp.message}')
                return resp.success, resp.message
            self.sleep(0.1)
        self.get_logger().error(f'S4 {name} 超时 ({self.t_grasp:.0f}s)')
        return False, ''

    # 通用 Trigger 服务调用 (worker 线程阻塞轮询 future, 响应由主线程 executor 处理).
    # 用于 /grasp/ready 与 /grasp/look 这类"发一次等一次"的臂姿服务.
    def call_trigger(self, cli, name, timeout):
        self._last_trigger_msg = ''
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'{name} 服务不可用')
            return False
        future = cli.call_async(Trigger.Request())
        deadline = self.now() + timeout
        while rclpy.ok() and self.now() < deadline:
            if future.done():
                resp = future.result()
                # /grasp/look 的 message 里带 "pickable=N, tray_free=M", stage_grasp_shelf 要读
                self._last_trigger_msg = resp.message
                if resp.success:
                    self.get_logger().info(f'{name} 成功: {resp.message}')
                else:
                    self.get_logger().error(f'{name} 失败: {resp.message}')
                return resp.success
            self.sleep(0.1)
        self.get_logger().error(f'{name} 超时 ({timeout:.0f}s)')
        return False

    # ---- 时间/睡眠工具 (用 wall clock; 阻塞在 worker 线程, 不卡 executor) ----
    def now(self):
        import time
        return time.monotonic()

    def sleep(self, sec):
        import time
        time.sleep(sec)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
