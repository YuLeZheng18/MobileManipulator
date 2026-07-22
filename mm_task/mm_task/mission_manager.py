#!/usr/bin/env python3
"""
mm_task / mission_manager — 顶层任务状态机 S0–S5 (架构 §7.2)

只做"编排", 不重算几何: 导航交 lane_navigator, 抓取/卸货整段交 grasp_node 的
/grasp/execute /grasp/unload (三段抓取 + 末段相对直插纪律都在 grasp_node 内).

状态流 (worker 线程 run_mission):
  S0 INIT    发 /initialpose (可配已知位姿) 给 AMCL 初值
  S1 NAV     发 /go_to=<nav_target>, 等 /lane_navigator/status "<target>:SUCCEEDED"
  S2 ALIGN   ArUco 精对位 (本轮 no-op, 直接放行)
  S3 DETECT  仅 action==grasp 时: 等 /perception/object_pose 新鲜 (age<1s)
  S4 GRASP/UNLOAD  action 分派: grasp→/grasp/execute; unload→/grasp/unload; none→跳过
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

        self._nav_status = None      # 最近一条 "<target>:SUCCEEDED|FAILED"
        self.create_subscription(
            String, '/lane_navigator/status', self.on_nav_status, 10,
            callback_group=cbg)

        self._last_obj = None        # (stamp_sec, PoseStamped)
        self.create_subscription(
            PoseStamped, '/perception/object_pose', self.on_object, 10,
            callback_group=cbg)

        self.grasp_cli = self.create_client(Trigger, '/grasp/execute', callback_group=cbg)
        self.unload_cli = self.create_client(Trigger, '/grasp/unload', callback_group=cbg)
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
        self.tasks = cfg.get('tasks', [])

    # ---- 订阅回调 ----
    def on_nav_status(self, msg):
        self._nav_status = msg.data

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
            self.get_logger().info(
                f'==== 任务 {i + 1}/{len(self.tasks)}: nav={target} action={action} ====')
            if not self.run_task(target, action):
                self.get_logger().error(f'任务 {i + 1} 失败, 任务序列中止')
                return
        self.get_logger().info('==== 全部任务完成 DONE ====')

    def run_task(self, target, action):
        if not self.stage_nav(target):
            return False
        self.stage_align(target)
        if action == 'grasp':
            if not self.stage_look():
                return False
            if not self.stage_detect():
                return False
            return self.stage_grasp(self.grasp_cli, '/grasp/execute')
        if action == 'unload':
            return self.stage_grasp(self.unload_cli, '/grasp/unload')
        if action == 'none':
            self.get_logger().info('action=none: 仅导航, 跳过抓取')
            return True
        self.get_logger().error(f'未知 action="{action}"')
        return False

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
        self._nav_status = None
        self.goto_pub.publish(String(data=target))
        deadline = self.now() + self.t_nav
        want_ok = f'{target}:SUCCEEDED'
        want_fail = f'{target}:FAILED'
        while rclpy.ok() and self.now() < deadline:
            st = self._nav_status
            if st == want_ok:
                self.get_logger().info(f'S1 到达 {target}')
                return True
            if st == want_fail:
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
    def stage_grasp(self, cli, name):
        self.get_logger().info(f'==== S4 调 {name} ====')
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'S4 服务不可用: {name}')
            return False
        future = cli.call_async(Trigger.Request())
        deadline = self.now() + self.t_grasp
        while rclpy.ok() and self.now() < deadline:
            if future.done():
                resp = future.result()
                if resp.success:
                    self.get_logger().info(f'S4 {name} 成功: {resp.message}')
                    return True
                self.get_logger().error(f'S4 {name} 失败: {resp.message}')
                return False
            self.sleep(0.1)
        self.get_logger().error(f'S4 {name} 超时 ({self.t_grasp:.0f}s)')
        return False

    # 通用 Trigger 服务调用 (worker 线程阻塞轮询 future, 响应由主线程 executor 处理).
    # 用于 /grasp/ready 与 /grasp/look 这类"发一次等一次"的臂姿服务.
    def call_trigger(self, cli, name, timeout):
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'{name} 服务不可用')
            return False
        future = cli.call_async(Trigger.Request())
        deadline = self.now() + timeout
        while rclpy.ok() and self.now() < deadline:
            if future.done():
                resp = future.result()
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
