#!/usr/bin/env python3
"""
方案A 车道导航节点 — 固定路网 + Dijkstra + 圆角连续路径 (一次走完, 拐角不停)

机器人在固定车道图上行走: 当前位置投影到最近车道边, Dijkstra 求到目标 node 的
最短路, 得到一串顶点. 把顶点序列里每个 90° 直角拐角用半径 corner_radius 的圆弧
倒成圆角, 直线段 + 圆弧串成一条连续 Path, 每个点朝向 = 该点行进切线方向.

整条路线 = [起步自转对齐首段切线, FollowPath 跑完整条圆角连续路径, 终点自转对齐目标 yaw].

关键点(为何能拐角不停): 底盘全向(Omni). 过圆角时车头沿切线"边平移边缓转",
不需要在拐点停车再自转. MPPI(Omni + PathAlignCritic use_path_orientations)锁车头
跟随路径切线, 圆弧足够缓(默认 r=0.4)朝向变化平滑, 不会出现急转抖动. 段内遇障由
MPPI 横移(vy)绕开; 真堵死则调 NavFn 重规划到终点节点、按切线朝向跟随绕行路径.

旧架构是"每个节点停+自转+直行"的离散分段: 副作用是每个拐角被 xy_goal_tolerance
提前 25cm 判完成 -> 不到拐角就停下自转("提前停"). 改成连续圆角后, 拐角不再是 goal,
容差只在最终节点生效 -> 拐角不停, 顺带消除提前停.

绕过 planner_server 与 BT(仅绕障重规划借用 NavFn). 触发与可视化:
    ros2 topic pub --once /go_to std_msgs/msg/String "{data: p3}"
    RViz 订阅 /lane_plan (nav_msgs/Path) 看整条圆角路线
"""
import math
import heapq

import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowPath, ComputePathToPose
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def norm_angle(a):
    """归一化到 (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class LaneNavigator(Node):
    def __init__(self):
        super().__init__('lane_navigator')

        self.declare_parameter('lane_graph', '')
        # 到达判定半径(米): 触发的目标若已在此范围内, 直接忽略
        self.declare_parameter('arrival_tolerance', 0.2)
        # 起步朝向对齐(闭环 cspin): 误差 < start_yaw_tol 跳过(残差极小, 交 MPPI 边走边顺, 看不出);
        # 否则用 P 闭环转到首段切线并"沉降"(连续几拍零速且落容差内)再放行 drive -> 保证转停稳了才跑,
        # 杜绝"没转完就跑/起步划弧甩头". 旧版用开环 Nav2 Spin: 报完成时车身还在泄角速度, 同一拍 MPPI
        # 已发前进 -> 残余自转+前进叠加成起步甩头; 闭环+沉降根治. 门限收到 5°(旧 28°)让中等误差也转
        # 到位再走, 不再边跑边扭.
        self.declare_parameter('start_yaw_tol', 0.087)   # ~5°: 起步对齐到位阈值(同时作跳过门限)
        self.declare_parameter('start_wz_max', 0.5)      # 起步转速上限(rad/s): 比终点精对 0.4 略快, 大角度起步不肉但不甩
        # 横向并入阈值(米): 车到车道横向距离小于此值视为已在道上, 跳过垂足 Q 直连
        # 第一个车道节点 -> 一次转到位, 残余横移交 MPPI PathAlign 拉正(避免 90+90 折线掉头)
        self.declare_parameter('merge_skip_dist', 0.25)
        # 拐角圆角半径(米): 每个内部顶点处把直角倒成此半径的圆弧, 车沿圆弧边平移边缓转,
        # 不在拐点停车. 越大越顺但越占走廊内侧空间; 机器人半径 0.20. 0.8: 弯更缓, 进弯前
        # MPPI 高速样本不被甩出弯 -> 减速/重规划消失; 代价是占内侧 0.8m, 走廊须够宽.
        self.declare_parameter('corner_radius', 0.8)
        # 终点 yaw 闭环对齐参数(cspin): Nav2 Spin 是开环(到点停发命令), 底盘 cmd_vel 有加速度
        # 斜坡+惯性 -> 停发后滑过目标留残差. 改用本节点读 TF 真实 yaw 的 P 闭环, 过冲自动反向
        # 修回, 落在 final_yaw_tol 内. (位置精度交 MPPI drive 段的 xy_goal_tolerance, 不在此处)
        self.declare_parameter('final_yaw_tol', 0.017)   # ~1°: 终点朝向到位阈值
        self.declare_parameter('cspin_kp', 1.2)          # 角 P 增益: wz = kp*yaw_err
        self.declare_parameter('cspin_wz_max', 0.4)      # 角速度上限(rad/s)
        self.declare_parameter('cspin_wz_min', 0.06)     # yaw 误差>tol 时最小转速地板, 克服静摩擦
        self.declare_parameter('cspin_timeout', 15.0)    # 超时(秒): 防卡死, 到点放弃微调直接完成
        # drive 段失败(如动态障碍逼停)的"快重试"次数; 每次重取位姿重规划绕障路径重发
        self.declare_parameter('drive_max_retries', 3)
        # 快重试前的等待(秒); 设小=判定堵死后几乎立刻重规划绕行, 不傻等
        self.declare_parameter('drive_retry_delay', 0.2)
        # 快重试用尽后进入"慢恢复"的重试周期(秒): 不放弃路线, 持续重规划等障碍移开
        # -> 障碍没了就自动接着往目标走(不永久死停). 设大一点避免堵死时狂刷规划.
        self.declare_parameter('recovery_retry_delay', 2.0)
        self.arrival_tol = self.get_parameter(
            'arrival_tolerance').get_parameter_value().double_value
        self.start_yaw_tol = self.get_parameter(
            'start_yaw_tol').get_parameter_value().double_value
        self.start_wz_max = self.get_parameter(
            'start_wz_max').get_parameter_value().double_value
        self.merge_skip_dist = self.get_parameter(
            'merge_skip_dist').get_parameter_value().double_value
        self.corner_radius = self.get_parameter(
            'corner_radius').get_parameter_value().double_value
        self.final_yaw_tol = self.get_parameter(
            'final_yaw_tol').get_parameter_value().double_value
        self.cspin_kp = self.get_parameter(
            'cspin_kp').get_parameter_value().double_value
        self.cspin_wz_max = self.get_parameter(
            'cspin_wz_max').get_parameter_value().double_value
        self.cspin_wz_min = self.get_parameter(
            'cspin_wz_min').get_parameter_value().double_value
        self.cspin_timeout = self.get_parameter(
            'cspin_timeout').get_parameter_value().double_value
        self.drive_max_retries = self.get_parameter(
            'drive_max_retries').get_parameter_value().integer_value
        self.drive_retry_delay = self.get_parameter(
            'drive_retry_delay').get_parameter_value().double_value
        self.recovery_retry_delay = self.get_parameter(
            'recovery_retry_delay').get_parameter_value().double_value
        graph_path = self.get_parameter('lane_graph').get_parameter_value().string_value
        if not graph_path:
            from ament_index_python.packages import get_package_share_directory
            graph_path = get_package_share_directory('mm_navigation') + '/config/lane_graph.yaml'
        self.get_logger().info(f'Loading lane graph: {graph_path}')
        self.load_graph(graph_path)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 锁存(latched)发布: RViz 任何时刻订阅都能立刻拿到最后一条路径
        latched = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.plan_pub = self.create_publisher(Path, 'lane_plan', latched)
        # 终点闭环 spin 直接发 /cmd_vel(与 Nav2 Spin 行为同一话题), 经 cmd_vel_smoother 到底盘
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.go_sub = self.create_subscription(String, 'go_to', self.on_go_to, 10)

        self._follow_client = ActionClient(self, FollowPath, 'follow_path')
        # 堵死重规划用: 调 planner_server(Theta*) 算绕障路径
        self._planner_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        # 状态机状态:
        #   _active_target : 当前正在追的目标名(None = 空闲)
        #   _steps         : [('spin', heading) | ('drive', p0, p1, heading), ...]
        #   _step_idx      : 当前执行到第几步
        #   _goal_handle   : 当前在途 action goal 句柄(用于切目标时抢占)
        #   _epoch         : 路线代号; 每次新路线 +1, 旧步骤回调凭 epoch 失效, 防串线
        self._active_target = None
        self._steps = []
        self._step_idx = 0
        self._goal_handle = None
        self._epoch = 0
        #   _retry_count   : 当前 drive 步骤已重试次数(成功推进/新路线时清零)
        #   _retry_timer   : 重试延时定时器句柄(抢占时取消)
        self._retry_count = 0
        self._retry_timer = None
        #   _cspin_timer   : 终点闭环对齐控制定时器(20Hz); 抢占/失败/完成时取消并停车
        self._cspin_timer = None

        self.get_logger().info(
            f'Lane graph loaded: {len(self.nodes)} nodes, {len(self.adj)} adjacency entries. '
            f'Trigger with: ros2 topic pub --once /go_to std_msgs/msg/String "{{data: <node>}}"')

    # ---------- graph ----------
    def load_graph(self, path):
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        self.frame_id = data.get('frame_id', 'map')
        self.point_spacing = float(data.get('point_spacing', 0.05))
        self.nodes = {}
        for name, v in data['nodes'].items():
            self.nodes[name] = (float(v['x']), float(v['y']), float(v.get('yaw', 0.0)))
        self.adj = {name: [] for name in self.nodes}
        for a, b in data['edges']:
            d = math.hypot(self.nodes[a][0] - self.nodes[b][0],
                           self.nodes[a][1] - self.nodes[b][1])
            self.adj[a].append((b, d))
            self.adj[b].append((a, d))

    def nearest_edge(self, px, py):
        """车投影到最近车道边. 返回 (dist, qx, qy, node_a, node_b)."""
        best = None
        seen = set()
        for a in self.adj:
            for b, _ in self.adj[a]:
                if (b, a) in seen:
                    continue
                seen.add((a, b))
                ax, ay, _ = self.nodes[a]
                bx, by, _ = self.nodes[b]
                abx, aby = bx - ax, by - ay
                ab2 = abx * abx + aby * aby
                t = 0.0 if ab2 < 1e-9 else ((px - ax) * abx + (py - ay) * aby) / ab2
                t = max(0.0, min(1.0, t))
                qx, qy = ax + t * abx, ay + t * aby
                d = math.hypot(px - qx, py - qy)
                if best is None or d < best[0]:
                    best = (d, qx, qy, a, b)
        return best

    def dijkstra(self, start, goal):
        """返回 (route_list, total_dist); 不可达返回 (None, inf)."""
        dist = {start: 0.0}
        prev = {}
        pq = [(0.0, start)]
        while pq:
            d, u = heapq.heappop(pq)
            if u == goal:
                break
            if d > dist.get(u, float('inf')):
                continue
            for v, w in self.adj[u]:
                nd = d + w
                if nd < dist.get(v, float('inf')):
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))
        if goal not in dist:
            return None, float('inf')
        route = [goal]
        while route[-1] != start:
            route.append(prev[route[-1]])
        route.reverse()
        return route, dist[goal]

    # ---------- pose ----------
    def get_robot_pose(self):
        """返回 (x, y, yaw); 取不到返回 None."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.frame_id, 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=0.5))
        except TransformException as ex:
            self.get_logger().warn(f'No robot pose: {ex}')
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    # ---------- path ----------
    def _sample_line(self, xy, p0, p1):
        """把线段 p0->p1 按 point_spacing 加密追加到 xy(不含 p0, 含 p1)."""
        seg = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        n = max(1, int(seg / self.point_spacing))
        for k in range(1, n + 1):
            t = k / n
            xy.append((p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t))

    def build_rounded_xy(self, pts, r):
        """顶点序列 pts -> 带圆角的连续加密点列 xy.
        每个内部顶点 V(在 A-V-B 之间)用半径 r 的圆弧倒角: 在 V 前后各 T 处与两边相切,
        T = r/tan(alpha/2) (alpha=两边夹角), 并裁剪到不超过相邻段一半防重叠. 起点/终点不倒角.
        近共线/近掉头的顶点跳过倒角直接穿过."""
        n = len(pts)
        fillets = {}
        for i in range(1, n - 1):
            A, V, B = pts[i - 1], pts[i], pts[i + 1]
            v1x, v1y = A[0] - V[0], A[1] - V[1]
            v2x, v2y = B[0] - V[0], B[1] - V[1]
            l1 = math.hypot(v1x, v1y)
            l2 = math.hypot(v2x, v2y)
            if l1 < 1e-6 or l2 < 1e-6:
                continue
            u1x, u1y = v1x / l1, v1y / l1
            u2x, u2y = v2x / l2, v2y / l2
            dot = max(-1.0, min(1.0, u1x * u2x + u1y * u2y))
            alpha = math.acos(dot)
            if alpha > math.pi - 0.05 or alpha < 0.05:
                continue  # 近共线或近掉头, 不倒角
            half = alpha / 2.0
            T = min(r / math.tan(half), 0.45 * l1, 0.45 * l2)
            r_eff = T * math.tan(half)
            p1 = (V[0] + u1x * T, V[1] + u1y * T)
            p2 = (V[0] + u2x * T, V[1] + u2y * T)
            bx, by = u1x + u2x, u1y + u2y
            bl = math.hypot(bx, by)
            if bl < 1e-6:
                continue
            cdist = r_eff / math.sin(half)
            cx, cy = V[0] + bx / bl * cdist, V[1] + by / bl * cdist
            a1 = math.atan2(p1[1] - cy, p1[0] - cx)
            a2 = math.atan2(p2[1] - cy, p2[0] - cx)
            dtheta = norm_angle(a2 - a1)  # 取劣弧(偏转角 = pi-alpha < pi)
            fillets[i] = (p1, p2, cx, cy, a1, dtheta, r_eff)

        xy = [pts[0]]
        cur = pts[0]
        for i in range(1, n):
            if i in fillets:
                p1, p2, cx, cy, a1, dtheta, r_eff = fillets[i]
                self._sample_line(xy, cur, p1)
                arc_len = abs(dtheta) * r_eff
                na = max(1, int(arc_len / self.point_spacing))
                for k in range(1, na + 1):
                    ang = a1 + dtheta * (k / na)
                    xy.append((cx + r_eff * math.cos(ang), cy + r_eff * math.sin(ang)))
                cur = p2
            else:
                self._sample_line(xy, cur, pts[i])
                cur = pts[i]

        dedup = [xy[0]]
        for p in xy[1:]:
            if math.hypot(p[0] - dedup[-1][0], p[1] - dedup[-1][1]) > 1e-4:
                dedup.append(p)
        return dedup

    def path_from_xy(self, xy):
        """加密点列 -> nav_msgs/Path, 每点朝向 = 到下一点的切线(末点沿用前一朝向)."""
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        prev_h = 0.0
        for k in range(len(xy)):
            if k < len(xy) - 1:
                dx, dy = xy[k + 1][0] - xy[k][0], xy[k + 1][1] - xy[k][1]
                h = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-6 else prev_h
            else:
                h = prev_h
            self.append_pose(path, xy[k][0], xy[k][1], h)
            prev_h = h
        return path

    def set_tangent_orientations(self, path):
        """就地把一条 Path 的每个 pose 朝向重写成切线方向(用于 NavFn 绕障路径)."""
        poses = path.poses
        prev_h = 0.0
        for k in range(len(poses)):
            if k < len(poses) - 1:
                dx = poses[k + 1].pose.position.x - poses[k].pose.position.x
                dy = poses[k + 1].pose.position.y - poses[k].pose.position.y
                h = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-6 else prev_h
            else:
                h = prev_h
            z, w = yaw_to_quat(h)
            poses[k].pose.orientation.x = 0.0
            poses[k].pose.orientation.y = 0.0
            poses[k].pose.orientation.z = z
            poses[k].pose.orientation.w = w
            prev_h = h

    def append_pose(self, path, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = self.frame_id
        p.pose.position.x = x
        p.pose.position.y = y
        z, w = yaw_to_quat(yaw)
        p.pose.orientation.z = z
        p.pose.orientation.w = w
        path.poses.append(p)

    # ---------- trigger ----------
    def on_go_to(self, msg):
        target = msg.data.strip()
        if target not in self.nodes:
            self.get_logger().error(f'Unknown node "{target}". Known: {list(self.nodes)}')
            return
        pose = self.get_robot_pose()
        if pose is None:
            return
        px, py, _ = pose

        # 已在目标点附近 -> 忽略
        tx, ty, _ = self.nodes[target]
        d_goal = math.hypot(px - tx, py - ty)
        if d_goal <= self.arrival_tol:
            self.get_logger().info(f'Already at "{target}" (dist={d_goal:.2f}m), ignoring')
            return

        # 相同目标且仍在执行 -> 忽略重复触发(防 ros2 topic pub -r 连发抖动)
        if self._active_target == target:
            self.get_logger().info(f'Target "{target}" already in progress, ignoring duplicate')
            return

        # 投影到最近车道边, 从投影点 Q 并入车道; 端点 a/b 选"经它到目标总程最短"那个,
        # 避免 snap 到反方向 node 造成先倒退再折返的锐角.
        ne = self.nearest_edge(px, py)
        if ne is None:
            self.get_logger().error('No edges in lane graph')
            return
        d_lat, qx, qy, a, b = ne
        ra, da = self.dijkstra(a, target)
        rb, db = self.dijkstra(b, target)
        cost_a = math.hypot(qx - self.nodes[a][0], qy - self.nodes[a][1]) + da
        cost_b = math.hypot(qx - self.nodes[b][0], qy - self.nodes[b][1]) + db
        if cost_a <= cost_b and ra is not None:
            route = ra
        elif rb is not None:
            route = rb
        else:
            self.get_logger().error(f'No route to {target}')
            return

        route_pts = [(self.nodes[n][0], self.nodes[n][1]) for n in route]
        # 垂足 Q 到首节点的距离: Q 贴近首节点时, 插 Q 会制造一条 <2*corner_radius 的短边,
        # 该短边两端各一个 90° 弯被防重叠裁剪压到极小半径 -> MPPI 车头跟不上急弯切线(转速
        # 需求 > wz_max)而蠕动 -> 触发 progress 失败重规划. 故 Q 贴近首节点时丢弃 Q 直连.
        q_to_first = math.hypot(qx - route_pts[0][0], qy - route_pts[0][1])
        # 车已基本在车道上(横向距离小), 或垂足贴近首节点: 跳过垂足 Q 的折线并入, 从车位置
        # 直连第一个车道节点 -> 转向一次到位; 否则保留垂直并入(车离车道远时需先回到车道).
        if d_lat < self.merge_skip_dist or q_to_first < 2.0 * self.corner_radius:
            self.get_logger().info(
                f'Direct merge to {route[0]} (d_lat={d_lat:.2f} q_to_first={q_to_first:.2f}) '
                f'-> route {" -> ".join(route)}')
            waypoints = [(px, py)] + route_pts
        else:
            self.get_logger().info(
                f'Merge on edge {a}-{b} at ({qx:.2f},{qy:.2f}) -> {" -> ".join(route)}')
            waypoints = [(px, py), (qx, qy)] + route_pts
        self.start_route(target, waypoints)

    # ---------- 状态机 ----------
    def start_route(self, target, waypoints):
        # 去掉相邻重复点(投影点可能与端点/相邻 node 重合), 避免零长段
        pts = [waypoints[0]]
        for p in waypoints[1:]:
            if math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) > 1e-3:
                pts.append(p)
        if len(pts) < 2:
            self.get_logger().info('Already on target node, nothing to do')
            return

        # 圆角连续路径: 顶点序列倒圆角 -> 一条加密 Path(每点切线朝向). 整条当一个 drive 步骤, 拐角不停.
        # 起步用闭环 cspin 对齐首段切线(start_yaw_tol~5°/start_wz_max~0.8): P 闭环转到切线并沉降
        # (连续几拍零速落容差)再放行 drive -> 转停稳了才跑, 不会没转完就被 MPPI 前进抢走而起步甩头.
        # 终点同样用闭环 cspin 对齐目标 yaw(final_yaw_tol~1°/cspin_wz_max 0.4, 紧而稳): 读 TF 真实
        # yaw 做 P 控制直接发 /cmd_vel, 过冲自动反向修回(开环 Spin 会被底盘斜坡滑过). 位置精度交 MPPI
        # drive 段的 xy_goal_tolerance(收紧它=更准, 代价是终点附近 MPPI 会蠕动收尾, 已接受).
        xy = self.build_rounded_xy(pts, self.corner_radius)
        if len(xy) < 2:
            self.get_logger().info('Degenerate route, nothing to do')
            return
        path = self.path_from_xy(xy)
        first_heading = math.atan2(xy[1][1] - xy[0][1], xy[1][0] - xy[0][0])
        last_heading = math.atan2(xy[-1][1] - xy[-2][1], xy[-1][0] - xy[-2][0])
        goal_xy = (pts[-1][0], pts[-1][1])
        steps = [
            ('cspin', first_heading, self.start_yaw_tol, self.start_wz_max),
            ('drive', path, goal_xy, last_heading),
            ('cspin', self.nodes[target][2], self.final_yaw_tol, self.cspin_wz_max),
        ]

        # 可视化整条圆角路线
        self.plan_pub.publish(path)

        # 新路线: epoch+1; 若有在途 goal 先抢占(其回调凭旧 epoch 自动失效)
        self._epoch += 1
        ep = self._epoch
        if self._goal_handle is not None:
            self.get_logger().info(f'Preempting current route for new target "{target}"')
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._cancel_retry_timer()
        if self._cspin_timer is not None:  # 抢占在途终点闭环对齐: 停转并清定时器
            self._cancel_cspin_timer()
            self.cmd_pub.publish(Twist())
        self._retry_count = 0
        self._active_target = target
        self._steps = steps
        self._step_idx = 0
        self.get_logger().info(f'Route "{target}": {len(steps)} steps over {len(pts) - 1} legs')
        self.run_next_step(ep)

    def run_next_step(self, ep):
        if ep != self._epoch:
            return
        if self._step_idx >= len(self._steps):
            self.get_logger().info(f'Route to "{self._active_target}" complete')
            self._active_target = None
            self._goal_handle = None
            self._steps = []
            return
        step = self._steps[self._step_idx]
        if step[0] == 'cspin':
            self.do_cspin(step[1], step[2], step[3], ep)
        else:
            # step = ('drive', path, goal_xy, last_heading)
            self.get_logger().info(
                f'[step {self._step_idx}] Drive rounded route, {len(step[1].poses)} poses')
            self.follow_path(step[1], ep)

    def do_cspin(self, target_yaw, tol, wz_max, ep):
        """yaw 闭环对齐(起步对首段切线 / 终点对目标 yaw 共用): 起一个 20Hz 控制定时器, 读 TF 真实
        yaw 做 P 控制发 /cmd_vel, 过冲自动反向修回; 连续几拍落 tol 内且零速(沉降)才推进下一步
        -> 保证"转停稳了才跑", 不会没转完就被 drive 抢走. 起步松而快(start_yaw_tol/start_wz_max),
        终点紧而稳(final_yaw_tol/cspin_wz_max). (位置精度交 MPPI drive 段, 这里只对朝向)"""
        self._cancel_cspin_timer()
        self._cspin_tol = tol
        self._cspin_wzmax = wz_max
        pose = self.get_robot_pose()
        if pose is not None and abs(norm_angle(target_yaw - pose[2])) < tol:
            self.advance_step(ep)  # 已对齐, 跳过
            return
        self.get_logger().info(
            f'[step {self._step_idx}] Closed-loop spin -> yaw {math.degrees(target_yaw):.0f}deg '
            f'(tol={math.degrees(tol):.0f}deg)')
        self._cspin_target = target_yaw
        self._cspin_t0 = self.get_clock().now()
        self._cspin_dwell = 0
        self._cspin_timer = self.create_timer(0.05, lambda: self._cspin_tick(ep))

    def _cspin_tick(self, ep):
        # 被抢占/失效: 停车并清定时器
        if ep != self._epoch:
            self.cmd_pub.publish(Twist())
            self._cancel_cspin_timer()
            return
        pose = self.get_robot_pose()
        if pose is None:
            return  # 暂取不到位姿, 下一拍再试(不发命令 -> 平滑器超时归零)
        err = norm_angle(self._cspin_target - pose[2])
        elapsed = (self.get_clock().now() - self._cspin_t0).nanoseconds / 1e9
        # 到位: 连续 3 拍(~0.15s)在容差内才算稳, 防过冲瞬间穿越误判
        if abs(err) < self._cspin_tol:
            self._cspin_dwell += 1
            self.cmd_pub.publish(Twist())
            if self._cspin_dwell >= 3:
                self.get_logger().info(
                    f'[step {self._step_idx}] Aligned: err={math.degrees(err):+.1f}deg')
                self._cancel_cspin_timer()
                self.advance_step(ep)
            return
        self._cspin_dwell = 0
        if elapsed > self.cspin_timeout:
            self.get_logger().warn(
                f'[step {self._step_idx}] cspin timeout, err={math.degrees(err):+.1f}deg -> accept')
            self.cmd_pub.publish(Twist())
            self._cancel_cspin_timer()
            self.advance_step(ep)
            return
        wz = self.cspin_kp * err
        wz = max(-self._cspin_wzmax, min(self._cspin_wzmax, wz))
        if abs(wz) < self.cspin_wz_min:  # 误差仍超容差但 P 输出太小 -> 抬到地板克服静摩擦
            wz = math.copysign(self.cspin_wz_min, err)
        cmd = Twist()
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)

    def _cancel_cspin_timer(self):
        if self._cspin_timer is not None:
            self._cspin_timer.cancel()
            self.destroy_timer(self._cspin_timer)
            self._cspin_timer = None

    def follow_path(self, path, ep):
        """发 FollowPath 跟随给定路径(死直线 或 重规划绕障路径), 复用同一结果回调."""
        if not self._follow_client.wait_for_server(timeout_sec=2.0):
            self.fail_route(ep, 'follow_path action server not available')
            return
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'general_goal_checker'
        fut = self._follow_client.send_goal_async(goal)
        fut.add_done_callback(lambda f: self.on_goal_accept(f, ep))

    def _make_pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = self.frame_id
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = x
        p.pose.position.y = y
        z, w = yaw_to_quat(yaw)
        p.pose.orientation.z = z
        p.pose.orientation.w = w
        return p

    def on_goal_accept(self, fut, ep):
        if ep != self._epoch:
            return  # 旧路线被抢占, 忽略
        handle = fut.result()
        if not handle.accepted:
            self.fail_route(ep, f'step {self._step_idx} goal rejected')
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self.on_goal_result(f, ep))

    def on_goal_result(self, fut, ep):
        if ep != self._epoch:
            return  # 旧路线(被抢占)的结果, 不推进新路线
        self._goal_handle = None
        status = fut.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._retry_count = 0
            self.advance_step(ep)
            return
        # drive 段失败(动态障碍逼停等): 延时重试本段(重试时走 Theta* 重规划绕障)而非放弃
        if self._steps[self._step_idx][0] == 'drive':
            self._schedule_retry_or_fail(ep, f'status {status}')
        else:
            self.fail_route(ep, f'step {self._step_idx} ended with status {status}')

    def _schedule_retry_or_fail(self, ep, why):
        """drive 段受阻的统一处理: 永不放弃整条路线, 持续重规划绕障直到障碍移开成功推进.
        前 drive_max_retries 次"快重试"(短延时, 快速绕行); 之后转入"慢恢复"(长延时), 一直
        重规划等障碍消失 -> 满足"障碍没了就接着往目标走", 不再永久死停."""
        self._retry_count += 1
        if self._retry_count <= self.drive_max_retries:
            delay = self.drive_retry_delay
            self.get_logger().warn(
                f'step {self._step_idx} {why}, retry '
                f'{self._retry_count}/{self.drive_max_retries} in {delay:.1f}s')
        else:
            delay = self.recovery_retry_delay
            if self._retry_count == self.drive_max_retries + 1:
                self.get_logger().warn(
                    f'step {self._step_idx} {why}: fast retries exhausted -> recovery '
                    f'(replan every {delay:.0f}s until clear, route kept alive)')
            else:
                self.get_logger().warn(
                    f'step {self._step_idx} still blocked ({why}), recovery replan '
                    f'#{self._retry_count - self.drive_max_retries}')
        self._cancel_retry_timer()
        self._retry_timer = self.create_timer(delay, lambda: self._retry_drive(ep))

    def _retry_drive(self, ep):
        # create_timer 是周期定时器, 进回调先取消防重复触发
        self._cancel_retry_timer()
        if ep != self._epoch:
            return
        # 本段被堵: 不再发死直线, 改调全局 planner(NavFn) 从当前位姿重规划一条避障路径到
        # 本段终点(读 global_costmap 自动绕远), 障碍移动 / costmap 更新后每次重试都重算.
        _, p1, heading = self._steps[self._step_idx][1:]
        self.do_replan_drive(p1, heading, ep)

    def do_replan_drive(self, p1, heading, ep):
        if not self._planner_client.wait_for_server(timeout_sec=2.0):
            self.fail_route(ep, 'compute_path_to_pose server not available')
            return
        goal = ComputePathToPose.Goal()
        goal.goal = self._make_pose(p1[0], p1[1], heading)
        goal.use_start = False  # 用机器人当前 TF 作起点
        goal.planner_id = 'GridBased'
        self.get_logger().info(
            f'[step {self._step_idx}] Replan (NavFn) around obstacle '
            f'-> ({p1[0]:.2f},{p1[1]:.2f})')
        fut = self._planner_client.send_goal_async(goal)
        fut.add_done_callback(lambda f: self.on_plan_accept(f, ep))

    def on_plan_accept(self, fut, ep):
        if ep != self._epoch:
            return
        handle = fut.result()
        if not handle.accepted:
            self.fail_route(ep, 'replan goal rejected')
            return
        handle.get_result_async().add_done_callback(lambda f: self.on_plan_result(f, ep))

    def on_plan_result(self, fut, ep):
        if ep != self._epoch:
            return
        res = fut.result()
        if res.status != GoalStatus.STATUS_SUCCEEDED or not res.result.path.poses:
            # 规划失败/空路径(真·完全堵死): 延时再重试, 等障碍移开 / costmap 更新; 到上限才放弃
            self._schedule_retry_or_fail(ep, f'replan failed (status {res.status})')
            return
        path = res.result.path
        # 绕障路径取 NavFn 的位置, 把每个 pose 朝向重写成切线方向: 车头沿绕障路径行进方向,
        # 边平移边缓转跟随(与圆角连续路径同一套朝向策略), 障碍过后自然回到车道.
        self.set_tangent_orientations(path)
        self.get_logger().info(
            f'[step {self._step_idx}] Replan ok: {len(path.poses)} poses, following detour @ tangent')
        self.follow_path(path, ep)

    def _cancel_retry_timer(self):
        if self._retry_timer is not None:
            self._retry_timer.cancel()
            self.destroy_timer(self._retry_timer)
            self._retry_timer = None

    def advance_step(self, ep):
        if ep != self._epoch:
            return
        self._retry_count = 0
        self._step_idx += 1
        self.run_next_step(ep)

    def fail_route(self, ep, why):
        if ep != self._epoch:
            return
        self.get_logger().warn(f'Route to "{self._active_target}" failed: {why}')
        self._cancel_retry_timer()
        self._cancel_cspin_timer()
        self.cmd_pub.publish(Twist())
        self._retry_count = 0
        self._active_target = None
        self._goal_handle = None
        self._steps = []


def main():
    rclpy.init()
    node = LaneNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
