#!/usr/bin/env python3
"""
命名路点导航节点 — 全局规划器出路径 + "先转到路线方向再严格巡路"

`/go_to <name>` -> 从 lane_graph.yaml 的 nodes 表查出目标位姿 -> 调 planner_server
(ComputePathToPose, SmacPlanner2D) 从当前位姿规划一条路径 -> 把每个 pose 的朝向重写成
路径切线 -> 三步跑完:
    ① cspin 闭环自转对齐首段切线
    ② FollowPath 一次跑完整条路径 (MPPI 锁车头跟切线, 拐角不停)
    ③ cspin 闭环自转对齐节点目标 yaw

"严格巡路"靠两件事: 路径 pose 带切线朝向 + MPPI 的 PathAlignCritic(use_path_orientations)
把车头锁在这个朝向上; 配合 vy_max 压到 0.05 禁掉横移, 车只能"先把头转到路线方向再纵向开",
不会斜着平移抄近路。

===== 2026-08-11: 去掉固定路网 =====
上一版走"车道图 + Dijkstra + 顶点倒圆角"。拓扑是纯正交方格网, 于是任何 A->B 都被拆成
一串 90° 直角, 每个直角都要停下转头; 加上车道边是人工押的直线, 与真实空隙的中线并不重合,
窄处(去 place1 那条走廊)常年贴着代价墙走 -> 车走走停停、"犹豫"。用户判定不够流畅, 改成
纯规划: 让规划器读 costmap 自己找连续、居中、拐角平缓的路线。

保留的部分(用户硬需求, 一个都没动):
  - 三步 cspin -> drive -> cspin 状态机与其全部时序纪律(沉降判定/抢占/重试/慢恢复)
  - 切线朝向 + PathAlignCritic 锁头 = "严格巡路"
  - hold_yaw(节点级"末段锁朝向", 见 load_graph)
删掉的部分: nearest_edge / dijkstra / build_rounded_xy / corner_radius / merge_skip_dist,
以及 lane_graph.yaml 的 edges(文件里留着但不再读)。nodes 表退化成纯"命名位姿查找表"。

触发与可视化:
    ros2 topic pub --once /go_to std_msgs/msg/String "{data: pick3}"
    RViz 订阅 /lane_plan (nav_msgs/Path) 看规划出来的路线
"""
import math

import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowPath, ComputePathToPose
from nav2_msgs.srv import ClearEntireCostmap, IsPathValid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
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
        # 到达判定半径(米): 触发的目标若已在此范围内, 直接忽略(不规划不走, 直接报成功)
        # 2026-08-14: 0.2 -> 0.05。0.2 等于"差 19cm 也算到了", 是条精度黑洞: 一旦命中, 车一步
        # 不走、末段 cspin 也不做, 朝向都不对就报 True。防重复触发靠下面 _active_target 判重
        # (on_go_to 里), 从来不靠这个半径, 所以收紧不影响防抖。
        # 留 5cm 而非 0: 完全为零时"重发同一目标"会让已到位的车再规划一次并原地折腾。
        self.declare_parameter('arrival_tolerance', 0.05)
        # 起步朝向对齐(闭环 cspin): 误差 < start_yaw_tol 跳过(残差极小, 交 MPPI 边走边顺, 看不出);
        # 否则用 P 闭环转到首段切线并"沉降"(连续几拍零速且落容差内)再放行 drive -> 保证转停稳了才跑,
        # 杜绝"没转完就跑/起步划弧甩头". 旧版用开环 Nav2 Spin: 报完成时车身还在泄角速度, 同一拍 MPPI
        # 已发前进 -> 残余自转+前进叠加成起步甩头; 闭环+沉降根治. 门限收到 5°(旧 28°)让中等误差也转
        # 到位再走, 不再边跑边扭.
        self.declare_parameter('start_yaw_tol', 0.087)   # ~5°: 起步对齐到位阈值(同时作跳过门限)
        self.declare_parameter('start_wz_max', 0.5)      # 起步转速上限(rad/s): 比终点精对 0.4 略快, 大角度起步不肉但不甩
        # 切线朝向的前视基线(米): 每个 pose 的朝向 = 从它指向"前方至少这么远"的那个点,
        # 而不是相邻两点差分。⚠️ 不能用相邻差分: 全局规划器是栅格搜索(5cm 格), 相邻两点的
        # 方向被量化成 45° 的整数倍, 而这些朝向会被 MPPI 的 PathAlignCritic 当"该点车头指向"
        # 照着锁 -> 车头在直线上也会按 45° 台阶来回扭。前视 0.15m(=3 个栅格)把台阶抹平成
        # 连续切线场, 又远小于最短转弯半径, 不会把弯道方向平均掉。
        self.declare_parameter('heading_baseline', 0.15)
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
        # ===== 倒车脱困(2026-08-13 新增) =====
        # 慢恢复原先**只重新要路径, 从不动车** —— 车楔在墙角时这是个死循环: robot_radius
        # 划出的内切圆压在致命格上, SmacPlanner2D 判起点被占, 同一个注定失败的问题问一万遍
        # 也没用。2026-08-13 实测 place1->place2 那趟连报 75 次 replan failed (status 6),
        # 空转 150s, 第 76 次才因 costmap 自己衰减而突然成功 —— 但已比 S1 的 180s 超时晚
        # 12s, 整条任务序列被判失败。既然唯一失败模式是"起点被占", 唯一解法就是把起点挪出去。
        # 每 escape_after_recovery 次慢恢复插一次"清 costmap + 倒车", 倒完接着走原恢复循环。
        # 倒 -x 是因为那是**车刚开过来的方向**, 按构造必然是空的 (车后方被机械臂挡住,
        # scan_filter 把那一扇滤掉了, 所以这里不可能有观测依据, 只能靠"来路是空的"这个先验)。
        #
        # ⚠️ 为什么不用 Nav2 现成的 BackUp action (behavior_server 就在栈里跑着):
        # nav2_behaviors 每拍都拿 CostmapTopicCollisionChecker::isCollisionFree 校验前方位姿,
        # 而它的判据是 footprint 代价 >= 253 (INSCRIBED_INFLATED_OBSTACLE) 即算碰撞 ——
        # 车已经楔住时 footprint 本来就压着 253 的格子, BackUp 会当场 abort ("Collision
        # Ahead")。**恰恰在最需要它的场景下它拒绝动**, 故这里自己开环倒。
        self.declare_parameter('escape_after_recovery', 3)  # 每几次慢恢复插一次脱困; 0=关
        self.declare_parameter('escape_dist', 0.20)         # 倒车距离(米)
        self.declare_parameter('escape_speed', 0.08)        # 倒车速度(米/秒, 取正值)
        # ===== 卡住看门狗(2026-08-13 新增) =====
        # 背景: drive 段受阻只能靠 FollowPath 的 action 结果不是 SUCCEEDED 来触发上面的
        # 快重试/慢恢复, 而这个结果由 controller_server 的 progress_checker 判定 ——
        # 它的 movement_time_allowance 之前从 3.0 调到 10.0(为了让到点前的低速蠕动不被
        # 误判 ABORT, 见 nav2_params.yaml), 副作用是人往车前一站, 车要在原地"愣" 10s
        # 才等到那个 ABORT, 用户观感是"等待犹豫"。旧路网时代靠 vy 横移绕障"反应快",
        # 现在纯规划 + vy_max 压到 0.1(用户已接受的取舍), 横移躲不动, 只能靠重规划,
        # 而重规划的触发本身太慢。
        # 做法: 不碰 movement_time_allowance(它仍保护到点蠕动这个原用途), 而是在本节点
        # 自己独立监视 drive 段的位移 —— 短时间窗口内几乎没挪动就直接 cancel 当前
        # FollowPath goal。取消后 on_goal_result 收到非 SUCCEEDED, 走的还是原有那套
        # "drive 段失败 -> 快重试 -> 慢恢复"逻辑, 只是触发信号从"等 10s"变成"1s 出头"。
        self.declare_parameter('stuck_check_interval', 0.3)   # 看门狗采样周期(秒)
        self.declare_parameter('stuck_window', 1.2)           # 判定窗口(秒): 这段时间内的位移
        self.declare_parameter('stuck_radius', 0.04)          # 窗口内位移小于它(米)才算"卡住"
        self.declare_parameter('stuck_grace', 1.0)            # drive 起步宽限期(秒): 刚起步加速
                                                               # 阶段位移天然小, 不算卡住
        # 快到目标时豁免看门狗: 交给 nav2 自己的到点判定收尾, 不要在最后蠕动阶段被误判
        # "卡住"抢着 cancel, 那样反而打断即将成功的到点。
        self.declare_parameter('near_goal_skip_radius', 0.15)
        # ===== 路径失效巡检(2026-08-13 新增): "遇见障碍马上规划新路径" =====
        # 上面那个卡住看门狗是**事后**判据 —— 必须先等车真的停住 stuck_window(1.2s) 才反应,
        # 观感仍是"先愣一下再绕"。本巡检是**事前**判据: 直接问 planner_server 当前这条路
        # 还通不通, 人一站进路径就立刻触发重规划, 车不必先被逼停。
        # 单开 Nav2(bt_navigator)时之所以"人一站路径立刻绕开", 正是因为它的行为树里挂着
        # 1Hz 的重规划; lane_navigator 绕过 bt_navigator 直调 planner_server, 就没有这一环,
        # 本节补的就是它。
        # ⚠️ 服务由 planner_server 提供, 名字是相对名解析出来的 /is_path_valid (不是
        #    /planner_server/is_path_valid —— 相对名按**命名空间**解析, 与节点名无关)。
        #    拿不到服务时本巡检自动降级为不启用, 由卡住看门狗兜底, 不影响原有行为。
        self.declare_parameter('path_check_enabled', True)
        self.declare_parameter('path_check_interval', 0.5)   # 巡检周期(秒)
        self.declare_parameter('is_path_valid_service', '/is_path_valid')
        self.arrival_tol = self.get_parameter(
            'arrival_tolerance').get_parameter_value().double_value
        self.start_yaw_tol = self.get_parameter(
            'start_yaw_tol').get_parameter_value().double_value
        self.start_wz_max = self.get_parameter(
            'start_wz_max').get_parameter_value().double_value
        self.heading_baseline = self.get_parameter(
            'heading_baseline').get_parameter_value().double_value
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
        self.escape_after_recovery = self.get_parameter(
            'escape_after_recovery').get_parameter_value().integer_value
        self.escape_dist = self.get_parameter(
            'escape_dist').get_parameter_value().double_value
        self.escape_speed = self.get_parameter(
            'escape_speed').get_parameter_value().double_value
        self.stuck_check_interval = self.get_parameter(
            'stuck_check_interval').get_parameter_value().double_value
        self.stuck_window = self.get_parameter(
            'stuck_window').get_parameter_value().double_value
        self.stuck_radius = self.get_parameter(
            'stuck_radius').get_parameter_value().double_value
        self.stuck_grace = self.get_parameter(
            'stuck_grace').get_parameter_value().double_value
        self.near_goal_skip_radius = self.get_parameter(
            'near_goal_skip_radius').get_parameter_value().double_value
        self.path_check_enabled = self.get_parameter(
            'path_check_enabled').get_parameter_value().bool_value
        self.path_check_interval = self.get_parameter(
            'path_check_interval').get_parameter_value().double_value
        self.is_path_valid_service = self.get_parameter(
            'is_path_valid_service').get_parameter_value().string_value
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
        # 全部命名路点的静态叠加(节点球+名字+朝向箭头): 起栈发一次, latched 让晚开的 RViz 也能拿到.
        self.graph_pub = self.create_publisher(MarkerArray, 'lane_graph_markers', latched)
        # 路由终态回报(供 mm_task 状态机知悉 S1 完成/失败):
        #   "<seq> <target>:SUCCEEDED" / "<seq> <target>:FAILED"   seq 从 1 起单调递增
        #
        # ⚠️ **刻意不 latched**(2026-08-11 改)。终态是**事件**不是状态, 锁存它是反模式:
        # TRANSIENT_LOCAL 会把最后一条终态重放给任何晚订阅者 —— 状态机中途重启而本节点
        # 没重启时, mm_task 一订上就立刻收到上一轮的 "pick1:SUCCEEDED", S1 瞬间假成功、
        # 车压根没动就进 S3 抓取。(mission_manager.stage_nav 进来会清 _nav_status, 故正常
        # 连跑不会踩; 只有"订阅时刻晚于上一次终态"这个窗口会踩。)
        # 前缀 seq 是第二道防线: 订阅方比的是完整字符串, 序号让每次终态都唯一, 即便将来
        # 有人把 QoS 改回 latched, 重放的旧序号也匹配不上本轮期望值。
        self._status_seq = 0
        self.status_pub = self.create_publisher(String, 'lane_navigator/status', 10)
        # cspin(起步/终点闭环对齐)的速度出口。⚠️ 必须发 /cmd_vel_spin 走 twist_mux, 不能直发
        # /cmd_vel: twist_mux 的输出就 remap 在 /cmd_vel 上, 且它在两路输入都超时时**持续发零**。
        # 直发会变成"cspin 的 wz"与"mux 的零"两个发布者在同一话题上交替 -> 固件收到
        # 零/wz/零/wz, 车原地抽搐而不是平稳转 (2026-08-09 跑 pick3 实测, 采样到
        # /cmd_vel angular.z = 0.0/0.0/0.073/0.0)。mux 里 spin 优先级 50, 夹在 nav(10) 与
        # 手柄(100) 之间: 转向时压住 Nav2 的零输出, 手柄按下仍能接管。
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_spin', 10)
        self.go_sub = self.create_subscription(String, 'go_to', self.on_go_to, 10)

        self._follow_client = ActionClient(self, FollowPath, 'follow_path')
        # 路线规划(去路网后这是**唯一**路径来源) + drive 段受阻时的绕障重规划, 同一个客户端。
        self._planner_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self._clear_costmap_client = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        # local 那张也要能清: 慢恢复里堵路的常常是 local costmap 上一帧没衰减掉的残影
        # (2026-08-13 那趟第 76 次重规划突然成功, 就是等它自己衰减等来的)。
        self._clear_local_costmap_client = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        # 路径失效巡检: planner_server 拿当前 global_costmap 校验整条路径是否仍无碰撞
        self._path_valid_client = self.create_client(
            IsPathValid, self.is_path_valid_service)

        # 状态机状态:
        #   _active_target : 当前正在追的目标名(None = 空闲)
        #   _steps         : [('cspin', yaw, tol, wz_max) | ('drive', path, goal_xy, heading)]
        #                    规划是异步的, 故 _steps 在规划结果回调里才填上(此前为空列表)
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
        #   _escape_timer  : 倒车脱困控制定时器(20Hz); 与 _cspin_timer 同一套停车约定
        self._escape_timer = None
        self._escape_deadline = None
        #   _stuck_timer/_stuck_hist/_drive_start_t : 卡住看门狗(见上面参数注释),
        #   只在 drive 步骤在途时跑, goal 结束/抢占时随其他定时器一起清.
        self._stuck_timer = None
        self._stuck_hist = []
        self._drive_start_t = None
        #   _path_timer    : 路径失效巡检定时器, 与卡住看门狗同生命周期(drive 在途时跑)
        #   _cur_path      : 当前正在跟随的路径, 巡检拿它去问 planner 还通不通
        #   _path_check_busy: 上一次 IsPathValid 还没回来时跳过本拍, 防止请求堆积
        #   _proactive_replan: 本次 goal 的终止是"巡检主动 cancel"而非真失败。
        #                    on_goal_result 据此立刻重规划且**不计入重试次数** ——
        #                    动态障碍频繁触发时不该把 drive_max_retries 烧光而掉进慢恢复。
        self._path_timer = None
        self._cur_path = None
        self._path_check_busy = False
        self._proactive_replan = False
        self._path_valid_warned = False

        self.publish_graph_markers()

        self.get_logger().info(
            f'Waypoints loaded: {len(self.nodes)} named poses (routing = global planner). '
            f'Trigger with: ros2 topic pub --once /go_to std_msgs/msg/String "{{data: <node>}}"')

    # ---------- graph ----------
    def load_graph(self, path):
        """读 lane_graph.yaml 的 nodes 段当"命名位姿查找表"。

        ⚠️ edges 段**不再读**(2026-08-11 去路网): 路线由 planner_server 读 costmap 现规划,
        不走固定车道。yaml 里的 edges 保留只为存档/画图参考, 改它对行为没有任何影响。"""
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        self.frame_id = data.get('frame_id', 'map')
        self.nodes = {}
        # hold_yaw: 可选, 单位米。置了则该节点**最后 hold_yaw 米**的路径朝向全部写成节点
        # 目标 yaw, 而不是路径切线 —— 车保持这个朝向平移/倒行进去, 到点即已对齐。
        # 用途: 净距不够原地自转的死头位(外接圆 > 净距), 终点 cspin 若真转会撞墙。
        # 不置(None)则沿用原行为: 朝向 = 切线, 终点靠 cspin 转到目标 yaw。
        self.hold_yaw = {}
        for name, v in data['nodes'].items():
            self.nodes[name] = (float(v['x']), float(v['y']), float(v.get('yaw', 0.0)))
            hy = v.get('hold_yaw')
            self.hold_yaw[name] = None if hy is None else float(hy)

    def publish_graph_markers(self):
        """所有命名路点 -> MarkerArray: 节点球 + 节点名 + 目标朝向箭头.

        任务点(有实际停靠语义的)与转接点(w_*/j_*/c_*, 旧路网的过路点)用颜色区分; 带 hold_yaw
        的画成橙色以提示"该点末段锁朝向、不原地自转"。
        ⚠️ 去路网后不再画边线: 边已经不参与规划, 画出来会让人误以为车沿着它走。"""
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for i, (name, (x, y, yaw)) in enumerate(sorted(self.nodes.items())):
            # 转接点前缀: w_(放货区入口) j_(home 行与南北列的交点) c_(东西廊道的交点).
            # 三类都只过路不停, 其 yaw 是占位值。
            is_transit = name.startswith(('w_', 'j_', 'c_'))
            held = self.hold_yaw.get(name) is not None

            sph = Marker()
            sph.header.frame_id = self.frame_id
            sph.header.stamp = stamp
            sph.ns = 'lane_nodes'
            sph.id = i
            sph.type = Marker.SPHERE
            sph.action = Marker.ADD
            sph.pose.position.x, sph.pose.position.y, sph.pose.position.z = x, y, 0.02
            sph.pose.orientation.w = 1.0
            d = 0.08 if is_transit else 0.13
            sph.scale.x = sph.scale.y = sph.scale.z = d
            if held:
                sph.color.r, sph.color.g, sph.color.b = 1.0, 0.55, 0.0
            elif is_transit:
                sph.color.r, sph.color.g, sph.color.b = 0.6, 0.6, 0.65
            else:
                sph.color.r, sph.color.g, sph.color.b = 0.1, 0.85, 1.0
            sph.color.a = 0.95
            ma.markers.append(sph)

            txt = Marker()
            txt.header.frame_id = self.frame_id
            txt.header.stamp = stamp
            txt.ns = 'lane_labels'
            txt.id = i
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = x, y, 0.22
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.13
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            txt.text = f'{name}*' if held else name
            ma.markers.append(txt)

            # 转接点没有停靠语义, 其 yaw 只是占位, 画箭头会误导
            if is_transit:
                continue
            arw = Marker()
            arw.header.frame_id = self.frame_id
            arw.header.stamp = stamp
            arw.ns = 'lane_yaw'
            arw.id = i
            arw.type = Marker.ARROW
            arw.action = Marker.ADD
            arw.pose.position.x, arw.pose.position.y, arw.pose.position.z = x, y, 0.02
            z, w = yaw_to_quat(yaw)
            arw.pose.orientation.z, arw.pose.orientation.w = z, w
            arw.scale.x, arw.scale.y, arw.scale.z = 0.28, 0.045, 0.045
            arw.color.r, arw.color.g, arw.color.b, arw.color.a = 1.0, 0.9, 0.1, 0.9
            ma.markers.append(arw)

        self.graph_pub.publish(ma)
        self.get_logger().info(f'Published lane graph markers: {len(ma.markers)} markers')

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
    def _end_heading(self, xy, at_start, baseline=None):
        """路径首端/末端的行进方向, 用"跨过 baseline 米的位移"算而非相邻两点差分。

        ⚠️ 不能只取 xy[0]->xy[1]: 栅格规划器的相邻点方向被量化成 45° 整数倍, 且路径头尾
        常出现间距近于零的重合点, 两点差分的方向于是被量化台阶和浮点噪声主导。
        2026-08-10 跑 pick3 实测: 目标在正前方偏右 8°, 而 xy[0]->xy[1] 算出 -143deg,
        起步 cspin 照着它把车掉头转了大半圈再倒着开过去。
        """
        if baseline is None:
            baseline = self.heading_baseline
        if len(xy) < 2:
            return 0.0
        if at_start:
            p0 = xy[0]
            for p in xy[1:]:
                if math.hypot(p[0] - p0[0], p[1] - p0[1]) >= baseline:
                    return math.atan2(p[1] - p0[1], p[0] - p0[0])
            p1 = xy[-1]     # 整条路径都比 baseline 短: 退化成首末连线
        else:
            p1 = xy[-1]
            for p in reversed(xy[:-1]):
                if math.hypot(p1[0] - p[0], p1[1] - p[1]) >= baseline:
                    return math.atan2(p1[1] - p[1], p1[0] - p[0])
            p0 = xy[0]
            return math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        return math.atan2(p1[1] - p0[1], p1[0] - p0[0])

    def set_tangent_orientations(self, path):
        """就地把规划器路径的每个 pose 朝向重写成"前视 heading_baseline 米"的切线方向。

        这些朝向就是 MPPI 的 PathAlignCritic(use_path_orientations) 锁车头用的目标 ——
        即"严格巡路"里"车头必须对着路线方向"的那一半(另一半是 vy_max=0.05 禁横移)。

        ⚠️ 用前视窗口而不是相邻两点差分: 全局规划器是 5cm 栅格搜索, 相邻两点的连线方向
        只能取 45° 的整数倍, 照抄就等于命令车头在直线段上按 45° 台阶来回扭。前视 0.15m
        (3 个格)把台阶平均成连续切线场, 又短于任何一个弯, 不会把弯道方向抹平。
        尾部不足 baseline 的点沿用最后一个有效朝向 = 终点进场方向, 正确。
        """
        poses = path.poses
        n = len(poses)
        if n == 0:
            return
        xs = [p.pose.position.x for p in poses]
        ys = [p.pose.position.y for p in poses]
        # j 单调右移: 对每个 k 找第一个距 k 至少 baseline 的点(k 增大时 j 不回退)
        heads = [0.0] * n
        last = math.atan2(ys[-1] - ys[0], xs[-1] - xs[0]) if n > 1 else 0.0
        j = 0
        for k in range(n):
            j = max(j, k + 1)
            while j < n and math.hypot(xs[j] - xs[k], ys[j] - ys[k]) < self.heading_baseline:
                j += 1
            if j < n:
                last = math.atan2(ys[j] - ys[k], xs[j] - xs[k])
            heads[k] = last
        for k in range(n):
            z, w = yaw_to_quat(heads[k])
            poses[k].pose.orientation.x = 0.0
            poses[k].pose.orientation.y = 0.0
            poses[k].pose.orientation.z = z
            poses[k].pose.orientation.w = w

    def _rewrite_tail_orientations(self, path, hold_meters, target_yaw):
        """把 path 末尾 hold_meters 米内所有 pose 的朝向改写成 target_yaw, 返回锁定段起始下标.

        用于"死头位": 终点净距不够原地自转(外接圆 > 净距), 不能靠终点 cspin 转朝向, 只能
        让车提前转好、靠全向底盘平移/倒行进去. MPPI 的 PathAlignCritic(use_path_orientations)
        读的就是这些 pose 朝向, 于是末段车头被锁死在 target_yaw, 到点时 yaw 已经对了.
        返回 0 表示锁定段覆盖整条路径(调用方据此把起步 cspin 也对齐 target_yaw)."""
        poses = path.poses
        # 从末点往前累距, 找到"距终点 >= hold_meters"的那一点, 它之后全部锁朝向
        acc = 0.0
        idx = 0
        for k in range(len(poses) - 1, 0, -1):
            acc += math.hypot(
                poses[k].pose.position.x - poses[k - 1].pose.position.x,
                poses[k].pose.position.y - poses[k - 1].pose.position.y)
            if acc >= hold_meters:
                idx = k - 1
                break
        z, w = yaw_to_quat(target_yaw)
        for k in range(idx, len(poses)):
            poses[k].pose.orientation.x = 0.0
            poses[k].pose.orientation.y = 0.0
            poses[k].pose.orientation.z = z
            poses[k].pose.orientation.w = w
        self.get_logger().info(
            f'Hold yaw {math.degrees(target_yaw):.0f}deg over last {hold_meters:.2f}m '
            f'(poses {idx}..{len(poses) - 1} of {len(poses)})')
        return idx

    # ---------- status ----------
    def publish_status(self, target, ok):
        """发一条终态回报 "<seq> <target>:SUCCEEDED|FAILED" (seq 单调递增, 见构造函数注释)."""
        self._status_seq += 1
        verdict = 'SUCCEEDED' if ok else 'FAILED'
        self.status_pub.publish(String(data=f'{self._status_seq} {target}:{verdict}'))

    # ---------- trigger ----------
    def on_go_to(self, msg):
        target = msg.data.strip()
        if target not in self.nodes:
            self.get_logger().error(f'Unknown node "{target}". Known: {list(self.nodes)}')
            self.publish_status(target, False)
            return
        pose = self.get_robot_pose()
        if pose is None:
            self.publish_status(target, False)
            return
        px, py, _ = pose

        # 已在目标点附近 -> 直接回报到位(状态机据此放行, 不再干等)
        tx, ty, _ = self.nodes[target]
        d_goal = math.hypot(px - tx, py - ty)
        if d_goal <= self.arrival_tol:
            self.get_logger().info(f'Already at "{target}" (dist={d_goal:.2f}m), ignoring')
            self.publish_status(target, True)
            return

        # 相同目标且仍在执行 -> 忽略重复触发(防 ros2 topic pub -r 连发抖动)
        if self._active_target == target:
            self.get_logger().info(f'Target "{target}" already in progress, ignoring duplicate')
            return

        # 新路线: epoch+1 并立刻抢占在途的一切(旧回调凭旧 epoch 自动失效)。
        # ⚠️ 抢占必须发生在**发规划请求之前**: 规划是异步的, 期间若不先把旧 goal/定时器
        # 掐掉, 旧路线会继续开着车跑到新路径回来为止。
        self._epoch += 1
        ep = self._epoch
        if self._goal_handle is not None:
            self.get_logger().info(f'Preempting current route for new target "{target}"')
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._cancel_retry_timer()
        self._cancel_stuck_timer()
        self._cancel_path_timer()
        self._proactive_replan = False   # 抢占后旧旗作废, 否则会污染新路线的首个结果
        if self._cspin_timer is not None:  # 抢占在途闭环对齐: 停转并清定时器
            self._cancel_cspin_timer()
            self.cmd_pub.publish(Twist())
        if self._escape_timer is not None:  # 抢占在途倒车: 同上, 停车再清
            self._cancel_escape_timer()
            self.cmd_pub.publish(Twist())
        self._retry_count = 0
        self._active_target = target
        self._steps = []       # 规划还没回来, 保持空; run_next_step 此刻不能调
        self._step_idx = 0
        self.request_route_plan(target, ep)

    def request_route_plan(self, target, ep):
        """向 planner_server 要一条 当前位姿 -> 目标节点 的路径(去路网后的唯一路径来源)."""
        if not self._planner_client.wait_for_server(timeout_sec=2.0):
            self.fail_route(ep, 'compute_path_to_pose server not available')
            return
        tx, ty, tyaw = self.nodes[target]
        goal = ComputePathToPose.Goal()
        goal.goal = self._make_pose(tx, ty, tyaw)
        goal.use_start = False        # 起点用机器人当前 TF
        goal.planner_id = 'GridBased'
        self.get_logger().info(f'Planning route -> "{target}" ({tx:.2f},{ty:.2f})')
        self._clear_costmaps()
        fut = self._planner_client.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._on_route_plan_accept(f, ep, target))

    def _on_route_plan_accept(self, fut, ep, target):
        if ep != self._epoch:
            return
        handle = fut.result()
        if not handle.accepted:
            self.fail_route(ep, 'route plan goal rejected')
            return
        handle.get_result_async().add_done_callback(
            lambda f: self._on_route_plan_result(f, ep, target))

    def _on_route_plan_result(self, fut, ep, target):
        if ep != self._epoch:
            return
        res = fut.result()
        if res.status != GoalStatus.STATUS_SUCCEEDED or len(res.result.path.poses) < 2:
            self.fail_route(ep, f'no path to "{target}" (planner status {res.status})')
            return
        self.start_route(target, res.result.path, ep)

    # ---------- 状态机 ----------
    def start_route(self, target, path, ep):
        """规划器路径 -> 三步路线 [起步对齐切线, 跑完整条路径, 终点对齐目标 yaw].

        起步用闭环 cspin 对齐首段切线(start_yaw_tol~5°): P 闭环转到位并"沉降"(连续几拍零速
        且落容差内)再放行 drive -> 转停稳了才跑, 不会没转完就被 MPPI 前进抢走而起步甩头。
        整条路径当**一个** drive 步骤, 故拐角不是 goal、不会被 xy_goal_tolerance 提前判完成
        -> 拐角不停车。终点同样用闭环 cspin(final_yaw_tol~1°, 紧而稳): 读 TF 真实 yaw 做 P
        控制, 过冲自动反向修回(Nav2 的开环 Spin 会被底盘加速度斜坡滑过)。位置精度交 MPPI
        drive 段的 xy_goal_tolerance。"""
        if ep != self._epoch:
            return
        path.header.frame_id = self.frame_id
        # 规划器只给位置(其 pose 朝向对全向底盘无意义), 朝向由这里按前视切线重写 ——
        # 这就是 MPPI 锁车头的依据, "严格巡路"由此成立。
        self.set_tangent_orientations(path)
        xy = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        target_yaw = self.nodes[target][2]
        # hold_yaw: 末段锁定朝向(见 load_graph 注释)。把最后 hold_yaw 米的 pose 朝向全部
        # 改写成目标 yaw, 于是 MPPI 的 PathAlignCritic(use_path_orientations) 把车头锁在
        # 这个朝向, 车靠全向底盘平移/倒行走完末段 -> 到点时 yaw 已经对了, 终点 cspin 的
        # "已对齐则跳过"分支直接放行, 压根不转。
        # 起步 cspin 也随之改成对齐这个朝向(而非首段切线): 否则车会先转到切线跑, 进了窄道
        # 才发现要改朝向, 而那里正是转不了的地方。
        hold = self.hold_yaw.get(target)
        held_from = None
        if hold is not None and hold > 0.0:
            held_from = self._rewrite_tail_orientations(path, hold, target_yaw)
        if held_from == 0:
            # 锁定段覆盖了整条路径 -> 全程保持目标朝向, 起步就转到它
            first_heading = target_yaw
        else:
            first_heading = self._end_heading(xy, at_start=True)
        last_heading = target_yaw if held_from is not None else \
            self._end_heading(xy, at_start=False)
        # drive 受阻重规划时的目标用**节点真值**而不是 path 末点: 规划器允许在 tolerance 内
        # 收敛到附近格子, 拿它当重规划终点会让误差一轮轮累积着往外爬。
        goal_xy = (self.nodes[target][0], self.nodes[target][1])
        self._steps = [
            ('cspin', first_heading, self.start_yaw_tol, self.start_wz_max),
            ('drive', path, goal_xy, last_heading),
            ('cspin', target_yaw, self.final_yaw_tol, self.cspin_wz_max),
        ]
        self._step_idx = 0
        self.plan_pub.publish(path)
        self.get_logger().info(
            f'Route "{target}": {len(path.poses)} poses, '
            f'start heading {math.degrees(first_heading):.0f}deg')
        self.run_next_step(ep)

    def run_next_step(self, ep):
        if ep != self._epoch:
            return
        if self._step_idx >= len(self._steps):
            self.get_logger().info(f'Route to "{self._active_target}" complete')
            self.publish_status(self._active_target, True)
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
        self._cur_path = path          # 供路径失效巡检使用
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
        # 只有 drive 步骤会走到这里且需要看门狗(cspin 靠自己的定时器控速, 不发 FollowPath).
        if self._steps[self._step_idx][0] == 'drive':
            self._start_stuck_watchdog(ep)
            self._start_path_watchdog(ep)
        handle.get_result_async().add_done_callback(lambda f: self.on_goal_result(f, ep))

    def on_goal_result(self, fut, ep):
        self._cancel_stuck_timer()  # goal 已终结(成功/取消/中止), 看门狗跟着下岗
        self._cancel_path_timer()
        if ep != self._epoch:
            return  # 旧路线(被抢占)的结果, 不推进新路线
        self._goal_handle = None
        status = fut.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._retry_count = 0
            self.advance_step(ep)
            return
        # 巡检发现路径被挡而主动 cancel: 立刻重规划绕行, 不走延时重试、不计重试次数。
        # (真失败才该消耗重试预算; 主动重规划是正常绕障, 消耗它会让人在旁边走动几次就
        #  掉进 2s 一次的慢恢复, 车变得很迟钝。)
        if self._proactive_replan:
            self._proactive_replan = False
            if self._steps[self._step_idx][0] == 'drive':
                self._retry_drive(ep)
                return
        # drive 段失败(动态障碍逼停等): 延时重试本段(重取当前位姿重规划绕障)而非放弃
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
            rec_n = self._retry_count - self.drive_max_retries
            if self._retry_count == self.drive_max_retries + 1:
                self.get_logger().warn(
                    f'step {self._step_idx} {why}: fast retries exhausted -> recovery '
                    f'(replan every {delay:.0f}s until clear, route kept alive)')
            else:
                self.get_logger().warn(
                    f'step {self._step_idx} still blocked ({why}), recovery replan #{rec_n}')
            # 恢复到第 N 的整数倍次仍不通 -> 不再干问, 清 costmap + 倒车把起点挪出致命区
            if self.escape_after_recovery > 0 and rec_n % self.escape_after_recovery == 0:
                self._cancel_retry_timer()
                self._retry_timer = self.create_timer(
                    delay, lambda: self._escape_then_retry(ep))
                return
        self._cancel_retry_timer()
        self._retry_timer = self.create_timer(delay, lambda: self._retry_drive(ep))

    def _clear_costmaps(self):
        """两张 costmap 一起清. 服务没就绪就跳过(不阻塞恢复循环)."""
        for cli in (self._clear_costmap_client, self._clear_local_costmap_client):
            if cli.service_is_ready():
                cli.call_async(ClearEntireCostmap.Request())

    def _escape_then_retry(self, ep):
        """脱困: 清两张 costmap, 再开环倒 escape_dist 米, 倒完接回原恢复循环重规划.
        开环(按时长积分)而不是闭环查位移: 楔住时轮子可能在打滑, 位移判据会永远不满足;
        倒车本身有 escape_dist/escape_speed 的硬时长上限, 不会跑飞."""
        self._cancel_retry_timer()
        if ep != self._epoch:
            return
        self._clear_costmaps()
        dur = self.escape_dist / max(self.escape_speed, 1e-3)
        self.get_logger().warn(
            f'[step {self._step_idx}] 恢复无进展 -> 清 costmap + 倒车 '
            f'{self.escape_dist:.2f}m 脱困 ({dur:.1f}s)')
        self._escape_deadline = self.get_clock().now() + Duration(seconds=dur)
        self._cancel_escape_timer()
        self._escape_timer = self.create_timer(0.05, lambda: self._escape_tick(ep))

    def _escape_tick(self, ep):
        if ep != self._epoch:
            self._cancel_escape_timer()
            self.cmd_pub.publish(Twist())
            return
        if self.get_clock().now() >= self._escape_deadline:
            self._cancel_escape_timer()
            self.cmd_pub.publish(Twist())
            self._retry_drive(ep)
            return
        cmd = Twist()
        cmd.linear.x = -abs(self.escape_speed)
        self.cmd_pub.publish(cmd)

    def _cancel_escape_timer(self):
        if self._escape_timer is not None:
            self._escape_timer.cancel()
            self.destroy_timer(self._escape_timer)
            self._escape_timer = None

    def _retry_drive(self, ep):
        # create_timer 是周期定时器, 进回调先取消防重复触发
        self._cancel_retry_timer()
        if ep != self._epoch:
            return
        # 本段被堵: 从**当前**位姿向全局 planner 重新要一条到目标的路径(读 global_costmap
        # 自动绕远)。障碍移动 / costmap 更新后每次重试都重算, 故障碍挪开就自动接着走。
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
        # 每次绕障重规划前清一次 costmap (原先只有首次规划 request_route_plan 里清)。
        # 慢恢复里堵路的常是残影 —— 人走过留下的痕迹、AMCL 跳变把墙抹粗一圈、机械臂/货箱
        # 的回波。不清就只能干等它自己衰减 (2026-08-13 那趟等了 150s)。
        self._clear_costmaps()
        self.get_logger().info(
            f'[step {self._step_idx}] Replan around obstacle '
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
        # 与首次规划同一套处理: 位置用规划器的, 朝向按前视切线重写 -> 车头始终对着行进方向.
        self.set_tangent_orientations(path)
        self.get_logger().info(
            f'[step {self._step_idx}] Replan ok: {len(path.poses)} poses, following detour @ tangent')
        self.follow_path(path, ep)

    def _cancel_retry_timer(self):
        if self._retry_timer is not None:
            self._retry_timer.cancel()
            self.destroy_timer(self._retry_timer)
            self._retry_timer = None

    def _start_stuck_watchdog(self, ep):
        """drive 段 goal 一被接受就起这个表, 独立于 nav2 自己的 progress_checker
        判定"卡住"并主动 cancel, 让重规划不必等 controller 内部 10s 才反应.
        cancel 之后走的还是 on_goal_result 里原有的失败分支(快重试/慢恢复), 这里
        不重复那套逻辑, 只负责"更快地产生一次失败结果"。"""
        self._cancel_stuck_timer()
        self._stuck_hist = []
        self._drive_start_t = self.get_clock().now()
        self._stuck_timer = self.create_timer(
            self.stuck_check_interval, lambda: self._stuck_tick(ep))

    def _stuck_tick(self, ep):
        if ep != self._epoch or self._goal_handle is None:
            self._cancel_stuck_timer()
            return
        pose = self.get_robot_pose()
        if pose is None:
            return
        now = self.get_clock().now()
        px, py, _ = pose
        # drive 步骤存的是 ('drive', path, goal_xy, last_heading)
        goal_xy = self._steps[self._step_idx][2]
        if math.hypot(px - goal_xy[0], py - goal_xy[1]) < self.near_goal_skip_radius:
            self._stuck_hist = []  # 快到目标: 交给 nav2 自己的到点判定, 不掺和
            return
        if (now - self._drive_start_t).nanoseconds / 1e9 < self.stuck_grace:
            return  # 起步加速阶段, 位移天然小, 还在宽限期内
        self._stuck_hist.append((now, px, py))
        while self._stuck_hist and \
                (now - self._stuck_hist[0][0]).nanoseconds / 1e9 > self.stuck_window:
            self._stuck_hist.pop(0)
        if len(self._stuck_hist) < 2:
            return
        t0, x0, y0 = self._stuck_hist[0]
        span = (now - t0).nanoseconds / 1e9
        if span < self.stuck_window * 0.8:
            return  # 窗口还没攒够, 再等几拍
        if math.hypot(px - x0, py - y0) < self.stuck_radius:
            self.get_logger().warn(
                f'[step {self._step_idx}] Blocked (moved <{self.stuck_radius * 100:.0f}cm '
                f'in {span:.1f}s) -> cancel now & replan, not waiting for nav2 progress_checker')
            self._cancel_stuck_timer()
            handle = self._goal_handle
            if handle is not None:
                handle.cancel_goal_async()

    def _cancel_stuck_timer(self):
        if self._stuck_timer is not None:
            self._stuck_timer.cancel()
            self.destroy_timer(self._stuck_timer)
            self._stuck_timer = None

    # ---------- 路径失效巡检(事前判据, 见 __init__ 里 path_check_* 参数注释) ----------
    def _start_path_watchdog(self, ep):
        """drive goal 被接受后起表, 周期性问 planner_server 当前这条路还通不通。
        服务不可用时静默降级(只 warn 一次), 由卡住看门狗兜底 —— 宁可慢一点, 不要因为
        少一个可选服务就让整条路线走不了。"""
        self._cancel_path_timer()
        if not self.path_check_enabled:
            return
        if not self._path_valid_client.service_is_ready():
            if not self._path_valid_warned:
                self._path_valid_warned = True
                self.get_logger().warn(
                    f'{self.is_path_valid_service} 不可用 -> 主动绕障重规划关闭, '
                    f'退化为仅靠卡住看门狗(反应慢约 1.2s)')
            return
        self._path_timer = self.create_timer(
            self.path_check_interval, lambda: self._path_check_tick(ep))

    def _path_check_tick(self, ep):
        if ep != self._epoch or self._goal_handle is None:
            self._cancel_path_timer()
            return
        # 上一次请求还没回来就跳过本拍: 服务偶发变慢时不堆积请求
        if self._path_check_busy or self._cur_path is None or not self._cur_path.poses:
            return
        pose = self.get_robot_pose()
        if pose is None:
            return
        px, py, _ = pose
        # drive 步骤存的是 ('drive', path, goal_xy, last_heading)
        goal_xy = self._steps[self._step_idx][2]
        if math.hypot(px - goal_xy[0], py - goal_xy[1]) < self.near_goal_skip_radius:
            return  # 与卡住看门狗同一纪律: 末段交给 nav2 自己的到点判定收尾
        ahead = self._path_ahead(px, py)
        if len(ahead.poses) < 2:
            return
        self._path_check_busy = True
        req = IsPathValid.Request()
        req.path = ahead
        self._path_valid_client.call_async(req).add_done_callback(
            lambda f: self._on_path_valid(f, ep))

    def _path_ahead(self, px, py):
        """截出机器人**前方**那一段路径。
        ⚠️ 必须截: IsPathValid 校验的是传进去的整条路径, 若把已走过的那段也带上, 人站到
        车**身后**的路径上同样会判无效 -> 车为身后的障碍反复重规划, 永远走不到目标。"""
        poses = self._cur_path.poses
        best_i, best_d = 0, float('inf')
        for i, ps in enumerate(poses):
            d = math.hypot(ps.pose.position.x - px, ps.pose.position.y - py)
            if d < best_d:
                best_d, best_i = d, i
        ahead = Path()
        ahead.header = self._cur_path.header
        ahead.poses = poses[best_i:]
        return ahead

    def _on_path_valid(self, fut, ep):
        self._path_check_busy = False
        if ep != self._epoch or self._goal_handle is None:
            return
        try:
            valid = fut.result().is_valid
        except Exception as e:  # 服务异常不该拖垮路线, 交给卡住看门狗
            self.get_logger().warn(f'is_path_valid 调用失败: {e}')
            return
        if valid:
            return
        self.get_logger().warn(
            f'[step {self._step_idx}] 路径被挡(is_path_valid=false) -> 立即重规划绕行')
        # 先立旗再 cancel: on_goal_result 靠它区分"主动绕障"与"真失败"
        self._proactive_replan = True
        self._cancel_path_timer()
        self._cancel_stuck_timer()
        handle = self._goal_handle
        if handle is not None:
            handle.cancel_goal_async()

    def _cancel_path_timer(self):
        if self._path_timer is not None:
            self._path_timer.cancel()
            self.destroy_timer(self._path_timer)
            self._path_timer = None
        self._path_check_busy = False

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
        self.publish_status(self._active_target, False)
        self._cancel_retry_timer()
        self._cancel_stuck_timer()
        self._cancel_path_timer()
        self._cancel_cspin_timer()
        self._cancel_escape_timer()
        self.cmd_pub.publish(Twist())
        self._retry_count = 0
        self._proactive_replan = False
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
