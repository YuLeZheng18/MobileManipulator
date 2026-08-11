"""实机总 launch (real-only) — 一键起真机全栈 (架构 §7 自底向上六段).

与仿真的唯一区别在最底层硬件抽象 (谁产 /odom、谁吃 /cmd_vel); 其上 TF/话题/
服务/action 与仿真完全一致, 上层节点 (Nav2/MoveIt/grasp/mm_task) sim/real 无感 (架构 §4)。
故本 launch 不起 Gazebo/mock, 改起: micro-ROS 代理 + CAN 桥 + 真雷达/相机 + EKF 状态估计。
无头 (§7-E): 不起 RViz。

分阶段错峰 (TimerAction, 上层等下层就绪):
  t=0   micro-ROS 代理(底盘) + 机械臂实机 (RSP hw:=real + ros2_control + CAN 桥) + 雷达/相机
  t=5   robot_localization EKF: 融合 /wheel_odom + /imu -> /odom + odom->base_link TF
  t=10  Nav2 (无 RViz, 自带 velocity_smoother 限幅) + twist_mux 仲裁 + lane_navigator
  t=14  MoveIt move_group
  t=18  moveit_servo + grasp_node
  t=20  mm_perception 真感知 (yolo_box_detector + aruco_localizer; 默认关)
  t=25  mm_task 状态机 (默认关; run_mission:=true 自动跑 S0->S5)

抓取相关的开关连带关系 (三者要一起开, 少一个抓取跑不了):
  use_cameras:=true      -> 车体两路 USB + 手眼 D435i (align_depth 必开, 在 cameras.launch.py 里)
  use_perception:=true   -> yolo_box_detector 吃 D435i 出 /perception/object_pose 等
  run_mission:=true      -> mm_task 串起 S0-S5 (真机记得配 mission_file:=.../mission_real.yaml)
整机自主一条命令:
  ros2 launch mm_bringup real_bringup.launch.py use_cameras:=true use_perception:=true \
      run_mission:=true mission_file:=<mm_task share>/config/mission_real.yaml

⚠️ 运行前置 (目标机 Nano 上现装/现 source, 不进本 colcon ws):
  - micro_ros_agent: 需先 `source ~/microros_ws/install/setup.bash` 再起本 launch,
    否则找不到 micro_ros_agent 可执行 (代理是独立 ws, 见架构 §5.2 记忆)。
  - rplidar_ros / realsense2_camera / usb_cam / robot_localization: apt 装在 Nano 上。
  - CAN: 机械臂 can_bridge 需 CAN 接口已 up (ip link set can0 up ...)。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def _include(pkg, rel, **kwargs):
    # 同 sim_bringup: GroupAction 作用域隔离被包含 launch 的 DeclareLaunchArgument,
    # 防止兄弟 include 间配置泄漏 (scoped=True 默认)。
    path = os.path.join(get_package_share_directory(pkg), 'launch', rel)
    inc = IncludeLaunchDescription(PythonLaunchDescriptionSource(path), **kwargs)
    return GroupAction([inc])


def generate_launch_description():
    # 实机无 /clock, use_sim_time 恒 false (可覆盖但默认即真机正确值)
    use_sim_time = LaunchConfiguration('use_sim_time')
    run_mission = LaunchConfiguration('run_mission')
    use_lidar = LaunchConfiguration('use_lidar')
    use_nav2 = LaunchConfiguration('use_nav2')
    use_cameras = LaunchConfiguration('use_cameras')
    use_perception = LaunchConfiguration('use_perception')
    agent_serial_dev = LaunchConfiguration('agent_serial_dev')
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')
    mission_file = LaunchConfiguration('mission_file')
    map_yaml = LaunchConfiguration('map')
    nav2_params = LaunchConfiguration('params_file')

    mm_nav_share = get_package_share_directory('mm_navigation')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='实机无 /clock, 恒 false'),
        DeclareLaunchArgument('run_mission', default_value='false',
                              description='true=起栈后自动跑 mm_task 状态机; false=只起栈'),
        DeclareLaunchArgument('use_lidar', default_value='true',
                              description='起 rplidar_ros (思岚 A3 -> /scan, frame laser_link) '
                                          '+ scan_filter (-> /scan_filtered)'),
        DeclareLaunchArgument('use_nav2', default_value='true',
                              description='起 Nav2 全套 (map_server/AMCL/controller/...). '
                                          '建图时必须 false: AMCL 与 slam_toolbox 会抢发 '
                                          'map->odom TF, 两个源同时发 TF 树跳变, 地图必乱'),
        DeclareLaunchArgument('use_cameras', default_value='false',
                              description='起相机驱动: 车体两路 USB (cam_a/cam_b) + 手眼 D435i'),
        DeclareLaunchArgument('use_perception', default_value='false',
                              description='起 mm_perception 真感知 (yolo_box_detector + aruco); '
                                          '需连带 use_cameras:=true'),
        DeclareLaunchArgument(
            'mission_file', default_value='',
            description='mm_task 任务列表 (留空用包内 mission.yaml=仿真值; '
                        '真机传 config/mission_real.yaml)'),
        DeclareLaunchArgument('agent_serial_dev', default_value='/dev/ttyACM0',
                              description='micro-ROS 代理串口 (ESP32-S3 native USB)'),
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/rplidar',
                              description='思岚 A3 雷达串口 (经 CP2102 USB-TTL; udev 软链 /dev/rplidar, 见 99-rplidar.rules; 波特 256000)'),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(mm_nav_share, 'maps', 'room_real.yaml'),
            description='Nav2 地图 (真机实测图, 2026-08-08 slam_toolbox 建于卧室; '
                        '仿真图是 room.yaml, 本 launch 是 real-only 故不该拿它当默认)'),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(mm_nav_share, 'config', 'nav2_params.yaml'),
            description='Nav2 参数 (sim/real 共用, 底盘运动学一致故可移植)'),
    ]

    real_arg = {'use_sim_time': use_sim_time}.items()

    # ===== 阶段1 (t=0): 底盘 micro-ROS 代理 + 机械臂实机 + 雷达/相机 =====

    # micro-ROS 代理: 桥接 ESP32-S3 底盘固件 (node=chassis_driver, best_effort)。
    # 话题名对齐纪律 (架构 §5.2): 固件已直接发布 /wheel_odom (轮式里程计原始量, 仅话题不发 TF)
    #   与 /imu, 订阅 /cmd_vel; /odom 名字留给上位机 EKF 输出独占。故代理无需重映射。
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', agent_serial_dev],
    )

    # 机械臂实机: RSP(arm_description.urdf.xacro hw:=real, 内含整车 mm_robot.urdf 几何)
    #   + ros2_control_node + JTC + arm_control/can_bridge (0xFD CAN)。
    # 整车 robot_description 由这里的 RSP 唯一发布 (仿真里由 gazebo 发, 真机由此发)。
    arm_real = _include('arm_moveit_config', 'real_bringup.launch.py')

    # 雷达: 思岚 A3 -> rplidar_ros -> /scan (frame_id=laser_link, 与仿真/URDF 一致, 架构 §5.1)
    # ⚠️ frame_id 是 laser_link 不是 Link_12: 前者是**测量帧**, 后者是 mesh 帧, 两者差一个
    #    Rz(pi) —— 真机雷达绕竖轴转了 180° 装。指成 Link_12 会让点云前后颠倒
    #    (2026-08-08 实测三次挡测定案, 由来见 mm_robot.urdf 的 Joint_laser 注释)。
    lidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar',
        output='screen',
        parameters=[{
            'serial_port': lidar_serial_port,
            'serial_baudrate': 256000,   # A3
            'frame_id': 'laser_link',
            'scan_mode': 'Standard',
            'inverted': False,
            # ⚠️ angle_compensate 必须 True(与 lidar_test.launch.py 对齐):
            # False 时每圈点数随转速浮动(实测 285/286/287 三种), 而 slam_toolbox 在
            # 注册传感器时按首帧点数固定, 之后帧帧报
            # "LaserRangeScan contains 288 range readings, expected 287" 并最终**进程退出**
            # —— 表现是"地图不再刷新"而车还在跑。2026-08-08 建图第一次尝试因此失败。
            'angle_compensate': True,
        }],
        condition=IfCondition(use_lidar),
    )

    # 自遮挡滤除: /scan -> /scan_filtered。车后方被机械臂本体与线材挡住, 那段测距是"车自己"。
    # 下游 (AMCL / 两张 costmap / 建图) 全部用 /scan_filtered, 见 scan_filter.yaml。
    # ⚠️ 必须走 _include (GroupAction 隔离作用域) 且**显式传 params_file**:
    # 本 launch 自己有个 params_file 参数(给 Nav2 的 nav2_params.yaml), 裸 include 会让它
    # 泄漏进去覆盖 scan_filter 的默认值 -> filter chain 加载 nav2_params.yaml, 里面没有
    # scan_filter_chain 键 ⇒ **一个 filter 都不装, /scan_filtered 原样透传**, 而节点照常
    # 运行不报错。后果: 机械臂被建进地图成幽灵墙。2026-08-08 首次起栈时踩到。
    scan_filter = _include(
        'mm_navigation', 'scan_filter.launch.py',
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(mm_nav_share, 'config', 'scan_filter.yaml'),
        }.items(),
        condition=IfCondition(use_lidar),
    )

    # 车体两路 USB 相机 (use_cameras:=true 时起), 每路只起 usb_cam 发 image_raw:
    #   cam_a (Link_13, ArUco): usb口2.2.2, 装反   cam_b (Link_14, 监视): usb口2.2.3, 装反
    # ⚠️ 这里**不做转正** (原先的 image_rotator 链 2026-08-03 已删)。看画面靠
    # web_video_server 的 invert=1 服务端转正; ArUco 用 aruco_real.launch.py 自带的
    # image_rotator。详见 cameras.launch.py 文件头。
    cameras = _include('mm_perception', 'cameras.launch.py',
                       condition=IfCondition(use_cameras))

    stage1 = [micro_ros_agent, arm_real, lidar, scan_filter, cameras]

    # ===== 阶段2 (t=5): EKF 状态估计 =====
    # robot_localization 融合 /wheel_odom + /imu -> /odom + odom->base_link TF。
    # 纪律: odom->base_link 这段 TF 只有 EKF 能发, 固件绝不发 TF (架构 §5.2)。
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('mm_bringup'), 'config', 'ekf.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        # ⚠️ robot_localization 硬编码发到 /odometry/filtered, 而全栈约定是 /odom
        # (nav2_params.yaml:47 controller_server 与 :401 velocity_smoother 都订 /odom,
        #  仿真侧 planar_move 也直接发 /odom)。这个 remap 缺了则 Nav2 拿不到里程计,
        # 表现是控制器不出 cmd_vel 却不报错。2026-08-08 首次真起 EKF 时发现。
        remappings=[('odometry/filtered', 'odom')],
    )
    stage2 = TimerAction(period=5.0, actions=[ekf])

    # ===== 阶段3 (t=10): Nav2 (无 RViz) + cmd_vel 平滑 + 仲裁 + lane_navigator =====
    # 无头 (§7-E): 直接 include nav2_bringup/bringup_launch.py, 不走 mm_navigation 的
    # navigation2.launch.py (那个无条件起 rviz2)。
    # cmd_vel 走向 (三路汇入 mux, 只有 mux 能发 /cmd_vel):
    #   Nav2 controller/behavior --SetRemap--> /cmd_vel_nav -> velocity_smoother(Nav2 自带,
    #       加速度限幅) --SetRemap--> /cmd_vel_nav_out ┐
    #   lane_navigator cspin 闭环转向 -> /cmd_vel_spin ├-> twist_mux -> /cmd_vel -> 固件
    #   joy_arm_teleop (仅 DRIVE 态) -> /cmd_vel_manual ┘
    #   (自研电机速度环无加减速斜坡, 平滑只能在上位机这级补, 与仿真同策略。)
    # 末端那一级 twist_mux 是 D4.2: 手柄优先级最高(100) > cspin(50) > 导航(10), 于是遥控与
    # 导航可以常驻共存 —— 不必再为测导航去杀 joy_arm_teleop。详见
    # mm_navigation/config/twist_mux.yaml。
    # ⚠️ 任何一路都不许直发 /cmd_vel: mux 在所有输入超时时持续发零, 谁绕过 mux 就与那串零
    # 形成两个发布者交替 -> 车抽搐。2026-08-09 同时踩到两处 (cspin 直发 + Nav2
    # velocity_smoother 直发), 见下方 SetRemap 与 lane_navigator.py 的 cmd_pub 注释。
    nav2 = GroupAction([
        SetRemap('/cmd_vel', '/cmd_vel_nav'),
        # ⚠️ 把 Nav2 自带 velocity_smoother 的输出从 /cmd_vel 改接到 /cmd_vel_nav_out,
        # 使它经 twist_mux 而不是直怼固件。nav2_bringup/navigation_launch.py:182 原本写死
        #   remappings + [('cmd_vel','cmd_vel_nav'), ('cmd_vel_smoothed','cmd_vel')]
        # 即 Nav2 原生就自带一条 controller -> /cmd_vel 的完整平滑链。它绕过 mux ->
        # 与 mux 一起成为 /cmd_vel 的两个发布者互相冲刷, 车抽搐; 且手柄 R1 压不住导航
        # (mux 只能压 mux 自己那路)。2026-08-09 跑 pick3 时实测 /cmd_vel Publisher count=2。
        # 这条 SetRemap 能盖住节点自带那条: launch_ros/actions/node.py:468-476 把
        # global_remaps(SetRemap) 排在节点 remappings **之前**, 而 rcl 取首条命中。
        SetRemap('/cmd_vel_smoothed', '/cmd_vel_nav_out'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_yaml,
                'params_file': nav2_params,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
    ])
    # (原先这里还起一个自研 mm_description/cmd_vel_smoother.py 做 /cmd_vel_nav ->
    #  /cmd_vel_nav_out 的限幅, 2026-08-09 删。理由: Nav2 自带的 velocity_smoother 是
    #  同一份活儿且配置更全(nav2_params.yaml:423 有 OPEN_LOOP 反馈/deadband/三轴独立限幅),
    #  两个平滑器并行接在同一输入上纯属重复。上面那条 SetRemap 已把 Nav2 那个的输出接进
    #  mux, 链路遂成单一路径: controller -> /cmd_vel_nav -> velocity_smoother
    #  -> /cmd_vel_nav_out -> twist_mux -> /cmd_vel。)
    # 仲裁: 导航(低优先级) vs 手柄(高优先级) -> /cmd_vel。R1 死人开关天然是接管键,
    # 松开后 0.5s timeout 自动交还导航, 见 twist_mux.yaml。
    # ⚠️ twist_mux 的输出话题名固定为 `cmd_vel_out`(源码硬编码), 必须靠 remap 改成
    # /cmd_vel —— 不能靠参数设。
    twist_mux = Node(
        package='twist_mux', executable='twist_mux', name='twist_mux', output='screen',
        parameters=[os.path.join(mm_nav_share, 'config', 'twist_mux.yaml'),
                    {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel_out', '/cmd_vel')],
    )
    # corner_radius 必须显式传: 节点默认 0.8 是仿真值, 在真机图上会撞 —— 去 place1 要过
    # x≈-2.6 那条净宽仅 0.65~0.70m 的南北通道, 半径 >=0.35 时倒出的圆弧被甩到通道内侧墙上
    # (实测 r=0.4 撞 1 点 / r=0.5~0.8 撞 2 点, 撞点都在 (-2.56,+1.9) 附近)。<=0.3 全部通过。
    lane_navigator = Node(
        package='mm_navigation', executable='lane_navigator.py', name='lane_navigator',
        output='screen', parameters=[{'use_sim_time': use_sim_time,
                                      'corner_radius': 0.3}])
    # use_nav2:=false 时整段跳过 (建图/纯遥操作场景)。⚠️ twist_mux 也在这一段里, 故关掉
    # Nav2 时手柄是**直连**固件的 —— joy_arm_teleop 得自己发 /cmd_vel 才能动车。若哪天想让
    # mux 常驻, 要把它挪出这个 GroupAction 并确认 joy_arm_teleop 的出口话题跟着改。
    #   lane_navigator 会调 Nav2 action, Nav2 不在时只是空等, 但没有意义故一并关。
    stage3 = TimerAction(period=10.0, actions=[
        GroupAction([nav2, twist_mux, lane_navigator],
                    condition=IfCondition(use_nav2)),
    ])

    # ===== 阶段4 (t=14): MoveIt move_group (无头, 依赖阶段1 的 RSP) =====
    move_group = _include('arm_moveit_config', 'move_group.launch.py', launch_arguments=real_arg)
    stage4 = TimerAction(period=14.0, actions=[move_group])

    # ===== 阶段5 (t=18): moveit_servo + grasp_node (依赖 move_group) =====
    grasp = _include('mm_grasp', 'grasp.launch.py', launch_arguments=real_arg)
    stage5 = TimerAction(period=18.0, actions=[grasp])

    # ===== 阶段6 (t=20): mm_perception 真感知 (默认关) =====
    # 仿真用 mock (mock_object_detector/mock_aruco); 真机换真节点, 话题接口一致上层无感。
    # 盒子识别 = yolo_box_detector (自训练 YOLO, NCNN 后端): 发 /perception/object_pose
    #   + /object_poses(可抓盒数组) + /object_point_cam(相机系, 精修闭环用) + /object_axis_angle。
    #   必须走它自己的 launch 而不是裸 Node: 那个 launch 要读 yaml 再把 NCNN 模型目录拼成
    #   share 下的绝对路径 (yaml 里存的是相对名), 裸起会因模型路径找不到而挂。
    #   with_rsp:=false —— 整车 TF 由阶段1 的臂 RSP 发, 别再起第二个 rsp 抢 /robot_description。
    # ArUco: aruco_localizer -> /tf 广播 aruco_<id> (parent base_link), 吃 cam_a 转正流。
    # 两者都吃相机, 故需连带 use_cameras:=true。
    yolo = _include('mm_perception', 'yolo_box_detector.launch.py',
                    launch_arguments={'with_rsp': 'false'}.items(),
                    condition=IfCondition(use_perception))
    aruco_localizer = Node(
        package='mm_perception', executable='aruco_localizer', name='aruco_localizer',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('mm_perception'),
                         'config', 'aruco_localizer.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(use_perception))
    stage6 = TimerAction(period=20.0, actions=[yolo, aruco_localizer])

    # ===== 阶段7 (t=25): mm_task 状态机 (默认关, run_mission:=true 自动跑) =====
    # mission_file 留空时传空串: mission_manager 见空串会自己回退到包内 mission.yaml,
    # 故这里无需在 launch 侧再判一次 (真机传 mission_real.yaml)。
    mission = _include('mm_task', 'mission.launch.py',
                       launch_arguments={'use_sim_time': use_sim_time,
                                         'mission_file': mission_file}.items(),
                       condition=IfCondition(run_mission))
    stage7 = TimerAction(period=25.0, actions=[mission])

    return LaunchDescription(
        args + stage1 + [stage2, stage3, stage4, stage5, stage6, stage7])
