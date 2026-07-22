"""实机总 launch (real-only) — 一键起真机全栈 (架构 §7 自底向上六段).

与仿真的唯一区别在最底层硬件抽象 (谁产 /odom、谁吃 /cmd_vel); 其上 TF/话题/
服务/action 与仿真完全一致, 上层节点 (Nav2/MoveIt/grasp/mm_task) sim/real 无感 (架构 §4)。
故本 launch 不起 Gazebo/mock, 改起: micro-ROS 代理 + CAN 桥 + 真雷达/相机 + EKF 状态估计。
无头 (§7-E): 不起 RViz。

分阶段错峰 (TimerAction, 上层等下层就绪):
  t=0   micro-ROS 代理(底盘) + 机械臂实机 (RSP hw:=real + ros2_control + CAN 桥) + 雷达/相机
  t=5   robot_localization EKF: 融合 /wheel_odom + /imu -> /odom + odom->base_link TF
  t=10  Nav2 (无 RViz) + cmd_vel 平滑 + lane_navigator
  t=14  MoveIt move_group
  t=18  moveit_servo + grasp_node
  t=20  mm_perception 真感知 (队友节点, 默认关; 调试就绪后 use_perception:=true)
  t=25  mm_task 状态机 (默认关; run_mission:=true 自动跑 S0->S5)

⚠️ 运行前置 (目标机 Nano 上现装/现 source, 不进本 colcon ws):
  - micro_ros_agent: 需先 `source ~/microros_ws/install/setup.bash` 再起本 launch,
    否则找不到 micro_ros_agent 可执行 (代理是独立 ws, 见架构 §5.2 记忆)。
  - rplidar_ros / 深度相机驱动 / robot_localization: apt 或源码装在 Nano 上。
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
    use_cameras = LaunchConfiguration('use_cameras')
    use_perception = LaunchConfiguration('use_perception')
    agent_serial_dev = LaunchConfiguration('agent_serial_dev')
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')
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
                              description='起 rplidar_ros (思岚 A3 -> /scan, frame Link_12)'),
        DeclareLaunchArgument('use_cameras', default_value='false',
                              description='起深度相机/车体相机驱动 (TODO: 驱动型号确定后接线)'),
        DeclareLaunchArgument('use_perception', default_value='false',
                              description='起 mm_perception 真感知 (队友节点调试中, 就绪后开)'),
        DeclareLaunchArgument('agent_serial_dev', default_value='/dev/ttyACM0',
                              description='micro-ROS 代理串口 (ESP32-S3 native USB)'),
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyTHS1',
                              description='思岚雷达串口 (Jetson 硬件 UART; 走排针串口非 USB, 用前 stop nvgetty)'),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(mm_nav_share, 'maps', 'room.yaml'),
            description='Nav2 地图 (默认复用仿真同图, 实机重建后替换)'),
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

    # 雷达: 思岚 A3 -> rplidar_ros -> /scan (frame_id=Link_12, 与仿真/URDF 一致, 架构 §5.1)
    lidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar',
        output='screen',
        parameters=[{
            'serial_port': lidar_serial_port,
            'serial_baudrate': 256000,   # A3
            'frame_id': 'Link_12',
            'scan_mode': 'Standard',
        }],
        condition=IfCondition(use_lidar),
    )

    # TODO 相机驱动 (use_cameras:=true 时起, 驱动型号/话题确定后接线):
    #   - 手眼深度相机 (Link_30, Joint_17 固连 Link_29 腕部): 供 mm_perception 抓取识别。
    #   - 车体 ArUco 相机 (Link_13): 供 aruco_localizer 精对位。
    #   frame_id 必须对齐 URDF 的 Link_30 / Link_13 (interface_contract 链接角色表)。
    #   例: realsense2_camera / usb_cam Node, 此处按实物补。

    stage1 = [micro_ros_agent, arm_real, lidar]

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
    )
    stage2 = TimerAction(period=5.0, actions=[ekf])

    # ===== 阶段3 (t=10): Nav2 (无 RViz) + cmd_vel 平滑 + lane_navigator =====
    # 无头 (§7-E): 直接 include nav2_bringup/bringup_launch.py, 不走 mm_navigation 的
    # navigation2.launch.py (那个无条件起 rviz2)。
    # cmd_vel 走向: Nav2(controller/behavior 发 /cmd_vel) --SetRemap--> /cmd_vel_nav
    #   -> cmd_vel_smoother 加速度限幅 -> /cmd_vel -> 底盘固件订阅。
    #   (自研电机速度环无加减速斜坡, 平滑只能在上位机这级补, 与仿真同策略。)
    nav2 = GroupAction([
        SetRemap('/cmd_vel', '/cmd_vel_nav'),
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
    cmd_vel_smoother = Node(
        package='mm_description',
        executable='cmd_vel_smoother.py',
        name='cmd_vel_smoother',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': '/cmd_vel_nav',
            'output_topic': '/cmd_vel',
            'linear_acceleration': 3.0,
            'angular_acceleration': 2.0,
            'rate': 50.0,
        }],
    )
    lane_navigator = Node(
        package='mm_navigation', executable='lane_navigator.py', name='lane_navigator',
        output='screen', parameters=[{'use_sim_time': use_sim_time}])
    stage3 = TimerAction(period=10.0, actions=[nav2, cmd_vel_smoother, lane_navigator])

    # ===== 阶段4 (t=14): MoveIt move_group (无头, 依赖阶段1 的 RSP) =====
    move_group = _include('arm_moveit_config', 'move_group.launch.py', launch_arguments=real_arg)
    stage4 = TimerAction(period=14.0, actions=[move_group])

    # ===== 阶段5 (t=18): moveit_servo + grasp_node (依赖 move_group) =====
    grasp = _include('mm_grasp', 'grasp.launch.py', launch_arguments=real_arg)
    stage5 = TimerAction(period=18.0, actions=[grasp])

    # ===== 阶段6 (t=20): mm_perception 真感知 (队友节点, 默认关) =====
    # 仿真用 mock (mock_object_detector/mock_aruco); 真机换队友真节点, 话题接口一致上层无感。
    # 队友节点仍在调试, 故 IfCondition(use_perception) 默认 false; 就绪后 use_perception:=true。
    # 真节点 (interface_contract §1/§2): object_detector -> /perception/object_pose;
    #   aruco_localizer -> /tf 广播 aruco_<id> (parent base_link)。均吃深度/车体相机, 故连带 use_cameras。
    object_detector = Node(
        package='mm_perception', executable='object_detector', name='object_detector',
        output='screen', parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_perception))
    aruco_localizer = Node(
        package='mm_perception', executable='aruco_localizer', name='aruco_localizer',
        output='screen', parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_perception))
    stage6 = TimerAction(period=20.0, actions=[object_detector, aruco_localizer])

    # ===== 阶段7 (t=25): mm_task 状态机 (默认关, run_mission:=true 自动跑) =====
    mission = _include('mm_task', 'mission.launch.py',
                       launch_arguments=real_arg, condition=IfCondition(run_mission))
    stage7 = TimerAction(period=25.0, actions=[mission])

    return LaunchDescription(
        args + stage1 + [stage2, stage3, stage4, stage5, stage6, stage7])
