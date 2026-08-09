"""
实车机械臂 bringup (不启动 gazebo).

启动链:
  robot_state_publisher (hw:=real 展开, ros2_control 用 topic_based 后端)
  ros2_control_node (controller_manager, 100Hz)
    -> joint_state_broadcaster
    -> arm_controller (JointTrajectoryController, 五次样条插补)
  arm_can_bridge (订阅 JTC 稠密指令 /arm_joint_commands -> 0xFD 发 CAN; 读反馈 -> /arm_joint_states)

配合 move_group.launch.py 即可在实车上做 MoveIt 规划执行.
实车与仿真共用同一套 MoveIt + JTC, 仅硬件后端不同 (xacro hw 参数切换).
"""
import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("arm_moveit_config")

    xacro_file = PathJoinSubstitution([pkg, "config", "arm_description.urdf.xacro"])
    controllers_yaml = PathJoinSubstitution([pkg, "config", "ros2_controllers.yaml"])

    # hw:=real -> ros2_control 用 topic_based_ros2_control/TopicBasedSystem
    # on_stderr='ignore': xacro 的 load_yaml deprecation 警告走 stderr, 不应判失败
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [FindExecutable(name="xacro"), " ", xacro_file, " ", "hw:=real"],
                on_stderr="ignore",
            ),
            value_type=str,
        )
    }

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, controllers_yaml],
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    can_bridge = Node(
        package="arm_control",
        executable="can_bridge",
        output="screen",
        parameters=[{
            "command_topic": "/arm_joint_commands",
            "state_topic": "/arm_joint_states",
            "send_rate_hz": 100.0,
            # 30 而不是原来的 10: _send_loop 发位置帧期间置 _query_paused 挡住查询帧,
            # 10Hz 下查询线程醒来常撞在挡窗里、撞上就等下一个 100ms 周期。2026-08-02 实测
            # 把它从 10 提到 30 后 /arm_joint_states 从 55Hz 涨到 121Hz、最大间隔从
            # 106~209ms 降到 46ms。反馈更密对 JTC 闭环有实际好处, 故保留。
            # ⚠️ 别再往上提: _query_paused 的挡窗还在(它是为防查询帧插进双帧中间造成
            # 00 EE, 别直接删), 提太高只是更频繁地撞窗。
            "query_rate_hz": 30.0,
            # 'fd'=梯形曲线 / 'fb'=直通限速(无梯形加减速).
            # 2026-08-03 定回 fd, 依据是**异响**这个此前没被单独测过的现象:
            # fb 下规划时 J12/J13 有高频振动+异响(J14/J15 轻微, J11/J16 干净), 切 fd 后
            # 规划完全干净、异响消失、点动同时改善。规划与点动共用同一位置环, 故异响生在
            # 驱动器内部而非上游命令流。
            # 与协议手册对得上(ZDT_X57_V2 原文核对, 非记忆): 0xFB 位置环 Kp=0x00071ED0
            # =466640, 0xFD 位置环 Kp=0x0001EEB0=126640, **fb 高 3.68 倍**。负载相关的
            # 高频振动+异响正是增益相对折算惯量过高、在目标附近猎振的特征 —— J11 与 J12
            # 减速比同为 50 却一好一坏, 排除了减速比/速度/量化, 只剩负载能区分二者。
            # ⇒ 修法是换回 fd 保留其梯形斜坡的阻尼作用, **不是**改驱动器 PID(0x4A):
            #   那 3.68 倍是手册默认值, 不是从这六个驱动器读出的实测值, 且写 PID 动的是
            #   厂调状态, 风险远高于换模式。别去写 0x4A。
            # ⚠️ fd 的已知代价, 别再重复踩:
            #   ① motor_accels 不能提(见下方注释, 20000 实测剧烈抖动+异响)。
            #   ② fd 每帧重启梯形斜坡, 与 send_deadband_deg 构成两难: 死区大 ⇒ 从动轴
            #      秒级阶跃; 死区小 ⇒ 从动轴 100Hz 微幅斜坡重启。0.05 是实测较优的一侧。
            # ⚠️ 当年从 fd 换到 fb 的理由是 fd **顿挫**, 而顿挫的真因后来查明在别处并已修:
            #   servo publish_joint_velocities 曾为 false、速度字段曾恒发 382(现由
            #   _feedforward_speed 按剩余距离算)、butterworth 系数曾默认 1.5(现 5.0)。
            #   那三条修完后 fd 不再顿, 所以这次切回不是原地打转。
            "position_mode": "fd",
            # 诊断开关: 额外查 0x37 电机位置误差 -> /arm_pos_error (度, 电机轴).
            # 三种波形对应三个相反的处理方向, 判据见 can_bridge.py 参数声明处注释.
            # ⚠️ 平时关掉: 查询帧翻倍占总线。这次留 True 是为了改完 servo 后复测对照。
            "query_pos_error": True,
            "auto_enable": True,
            # 整体提速试验: 六轴输出轴上限统一到 0.8rad/s(输出轴~7.6RPM).
            # 输出轴速度=电机speed/减速比, 按 speed=0.8*ratio*9.549 算, 六轴全覆盖保持同步到位.
            # 若电机丢步/异响/过冲, 说明 speed 太高, 往回降. ratio=[50,50,30,82.67,62.5,27].
            # J5: 电机到物理极限, 600也跟不上->不再顶巡航, 收回300(可靠区间); 改由joint_limits压J5上限让全臂同步.
            # 速度**上限**(不再是每帧恒发值): 实发速度由 JTC 给的 velocity 前馈算出,
            # 见 can_bridge._feedforward_speed(). 这里只当天花板.
            "motor_speeds": [382, 382, 229, 632, 300, 206],
            # 加减速度(RPM/s, 电机轴). 保持 500 —— **别再往上提, 已实测有害**。
            # 2026-08-02 试过 [20000,20000,12000,30000,20000,10000](按"加速段≈帧周期20%"
            # 反算): home->ready 剧烈抖动 + 异响, 当场回退。
            # 原因不是扭矩不够, 是**每帧重发梯形规划**与 100Hz 指令流不兼容: 0xFD 每帧语义是
            # "加速→减速→停在目标", 20000RPM/s=333rev/s², 于是电机每 10ms 被要求猛加速再猛
            # 减速停住, 加速度方向每 10ms 反向一次 ⇒ 100Hz 强迫激励, 正落在步进共振带。
            # 500 之所以安静, 恰恰因为它连加速段都跑不完, 从来没有那次减速反向。
            # ⇒ 顿挫要靠**去掉每帧的减速到停**来解(0xFB 直通限速), 不是调这个值。
            "motor_accels": [500, 500, 500, 500, 500, 500],
            # 落后帧数: 实发速度 = 剩余距离 /(此值 × 指令帧周期), 使电机永远差一点没走到,
            # 下一帧目标已到 ⇒ 不空等。稳态落后 = 此值 × 每帧增量(J1 约 0.42°), 不累积。
            # 调大 = 更平滑但跟随滞后更多; 调小趋近 1 = 更贴目标但会短暂到位空等(顿).
            "lag_frames": 1.3,
        }],
    )

    # broadcaster 起来后再起 arm_controller; CAN 桥接与 controller_manager 同时起
    delay_arm = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner])
    )

    return LaunchDescription([
        rsp,
        control_node,
        TimerAction(period=2.0, actions=[jsb_spawner]),
        delay_arm,
        can_bridge,
    ])
