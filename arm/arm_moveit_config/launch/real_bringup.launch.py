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
            "query_rate_hz": 10.0,
            "auto_enable": True,
            # 整体提速试验: 六轴输出轴上限统一到 0.8rad/s(输出轴~7.6RPM).
            # 输出轴速度=电机speed/减速比, 按 speed=0.8*ratio*9.549 算, 六轴全覆盖保持同步到位.
            # 若电机丢步/异响/过冲, 说明 speed 太高, 往回降. ratio=[50,50,30,82.67,62.5,27].
            # J5: 电机到物理极限, 600也跟不上->不再顶巡航, 收回300(可靠区间); 改由joint_limits压J5上限让全臂同步.
            "motor_speeds": [382, 382, 229, 632, 300, 206],
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
