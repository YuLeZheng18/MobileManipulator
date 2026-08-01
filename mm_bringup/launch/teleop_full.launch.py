"""手柄三态遥控 launch: 收拢 / 底盘行进 / 臂点动抓取 (路线图 D2 收尾).

    joy_node -> /joy ─┬─> teleop_twist_joy -> /cmd_vel_joy ─┐
                      │                                     ├─> joy_arm_teleop -> /cmd_vel
                      └─> joy_arm_teleop (状态机 + 臂点动) ──┘

状态机: HOME ──START──> DRIVE <──SELECT──> ARM
        臂 home         臂 ready          臂 look
        车禁止          车放行            车禁止
上电即 HOME (车臂都不动), 按 START 收臂到 ready 才放行底盘, SELECT 在行进/抓取间切。
键位与状态机细节见 joy_arm_teleop.py 的模块 docstring。

复用同包 teleop.launch.py 起前两个 (轴/按键/速度上限是 D2.2 实测定稿值, 不在这里
重写一份), 只把它的输出从 /cmd_vel 改到 /cmd_vel_joy 交 joy_arm_teleop 仲裁。
单独跑底盘遥控仍用 teleop.launch.py, 那条路直发 /cmd_vel 不经仲裁节点。

节点 joy_arm_teleop 在 mm_grasp (它调 /grasp/*, 属抓取域), 而本 launch 放 mm_bringup:
mm_bringup 已 exec_depend mm_grasp, 反向再加依赖就成环, colcon 会拒绝解析构建顺序。

⚠️ 前置: 臂模式要能动, 得先有臂 + move_group + servo_node + grasp_node:
    ros2 launch arm_moveit_config real_bringup.launch.py
    ros2 launch arm_moveit_config move_group.launch.py use_sim_time:=false
    ros2 launch mm_grasp grasp.launch.py use_sim_time:=false
底盘要能动还得起 micro_ros_agent (最容易漏):
    source ~/microros_ws/install/setup.bash
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp32_chassis

⚠️ 不与 Nav2 同时跑: 两者都往 /cmd_vel 发会打架 (路线图 D4.2 的 mux 还没做)。

⚠️ 安全: 臂点动经 moveit_servo 直发 /arm_controller/joint_trajectory, **绕过 MoveIt
碰撞检查**, 约束盒是唯一防线。而盒顶/四角当前是占位值, 首次真机务必架空或
把 servo_scale_linear 调小起步。三个待实测量见 joy_arm_teleop.py 里的 TODO。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joy_config = LaunchConfiguration('joy_config')

    args = [
        DeclareLaunchArgument(
            'joy_config',
            default_value=os.path.join(
                get_package_share_directory('mm_bringup'), 'config', 'teleop_ps2.yaml'),
            description='手柄轴/按键映射 (D2.2 实测定稿, 与单独遥控共用同一份)'),
        DeclareLaunchArgument(
            'home_on_start', default_value='false',
            description='起栈后自动把臂摆到 home 收拢位。臂当前位置未知时别开 (会突然大幅运动)'),
        DeclareLaunchArgument(
            'drive_without_arm', default_value='false',
            description='臂栈没跑时也允许进 DRIVE 遥控底盘。⚠️ 绕过"臂已收拢"校验, '
                        '仅在臂确实没通电时用'),
    ]

    # joy_node + teleop_twist_joy, 输出改到 /cmd_vel_joy 交 joy_arm_teleop 仲裁。
    base_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mm_bringup'), 'launch', 'teleop.launch.py')),
        launch_arguments={
            'config_file': joy_config,
            'cmd_vel_topic': '/cmd_vel_joy',
        }.items(),
    )

    # 状态机 + 臂点动。按键号/轴号已是 2026-08-01 实测定稿; 约束盒盒顶/四角仍待实测。
    arm_teleop = Node(
        package='mm_grasp',
        executable='joy_arm_teleop.py',
        name='joy_arm_teleop',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'home_on_start': LaunchConfiguration('home_on_start'),
            'drive_without_arm': LaunchConfiguration('drive_without_arm'),
        }],
        remappings=[
            ('cmd_vel_in', '/cmd_vel_joy'),
            ('cmd_vel_out', '/cmd_vel'),
            ('joy', '/joy'),
        ],
    )

    return LaunchDescription(args + [base_teleop, arm_teleop])
