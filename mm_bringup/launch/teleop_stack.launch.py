"""一键遥控全栈: 起这一个就能拿手柄装货卸货 (路线图 D2 收尾).

原先要手敲 6 条命令且顺序不能乱, 漏一条的表现还各不相同 (漏 agent = 车不动但臂正常;
漏感知 = ✕ 抓取阻塞 12s 才报错, 期间点动僵住). 本 launch 把整条链按依赖分阶段编排:

  t=0   micro_ros_agent (底盘 ESP32) + 臂实机 (RSP/ros2_control/JTC/CAN 桥) + D435i
  t=6   move_group (要 RSP 的 robot_description 先在, 否则起不来)
  t=12  servo_node + grasp_node (要 move_group 的 planning scene)
  t=18  yolo_box_detector (要相机出图 + TF 树含 Link_30->base_link, 后者靠真实 joint_states)
  t=22  joy_node + teleop_twist_joy + joy_arm_teleop (最后起: 手柄一通就能动臂)

⚠️ **不含 Nav2, 也不含 twist_mux**。D4.2 之后 joy_arm_teleop 输出改到 /cmd_vel_manual
   交 mux 仲裁, 本 launch 没起 mux -> **单跑它手柄推了车不动** (指令停在
   /cmd_vel_manual 无人转发, 不报错, 只是车不响应)。两条正确用法:
     - 遥控 + 导航共存(推荐): real_bringup.launch.py use_nav2:=true —— 它带起 mux,
       按住 R1 手柄接管(优先级 100 > 导航 10), 松开 0.5s 自动交还导航;
     - 只要遥控无臂状态机: teleop.launch.py (直发 /cmd_vel, 不经仲裁)。

⚠️ 前置 (本 launch 管不了的):
  - `source ~/microros_ws/install/setup.bash` —— micro_ros_agent 在独立 ws, 不 source
    则整个 launch 起不来(找不到可执行). 这是最容易漏的一条。
  - `export ROS_DOMAIN_ID=42` (与本机 RViz 一致)
  - 24V 电机上电, 且臂在零位 (增量编码器, 零点在驱动器里, 停机前必须按 START 回零)

用法:
  ros2 launch mm_bringup teleop_stack.launch.py
  ros2 launch mm_bringup teleop_stack.launch.py use_perception:=false   # 不用视觉抓取时省算力
  ros2 launch mm_bringup teleop_stack.launch.py use_rviz:=true          # Nano 本地可视化(卡)

键位/状态机见 mm_grasp/scripts/joy_arm_teleop.py 的模块 docstring.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(pkg, launch_file, launch_arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(pkg), 'launch', launch_file)),
        launch_arguments=(launch_arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    use_perception = LaunchConfiguration('use_perception')
    use_rviz = LaunchConfiguration('use_rviz')

    args = [
        DeclareLaunchArgument(
            'agent_serial_dev', default_value='/dev/esp32_chassis',
            description='底盘 ESP32-S3 串口 (udev 软链, 指向 ttyACM*)'),
        DeclareLaunchArgument(
            'use_perception', default_value='true',
            description='起 D435i + yolo_box_detector。false 则 ✕ 抓取不可用 '
                        '(取不到 /perception/object_pose), 但点动/放置/卸货照常'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Nano 本地起 RViz。默认关 —— 跨 WiFi 在笔记本上起更流畅'),
        DeclareLaunchArgument(
            'use_body_cameras', default_value='true',
            description='起车体两路 USB 相机(cam_a / cam_b 监视)。2026-08-04 起默认开: '
                        '本机 dev_bringup view_cameras:=true 就直接有画面, 不用记参数。'
                        '实测代价 ~0.5 核 + 6MB/s 过网; 不看画面时传 false 省掉'),
        DeclareLaunchArgument(
            'home_on_start', default_value='false',
            description='起栈后自动把臂摆到 home 收拢位。臂当前位置未知时别开(会突然大幅运动)'),
        DeclareLaunchArgument(
            'drive_without_arm', default_value='false',
            description='臂栈没跑时也允许进 DRIVE 遥控底盘 (绕过"臂已收拢"校验)'),
    ]

    # ===== t=0: 硬件层 =====
    # 底盘: 固件直发 /wheel_odom + /imu, 订 /cmd_vel, 不发 TF, 故代理无需重映射.
    micro_ros_agent = Node(
        package='micro_ros_agent', executable='micro_ros_agent',
        name='micro_ros_agent', output='screen',
        arguments=['serial', '--dev', LaunchConfiguration('agent_serial_dev')],
    )
    # 臂: RSP(hw:=real) + ros2_control + JTC + can_bridge. 整车 robot_description 由此唯一发布.
    arm_real = _include('arm_moveit_config', 'real_bringup.launch.py')
    # 深度相机必起(抓取要它); 车体两路 USB 相机 2026-08-04 起也默认开, 本机看画面只需
    #   ros2 launch mm_bringup dev_bringup.launch.py view_cameras:=true
    # Nano 侧只起 usb_cam 一级, 且只发 image_raw/compressed (0.09MB/帧@30Hz, 两路 ~6MB/s);
    # 解码与转正都在本机做. 实测 cam_a 31.9Hz / cam_b 23.6Hz (两路合计约 55, 在抢共享上限).
    # ⚠️ 裸 image_raw 已被 enable_pub_plugins 关掉 —— 它是长期卡顿的真凶 (640x480 rgb8
    # @30Hz ≈ 27MB/s 一路, 跨机时 DDS 照样往 WiFi 推). 详见 mm_perception/launch/
    # cameras.launch.py 的 _one_cam, 那里有完整判据, 别再打开.
    # align_depth 在 cameras.launch.py 里刻意关掉 (无人订阅, 白烧近半个核): yolo 走原始
    # 深度流. 那些"align_depth 必开"的旧注释已不成立, 详见 cameras.launch.py 的 _depth_cam.
    depth_cam = _include('mm_perception', 'cameras.launch.py',
                         {'use_body_cameras': LaunchConfiguration('use_body_cameras'),
                          'use_depth_camera': 'true'},
                         condition=IfCondition(use_perception))

    # ===== t=6: MoveIt =====
    # 必须等 RSP 把 robot_description 发出来, 否则 move_group 起不来.
    #
    # ⚠️ 必须用 scoped GroupAction 围起来: move_group.launch.py 里是全局 SetParameter 设
    # use_sim_time, 不隔离会泄漏到**本文件后续每一个节点**, 使它们都多带
    # `-p use_sim_time:=False -p trajectory_execution.allowed_execution_duration_multiplier`。
    # 2026-08-03 实测后果: 受污染的上下文里, 被 include 的 launch 传 dict 参数会渲染成
    # `/**:` 通配键而非具名键, 而通配键**优先级低于** yaml 的具名键 —— yolo_box_detector
    # 那份 overrides 算好的模型绝对路径就这样被 yaml 里的相对名压掉, ultralytics 当场
    # FileNotFoundError 退出。单独起 yolo launch 不会, 因为没有污染源。
    # scoped=True 让 SetParameter 只作用于组内, 这是这类 launch 的正确围法。
    move_group = GroupAction(scoped=True, actions=[
        _include('arm_moveit_config', 'move_group.launch.py', {'use_sim_time': 'false'}),
    ])

    # ===== t=12: 伺服 + 抓取状态机 =====
    # grasp_node 构造时就要 move_group 的 planning scene; servo 要 /joint_states.
    grasp = _include('mm_grasp', 'grasp.launch.py', {'use_sim_time': 'false'})

    # ===== t=18: 视觉 =====
    # 要相机出图, 还要 TF 链 base_link->...->Link_30 通 —— 中间 6 个活动关节靠真实
    # /joint_states 才算得出 TF, 所以必须排在臂栈之后. 不起 rsp: 已由臂栈起,
    # 再起一个会有两个节点抢发同一套 TF.
    #
    yolo = _include('mm_perception', 'yolo_box_detector.launch.py',
                    condition=IfCondition(use_perception))

    # ===== t=22: 手柄遥控 (最后起) =====
    # 放最后是有意的: joy_arm_teleop 一起来手柄就能动臂, 此前各段必须已就位.
    teleop = _include('mm_bringup', 'teleop_full.launch.py', {
        'home_on_start': LaunchConfiguration('home_on_start'),
        'drive_without_arm': LaunchConfiguration('drive_without_arm'),
    })

    # 三路监视页 (web_video_server 8080 + 静态页 8081): 抽到 monitor.launch.py 了,
    # 那边有全部实测数据与"别改 publish_rate / 别给 server_threads"等判据。
    # 抽出去的原因: 它与遥控/抓取毫无耦合, 纯"给人看画面"的一层, 而 real_bringup 那条
    # 主线跑真机时也要看画面 —— 定义留在本文件里就意味着"想看画面必须起遥控全栈"。
    # 现在两边都 include 同一份, 也可以完全独立起: ros2 launch mm_bringup monitor.launch.py
    monitor = _include('mm_bringup', 'monitor.launch.py',
                       condition=IfCondition(use_perception))

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz', output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(args + [
        micro_ros_agent, arm_real, depth_cam,
        TimerAction(period=6.0, actions=[move_group]),
        TimerAction(period=12.0, actions=[grasp]),
        TimerAction(period=18.0, actions=[yolo, monitor]),
        TimerAction(period=22.0, actions=[teleop, rviz]),
    ])
