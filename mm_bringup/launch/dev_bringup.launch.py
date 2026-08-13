"""本机 (笔记本) bringup — 分布式调试的"上半场" (架构 §7.4)。

只跑给人看的可视化 + 粗粒度调度, 不碰任何硬件、不产任何 TF:
  - RViz (**导航视图** nav_real.rviz: 地图 + 双 costmap + 车模 + TF + 滤后雷达 + 全局/车道
    规划 + AMCL 位姿, 带 SetInitialPose/SetGoal 工具)。臂的规划场景不在这份里, 要看单独起
    `ros2 launch arm_moveit_config moveit_rviz.launch.py`。
  - 三路相机监视 (默认开): xdg-open 拉浏览器开 Orin 的 monitor.html, 见下。
  - mm_task 状态机 (默认关): 发 /go_to /initialpose、调 /grasp/* 服务, 都是小消息粗指令。

⚠️⚠️ 相机画面走浏览器, **不经 ROS** (2026-08-04 定案, 这是折腾一整晚的结论)。
  Nano 上 monitor.launch.py 起了 web_video_server (HTTP/MJPEG); 本机这边 view_cameras:=true
  (默认) 只是 xdg-open 一个本地 html, 页面里三个 <img> 直连 Orin。
  三路地址/分辨率/invert 全写在 mm_bringup/web/monitor.html 里, 连同实测数据和
  "为什么不能把分辨率调大"的判据 —— 要改画面就改那个 html, 不用碰本文件。
  实测三路并发 (臂栈+yolo+servo 同跑): 各 29.9/29.6/28.1 fps, 本机网卡合计 1041KB/s,
  Orin load 4.03/6 核。收到的帧率与机上 camera_info (30.596/30.264/28.853Hz) 一致。

  **铁律: 本机 (或任何跨机进程) 绝不订阅图像话题。** 这是长期卡顿的唯一真凶 ——
  usb_cam 的 image_raw 是未压缩的普通 ROS 发布者, 跨机订它 DDS 就把 640x480 rgb8
  @30Hz ≈ 27MB/s 推上 WiFi, 两路把链路彻底打满, 所有流一起垮。
  判据: 停掉两个 usb_cam, Orin 网卡发送 15312KB/s -> 24KB/s。
  ROS/DDS 不是为跨 WiFi 传视频设计的 (可靠传输 + 每订阅者一份独立拷贝 + 全网发现);
  HTTP 这条全绕开: 图像流留在 Nano 机内, 过网只有一条 TCP, 丢包由浏览器扛。

  ⚠️ 2026-08-04 删掉了原先 view_cameras 起的三级链 (republish -> image_rotator ->
  image_view, 每路三个进程共六个)。它能出画面, 但前提正是上面那条铁律禁止的事。
  参数名保留了, 但现在它只管"要不要 xdg-open 那个网页"。
  连带作废的一堆历史结论 (rqt_image_view 会自己滑回 raw / image_view 重映射键要带
  /compressed 后缀 / Nano 侧 republish 白掉 8Hz / D435i 彩色 jpeg_quality 只在构造时
  读一次) 都只在"跨机订图像"的前提下才有意义, 现在这个前提没了。别再加回来。

  ⚠️ 测跨机带宽只能用 cat /sys/class/net/<iface>/statistics/rx_bytes 前后差。
  ros2 topic bw/hz **自己就是订阅者**, 而 DDS 单播是每订阅者一份拷贝 —— 用它量跨机流量
  会把结果翻倍 (早前那个"5.5 倍放大/重传"就是这么来的测量假象, 不是真的重传)。
  量真实采集帧率要在**机上**量 camera_info (几十字节, 与图像同一次 publish, 不受传输影响)。

机器人端全栈 (硬件/控制/Nav2/MoveIt/感知) 在 Nano 上由 nano_bringup.launch.py 起。
两机同一 ROS_DOMAIN_ID + 同 LAN, DDS 自动发现, 话题/TF/服务/action 跨机透明。
RViz 的机器人模型/TF/规划场景全部来自 Nano (over LAN, 只读可视化)。

调试常用 (本机直接敲, 无需进 launch):
  ros2 topic pub /go_to std_msgs/String "{data: p2}"      # 手动派一段导航
  ros2 service call /grasp/execute std_srvs/srv/Trigger    # 手动触发一次抓取
  ros2 topic echo /perception/object_pose                  # 看感知输出 (小消息, 过网 OK)

⚠️ 运行前置 (本机):
  - `export ROS_DOMAIN_ID=<N>`  与 Nano 一致, 同 RMW。
  - `source install/setup.bash`。
  - 两机 NTP/chrony 对时。
  - 不要在本机 source microros_ws / 不接任何硬件驱动。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    run_mission = LaunchConfiguration('run_mission')
    mission_file = LaunchConfiguration('mission_file')
    view_cameras = LaunchConfiguration('view_cameras')

    args = [
        DeclareLaunchArgument('run_mission', default_value='false',
                              description='true=起 mm_task 自动跑 S0->S5; '
                                          'false=只起 RViz, 手动派命令调试 (推荐)'),
        DeclareLaunchArgument(
            'mission_file',
            default_value=os.path.join(get_package_share_directory('mm_task'),
                                       'config', 'mission_real.yaml'),
            description='任务列表 (本机调度的是真机, 故默认 mission_real.yaml)'),
        DeclareLaunchArgument('view_cameras', default_value='true',
                              description='拉浏览器开三路监视页 (web/monitor.html)。'
                                          '只是开个网页, 不起任何 ROS 图像节点'),
    ]

    # RViz: **导航视图** (mm_navigation/config/rviz/nav_real.rviz)。
    # 2026-08-13 从 MoveIt 视图 (arm_moveit_config/launch/moveit_rviz.launch.py) 换过来:
    # 本机这一侧看的是"车在地图哪、往哪走", 不是臂的规划场景 —— 真机跑整轮时要盯的是
    # AMCL 位姿收敛 / 全局与车道规划 / 双 costmap / 滤后雷达, 这些 moveit.rviz 里一个都没有。
    # 这份 config 含: Map + GlobalCostmap + LocalCostmap + RobotModel + TF + ScanFiltered
    #   + Footprint + GlobalPlan + LaneGraph + LanePlan + AmclPose, Fixed Frame=map,
    #   并带 SetInitialPose / SetGoal 工具 (手动补一次初始位姿或派个点时直接用).
    # 不再 include moveit_rviz.launch.py 而是直接起 rviz2, 是因为那条 include 的全部价值
    # 在于喂 robot_description_semantic / kinematics 给 MotionPlanning 插件, 而这份 config
    # 没有 MotionPlanning; RobotModel 的 Description Source 是 Topic, 直接吃 Nano 发的
    # /robot_description 就够。robot_state_publisher 在 Nano, 本机不另起 rsp。
    # 要看臂的规划场景就单独起: ros2 launch arm_moveit_config moveit_rviz.launch.py
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(
            get_package_share_directory('mm_navigation'),
            'config', 'rviz', 'nav_real.rviz')],
        parameters=[{'use_sim_time': False}],
    )

    # 三路监视画面: 只是拉浏览器开一个本地 HTML, **不起任何 ROS 图像节点** (本文件头的铁律)。
    # 页面里三个 <img> 直连 Orin 的 web_video_server; 分辨率/画质/invert 都写在那个
    # html 里 (连同实测数据和为什么不能调大的判据), 要改布局或参数改 html 即可, 不用碰这里。
    # 延后 3s 是为了让 RViz 先抢到前台, 否则浏览器窗口会盖在它上面。
    # 失败不阻塞 (|| true): 没装浏览器/无 DISPLAY 时不该因此起不来 RViz。
    # ⚠️ 开的是 **Orin 上的 http 地址**, 不是本机 file:// —— 页面由 Orin 的
    # monitor.launch.py 里那个 http.server(8081) 发出, 本仓库只存源文件。
    # 为什么不能开本地文件 (2026-08-04 实测): 本机 `text/html` 的 mimetype 关联被代理
    # 客户端 mihomo-party.desktop 抢走了 (`xdg-mime query default text/html` 可复现),
    # 于是 xdg-open 任何本地 html 都拉起那个代理软件而**永远不进浏览器** —— 退出码照样
    # 是 0, 极具误导性。判据: Orin 侧 `ss -tn '( sport = :8080 )'` 一个连接都没有。
    # 注意 `xdg-settings get default-web-browser` 报的是 firefox, 但那只管
    # `x-scheme-handler/http` —— 与本地文件走的 `text/html` 是两套关联, 别被它骗。
    # 走 http:// 正好用的是那套没被抢走的关联, 所以能进浏览器。
    # 没有去改系统 mimetype 关联是刻意的: 那是用户桌面环境的全局设置, 不该由本仓库动,
    # 而且代理软件下次更新可能再抢回去。
    # 主机名用 ubuntu.local: Orin 的 wlP1p1s0 是 DHCP, IP 会变; 两机都跑 avahi-daemon。
    #
    # ⚠️ **先探测 8081 通了再开浏览器, 不能起来就开** —— 页面在 Orin 上, 由那边的
    # monitor.launch.py 发出, 而 **real_bringup 不会带起它** (2026-08-13 拆分, 理由见
    # monitor.launch.py 的 docstring: 冷启动那两分钟三路编码会跟 move_group 抢核)。
    # 所以这个轮询等的是"人有没有在 Orin 上另起一条 monitor"。
    # 2026-08-04 踩过: 本机固定等 3s 就 xdg-open,
    # 浏览器拿到"无法连接"错误页, 看着像"launch 没跳转", 其实命令执行成功了
    # (判据: launch 日志里 `[bash-N]: process has finished cleanly`)。
    # 现在改成轮询探测, 两机**任意顺序**起都行, 本机会自己等 Orin 就绪。
    # 探测用 curl -sf 只取 HTTP 状态, 不下载页面; 60 次 x 2s = 最多等 2 分钟。
    MONITOR_URL = 'http://ubuntu.local:8081/monitor.html'
    wait_and_open = (
        f'for i in $(seq 1 60); do '
        f'  if curl -sf -m 2 -o /dev/null "{MONITOR_URL}"; then '
        f'    echo "监视页就绪, 打开浏览器: {MONITOR_URL}"; '
        f'    xdg-open "{MONITOR_URL}"; exit 0; '
        f'  fi; '
        f'  sleep 2; '
        f'done; '
        f'echo "等了 2 分钟 Orin 的 8081 还没通 —— Orin 上起 monitor 了吗? '
        f'(ros2 launch mm_bringup monitor.launch.py) 手动开: {MONITOR_URL}" >&2'
    )
    cams = TimerAction(period=3.0, actions=[
        ExecuteProcess(
            cmd=['bash', '-c', f'{wait_and_open} || true'],
            output='screen', condition=IfCondition(view_cameras)),
    ])

    # mm_task: 顶层调度 (默认关, 调试时手动派命令; run_mission:=true 才自动整轮跑)
    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mm_task'),
                         'launch', 'mission.launch.py')),
        launch_arguments={'use_sim_time': 'false',
                          'mission_file': mission_file}.items(),
        condition=IfCondition(run_mission),
    )

    return LaunchDescription(args + [rviz, cams, mission])
