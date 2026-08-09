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
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
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

    # 看画面就靠这个: HTTP/MJPEG 服务器, 浏览器开 http://ubuntu.local:8080 即可。
    # 为什么不走 ROS 跨机订阅 (2026-08-04 定案, 折腾一整晚的结论):
    #   ROS/DDS 不是为跨 WiFi 传视频设计的 —— 可靠传输 + 每订阅者一份独立拷贝 +
    #   全网发现, 图像这种大流量在它手里必然打架。实测过的坑: 裸 raw 被跨机订走
    #   27MB/s 打满链路; ros2 topic bw 自己就是订阅者会把测量结果翻倍(那个"5.5 倍
    #   放大"是测量假象不是重传); rqt_image_view 会自己滑回 raw。
    #   HTTP 这条全绕开了: 图像流全留在 Nano 机内, 过网只有一条 TCP, 丢包由浏览器扛。
    # 实测 2026-08-04 (三路并发, 320x240, 臂栈+yolo+servo 同时在跑):
    #   cam_a 29.8fps/310KB/s, cam_b 29.8fps/420KB/s, D435i 彩色 29.6fps/264KB/s,
    #   笔记本网卡实收合计 1050KB/s, Orin load 3.04/6 核。与机上 camera_info
    #   (30.178/29.033/28.945Hz) 对得上 -> 这条链不丢帧。
    # ⚠️ 车体两路是倒装的, 加 invert=1 直接在服务端转正 —— 本机不用再起 rotator。
    # **要看画面只用记一个地址: http://ubuntu.local:8081/** (三路横向排列的监视页,
    # 由下面 monitor_page 那个 http.server 发出; 8080 是流本身, 一般不用直接开)。
    # 主机名用 ubuntu.local 不用 IP: Orin 的 wlP1p1s0 是 DHCP, IP 会变。
    #   http://ubuntu.local:8080/stream?topic=/cam_a/image_raw&width=320&height=240&quality=45&invert=1
    #   http://ubuntu.local:8080/stream?topic=/cam_b/image_raw&width=320&height=240&quality=45&invert=1
    #   http://ubuntu.local:8080/stream?topic=/camera/camera/color/image_raw&width=320&height=240&quality=60
    #   http://ubuntu.local:8080/                        <- 首页列出所有可用话题
    #   .../snapshot?topic=...                           <- 单帧 JPEG, 调参时比 stream 好使
    # ⚠️ 别从首页点链接: 那些链接不带缩放参数, 点进去是原生 1280x720, 实测 5566 KB/s
    #   一路就吃掉大半 WiFi, 表现就是"打不开"。首页只用来确认话题名。
    #
    # ⚠️⚠️⚠️ **`publish_rate: 15.0` 是消除卡顿的关键, 别删也别改回默认 -1。**
    # 默认 -1(不限速) 时画面**一顿一顿**: 帧成批涌出再停 200~500ms。
    # 2026-08-04 实测同一路 cam_a 各 30s:
    #   publish_rate=-1  -> 24.1 fps, 间隔 p50 **6**ms p90 157 max 576, >200ms **43 次**
    #   publish_rate=15  -> 30.1 fps, 间隔 p50 **34**ms p90  69 max 100, >200ms **0 次**
    # **限速反而更快**, 不是笔误: 不限速时它对每帧都立刻尝试写, 写不完就撞上
    # MultipartStream 的 `max_queue_size = 1` (硬编码, 见 /opt/ros/humble/include/
    # web_video_server/web_video_server/multipart_stream.hpp, 配 private `is_busy()`),
    # 后续帧被**直接丢弃而非排队**, 于是陷入"抢-丢-抢-丢"的自我干扰; 限速后每帧都有
    # 充足时间写完, is_busy() 几乎不再为真, 该发的帧全发出去了。经典拥塞控制现象:
    # 降低注入速率反而提高有效吞吐。p50 从 6ms 回到 34ms 就是"突发涌出"消失的判据。
    # (给 15 却实测出 30fps -> 它不是硬性限帧上限, 更像内部调度节拍。别嫌 15 小去调大,
    #  30fps 已经到手了。)
    #
    # ⚠️ 排查这类卡顿**别看平均帧率, 要看帧间隔分布** (p50/p90/max + >200ms 次数):
    # 平均值会把成串丢帧摊平 —— 上面那个卡得很难看的 case 平均仍有 24fps。
    # 我为此白查了五轮, 每层都有硬数据且**全都不是**原因, 别再重查:
    #   相机采集: 机上 camera_info p50 33ms max 95ms,  >200ms 0 次        -> 干净
    #   机内传输: 机上 image_raw(20.9MB/s) p50 33ms max 133ms, 0 次       -> 干净
    #   WiFi:     1200 个 ping p99 2.64ms max 9.78ms 0% 丢包              -> 干净
    #   带宽:     单路 56KB/s, 三路 176KB/s (跑过 1041KB/s 都没事)         -> 干净
    #   整机 CPU: 停掉 yolo(83%) 后 >200ms 空档 44 -> **53 次, 反而更差**  -> 排除
    # ⚠️ 最后一条**推翻**了本文件早前"帧率随整机 CPU 波动, yolo 是最大元凶"的说法。
    # (量 CPU 别用 `ps -eo pcpu` —— 那是进程生命周期**平均值**不是瞬时值, 我拿它当瞬时
    #  读过并据此误判 web_video_server 单线程饱和; 它空闲时其实是 0%。要瞬时值就
    #  top -H 看线程, 或 /proc/<pid>/stat 第 14+15 域做差分。)
    #
    # quality 逐路给(车体 45, 深度 60): 帧越大越容易撞 is_busy(), 但这只是**次要因素**
    # —— q45 把帧压到 2.0KB 后卡顿照旧, 真正解决靠的是上面的 publish_rate。
    #
    # ⚠️⚠️ **别给 server_threads** (2026-08-04 试过, server_threads:=4 直接 0 帧,
    # 连接立刻断), 日志:
    #   Unable to load plugin for transport 'image_transport/raw_sub' ...
    #   MultiLibraryClassLoader: Could not create object of class type
    #   image_transport::RawSubscriber as no factory exists for it
    # 三个线程同时加载 RawSubscriber 工厂全部失败 —— image_transport 的插件加载器不是
    # 线程安全的。这是 web_video_server 的类型缺陷, 不是参数给错。
    #
    # ⚠️ 也别给 type=ros_compressed 想省编码: 它是原样转发(零编码开销)但**忽略
    # 缩放/quality/invert**, 三路实测 12589 KB/s (D435i 一路 7.7MB/s), 帧率反而更低
    # (22.5/15.6/25.6) —— 省下的 CPU 被带宽吃回去了。
    web_video = Node(
        package='web_video_server', executable='web_video_server',
        name='web_video_server', output='screen',
        parameters=[{'port': 8080, 'address': '0.0.0.0',
                     'publish_rate': 15.0}],
        condition=IfCondition(use_perception),
    )

    # 监视页自己也走 HTTP (8081), 不让本机开 file://。
    # 为什么: 本机 `text/html` 的 mimetype 关联被代理客户端 mihomo-party.desktop 抢走了
    # (`xdg-mime query default text/html` 可复现), xdg-open 本地 html 会拉起那个代理软件
    # 而**永远不进浏览器** —— 退出码还是 0, 极具误导性 (判据: Orin 侧
    # `ss -tn '( sport = :8080 )'` 一个连接都没有)。而 `x-scheme-handler/http` 关联是好的,
    # 所以页面改由 Orin 用 http 发出就绕开了整个问题。注意 `xdg-settings get
    # default-web-browser` 报的是 firefox, 但那只管 http scheme, 与本地文件的 text/html
    # 是两套关联, 别被它骗。
    # 顺带的好处: 手机/平板同一个地址就能看; 页面只有一份(在仓库里), 不会有副本漂移。
    # 用标准库 http.server 是刻意的 —— 不引入任何依赖, 只发一个静态文件, 开销可忽略。
    # -d 指到 share/web/, 且页面文件名固定 monitor.html; 根路径要能直接出页面,
    # 故用 --directory 配 index.html 的替代: 直接访问 /monitor.html, 或根路径列目录。
    monitor_page = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8081', '--bind', '0.0.0.0',
             '-d', os.path.join(get_package_share_directory('mm_bringup'), 'web')],
        name='monitor_page', output='screen',
        condition=IfCondition(use_perception),
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz', output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(args + [
        micro_ros_agent, arm_real, depth_cam,
        TimerAction(period=6.0, actions=[move_group]),
        TimerAction(period=12.0, actions=[grasp]),
        TimerAction(period=18.0, actions=[yolo, web_video, monitor_page]),
        TimerAction(period=22.0, actions=[teleop, rviz]),
    ])
