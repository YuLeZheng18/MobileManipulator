"""三路相机监视页 (Nano 侧): web_video_server(8080) + 静态页 http.server(8081).

**只是"给人看画面"的一层, 不产任何 ROS 数据、不被任何节点依赖。** 相机驱动是另一条
(mm_perception/cameras.launch.py, real_bringup 已 include), 图像本来就在机内; 本 launch
只是把它们转成 HTTP/MJPEG 供浏览器取。

用法 (随用随起, 看完就 Ctrl-C):
  ros2 launch mm_bringup monitor.launch.py
然后开 http://ubuntu.local:8081/monitor.html (本机 dev_bringup 会自己轮询这个地址并跳转)。

⚠️ 为什么单独一条而不塞进 real_bringup (2026-08-13 定案):
  三路并发编码是实打实的 CPU 负载, 而 real_bringup 冷启动那两分钟 CPU 已经是瓶颈 ——
  实测 move_group 在 "Listening to planning_scene_world" 到 "Loading planning pipeline
  'ompl'" 之间静默 119s (Nano 上 dlopen OMPL/kinematics + 整车 URDF 建碰撞, 同时
  nav2_container/RealSense/两路 USB 相机在抢核), grasp_node 的 MoveGroupInterface 死等
  这一段, 连带 S0 的 reset_stack 排队 115s (原超时 120s 只剩 5s 余量)。
  看画面是"人要看才开"的事, 独立一条就不会在那个窗口里跟 move_group 抢核。
  **起栈头两分钟别开浏览器。**

为什么不走 ROS 跨机订阅 (2026-08-04 定案, 折腾一整晚的结论):
  ROS/DDS 不是为跨 WiFi 传视频设计的 —— 可靠传输 + 每订阅者一份独立拷贝 + 全网发现,
  图像这种大流量在它手里必然打架。实测过的坑: 裸 raw 被跨机订走 27MB/s 打满链路;
  ros2 topic bw 自己就是订阅者会把测量结果翻倍(那个"5.5 倍放大"是测量假象不是重传);
  rqt_image_view 会自己滑回 raw。HTTP 这条全绕开: 图像流全留在机内, 过网只有一条 TCP,
  丢包由浏览器扛。**铁律: 本机(或任何跨机进程)绝不订阅图像话题。**

实测 2026-08-04 (三路并发, 320x240, 臂栈+yolo+servo 同时在跑):
  cam_a 29.8fps/310KB/s, cam_b 29.8fps/420KB/s, D435i 彩色 29.6fps/264KB/s,
  笔记本网卡实收合计 1050KB/s, Orin load 3.04/6 核。与机上 camera_info
  (30.178/29.033/28.945Hz) 对得上 -> 这条链不丢帧。
⚠️ 测跨机带宽只能用 cat /sys/class/net/<iface>/statistics/rx_bytes 前后差; 量真实采集
  帧率要在**机上**量 camera_info (几十字节, 与图像同一次 publish, 不受传输影响)。

流地址 (车体两路倒装, invert=1 服务端转正, 本机不用再起 rotator):
  http://ubuntu.local:8080/stream?topic=/cam_a/image_raw&width=320&height=240&quality=45&invert=1
  http://ubuntu.local:8080/stream?topic=/cam_b/image_raw&width=320&height=240&quality=45&invert=1
  http://ubuntu.local:8080/stream?topic=/camera/camera/color/image_raw&width=320&height=240&quality=60
  http://ubuntu.local:8080/            <- 首页列出所有可用话题
  .../snapshot?topic=...               <- 单帧 JPEG, 调参时比 stream 好使
主机名用 ubuntu.local 不用 IP: 无线网卡是 DHCP, IP 会变; 两机都跑 avahi-daemon。
⚠️ 别从首页点链接: 那些链接不带缩放参数, 点进去是原生 1280x720, 实测 5566 KB/s 一路
  就吃掉大半 WiFi, 表现就是"打不开"。首页只用来确认话题名。
三路布局/分辨率/quality 写在 mm_bringup/web/monitor.html, 要改画面改那个文件。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
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
    # 为此白查了五轮, 每层都有硬数据且**全都不是**原因, 别再重查:
    #   相机采集: 机上 camera_info p50 33ms max 95ms,  >200ms 0 次        -> 干净
    #   机内传输: 机上 image_raw(20.9MB/s) p50 33ms max 133ms, 0 次       -> 干净
    #   WiFi:     1200 个 ping p99 2.64ms max 9.78ms 0% 丢包              -> 干净
    #   带宽:     单路 56KB/s, 三路 176KB/s (跑过 1041KB/s 都没事)         -> 干净
    #   整机 CPU: 停掉 yolo(83%) 后 >200ms 空档 44 -> **53 次, 反而更差**  -> 排除
    # ⚠️ 最后一条**推翻**了早前"帧率随整机 CPU 波动, yolo 是最大元凶"的说法。
    # (量 CPU 别用 `ps -eo pcpu` —— 那是进程生命周期**平均值**不是瞬时值, 拿它当瞬时读
    #  会误判 web_video_server 单线程饱和; 它空闲时其实是 0%。要瞬时值就 top -H 看线程,
    #  或 /proc/<pid>/stat 第 14+15 域做差分。)
    #
    # quality 逐路给(车体 45, 深度 60, 写在 monitor.html 的 URL 里): 帧越大越容易撞
    # is_busy(), 但这只是**次要因素** —— q45 把帧压到 2.0KB 后卡顿照旧, 真正解决靠
    # publish_rate。
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
    )

    # 监视页自己也走 HTTP (8081), 不让本机开 file://。
    # 为什么: 本机 `text/html` 的 mimetype 关联被代理客户端 mihomo-party.desktop 抢走了
    # (`xdg-mime query default text/html` 可复现), xdg-open 本地 html 会拉起那个代理软件
    # 而**永远不进浏览器** —— 退出码还是 0, 极具误导性 (判据: 机上
    # `ss -tn '( sport = :8080 )'` 一个连接都没有)。而 `x-scheme-handler/http` 关联是好的,
    # 所以页面改由机器人用 http 发出就绕开了整个问题。注意 `xdg-settings get
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
    )

    return LaunchDescription([web_video, monitor_page])
