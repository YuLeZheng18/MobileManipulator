# 系统架构与数据流 — 仿真主线 + 实机支线

> 配套文档:`interface_contract.md`(队友接口)。本文件讲整体原理、TF 树、话题/服务/动作、实机下位机。
> 目的:让架构"讲得出来",作为开发施工图。具体 link 名待整车 URDF 定型后回填(标 `待定`)。

---

## 0. 三种通信机制(理解全局的钥匙)

| 机制 | 用途 | 例子 |
|---|---|---|
| 话题 topic | 连续数据流,发后不管 | `/scan` `/odom` `/cmd_vel` `/imu` |
| 服务 service | 即时请求-响应 | 计算 IK、存地图、清代价地图 |
| 动作 action | 长耗时、带反馈、可取消 | 导航到点、机械臂规划执行 |

**关键认知:任务层主要靠 action 驱动各模块**(导航和抓取都是耗时任务)。

---

## 1. TF 树(目标态)

```
map ─(AMCL 定位)→ odom ─(里程计/EKF)→ base_footprint → base_link
                                                          ├→ laser_link
                                                          ├→ front_camera_link (车体二维相机, ArUco用)
                                                          ├→ tray_link (托盘)
                                                          └→ arm_base → ... → arm_tool
                                                                                └→ camera_link (深度相机, eye-in-hand)
```

**TF 是一棵树:每个 link 只有一个父,不能成环。**

### ArUco 不在常驻 TF 链里 —— 这是最容易搞错的点
`base_footprint` 的父**永远**是 `odom`(里程计维护)。ArUco 不是 base 的父。
ArUco 的作用是"一次性观测 → 算出一个数值 → 喂给 AMCL":

- **已知**:`map→aruco`(标定位置,预先写死)
- **观测**:相机看到 aruco → `camera→aruco` → 沿 TF 树推 `aruco→base_footprint`
- **算出**:`map→base_footprint` 位姿
- **用法**:
  - 上电初始化:把算出的位姿发 `/initialpose`,**给 AMCL 一个初值**,发完即止,不持续连树
  - 到点精矫正:用 aruco 相对位姿做底盘伺服对位

之后 `map→odom` 仍由 AMCL 维护。Gazebo 的 `world` 坐标系导航栈不关心,从 `map` 往下看即可。

---

## 2. 话题/服务/动作全图(SLAM/Nav2/MoveIt 怎么结合)

### 传感器源头
```
/scan                     LaserScan   ← 雷达(真机:直连 Nano 的 rplidar_ros)
/wheel_odom               Odometry    ← 轮式里程计原始值(真机 ESP32-S3 编码器正解;仿真无此路)
/odom                     Odometry    ← 融合后里程计(仿真 planar_move 真值 / 真机 EKF 融合输出)
/imu                      Imu         ← IMU(真机 HWT906P 经下位机)
/camera/color/image_raw   Image       ← 相机
/camera/depth/...         Image/PointCloud
/tf, /tf_static                       ← 坐标变换
```

### SLAM 建图阶段(与导航不同时,二选一)
```
slam_toolbox: 订阅 /scan /odom /tf → 发布 /map + map→odom TF
→ 满意后存图(服务调用)→ 关闭 SLAM
```

### 导航阶段 Nav2
```
AMCL          订阅 /scan /map /tf + /initialpose → 发布 map→odom TF(定位)
planner       全局规划 → /plan
controller    订阅 /plan /odom /scan → 发布 /cmd_vel(局部控制,当前 DWB)
costmap       订阅 /scan → 维护 global/local 代价地图
[动作接口]    NavigateToPose / NavigateThroughPoses   ← 任务层调这个
```

### MoveIt 机械臂
```
move_group    订阅 /joint_states + 规划场景 → 输出规划轨迹
[动作接口]    /move_action,  FollowJointTrajectory  ← 你的抓取代码调
[服务]        /compute_ik, /plan_kinematic_path 等
moveit_servo  实时 Cartesian jog(收 TwistStamped/JointJog)← 抓取精修阶段闭环伺服用(S4 ②)
```

### 感知(队友,见 interface_contract.md)
```
/perception/object_pose  PoseStamped  ← 盒子识别(顶面中心+yaw, 4-DOF top-down) → MoveIt 抓取订阅
aruco_<id> 的 TF                       ← 车体相机(Link_13) → 你的定位/对位用
```

### 任务层 mm_task 编排
```
发 /initialpose、调 NavigateToPose(action)、调 MoveIt(action)、控气泵 I/O
把以上全部按状态机串起来
```

---

## 3. 两层"编排"别混淆

```
mm_task(你写的业务状态机:初始化→导航→对位→抓取→放托盘→搬运)
   │ 通过 action 调用
   ├──→ Nav2     (内部有自带行为树 BT,管单次导航的重规划/恢复 —— 你不碰)
   └──→ MoveIt   (内部管运动规划细节 —— 你不碰)
```

- **Nav2 行为树(BT)**:Nav2 自带,管"单次导航内部"逻辑(规划→跟踪→卡住重规划→恢复)。黑盒,调 action 即可。
- **mm_task**:你的业务状态机,在最上层,管"整个流程"。
- **Nav2 Simple Commander**:一个 Python 封装库,让 mm_task 调 Nav2 时一行 `goToPose()` 搞定,不用裸写 action client。**它不是另一个状态机,只是顺手工具。**

---

## 4. 仿真 → 实机:只换硬件抽象层

上面的 TF / 话题 / 动作结构,**仿真和实机完全一致**。区别只在最底层"谁来产生 odom 和接收 cmd_vel":

| 接口 | 仿真 | 实机 |
|---|---|---|
| `/cmd_vel` 接收 | Gazebo planar_move 插件 | ESP32-S3 下位机(micro-ROS 订阅) |
| `/odom` 发布 | planar_move(完美真值,自带 TF) | ESP32-S3 发 `/wheel_odom` → EKF 融合出 `/odom` + TF |
| `/scan` | Gazebo 雷达插件 | 思岚 A3 直连 Nano,rplidar_ros |
| `/imu` | Gazebo imu 插件 | HWT906P 经下位机发原始数据 |
| 机械臂 | ros2_control + Gazebo | ros2_control + CAN 驱动桥 |

**上面三层(任务/感知/规划)对此无感。** 这是分层架构的核心价值。

---

## 5. 实机下位机架构(ESP32-S3 + micro-ROS)

> 对应总规划 Phase 4。这是并行支线,主线先推进仿真。micro-ROS 你自己先找资料,这里给模块划分施工图。

### 5.1 职责边界(很重要)
**ESP32-S3 只管运动控制 + 自身板载传感器。雷达不走 ESP32。**

- 雷达(思岚 A3):**直连 Jetson Orin Nano 的 USB/串口**,跑 `rplidar_ros` 发 `/scan`。让 ESP32 转发雷达是给自己加负担(数据量大、实时性高),不要做。
- ESP32-S3 负责:四轮电机 PID 闭环、编码器里程计、IMU 读取转发、电源 ADC。

### 5.2 ESP32-S3 ↔ Nano 通信:micro-ROS
下位机直接当一个 ROS2 节点,收发话题:

```
ESP32-S3 订阅:
  /cmd_vel (Twist)             → omni 逆解 → 四轮目标转速 → PID 闭环

ESP32-S3 发布:
  /wheel_odom (Odometry)       ← 编码器测速 → omni 正解 → 积分位姿。⚠️ 只发消息,不发 TF
  /imu  (Imu)                  ← HWT906P 原始数据(角速度+加速度+姿态),不在板上积分
  /battery (BatteryState)      ← ADC 采电池分压电压
```

融合与 TF 由上位机独占:
```
robot_localization ekf_node:
  订阅 /wheel_odom + /imu → 卡尔曼融合 → 发布 /odom + odom→base_footprint TF
```
**纪律:`odom→base_footprint` 这段 TF 只有 EKF 能发,ESP32-S3 绝不发 TF,否则两边抢发 TF 树会跳变。**

### 5.3 全向轮正逆解 + PID(标准 mecanum/omni 运动学)
- **逆解(收 cmd_vel→轮速)**:由 (vx, vy, ωz) + 轮距参数算出四个轮子目标转速
- **PID 闭环**:编码器测每轮实际转速 → PID 调到目标转速
- **正解(轮速→odom)**:四轮实际转速 → 反算实际 (vx, vy, ωz) → 积分得位姿 → 发 /odom
- 这套我能帮你写(运动学 + PID 框架),真机调参(PID 整定)你在硬件上配合。

### 5.4 IMU:发原始,不在板上积分
- `sensor_msgs/Imu` 就是发原始的角速度 + 线加速度(+ HWT906P 自带的融合姿态四元数)。
- **积分成位姿的活交给上位机 robot_localization EKF**。板上自己积分会漂且无法和轮速融合。
- HWT906P 经串口(UART)读数据,填进 Imu 消息发出即可。

### 5.5 里程计标定(你问的"标定是什么")
轮速里程计"以为走了1米"和"实际走了1米"有系统误差(轮径/轮距不准)。
**标定 = 让车实际走/转固定距离,对比里程计读数,算修正系数填进固件。**
参考 m3pro `calibration` 包的 `calibrate_linear.py` / `calibrate_angular.py`。
流程:先标定让单纯轮速里程计尽量准 → 再上 EKF 融合 IMU(双保险防漂)。

### 5.6 电源监测
32 用 ADC 采电池分压电压 → micro-ROS 发 `/battery`(BatteryState 或 Float32)。加一个 publisher 即可。

### 5.7 FreeRTOS
- **ESP32-S3 原生跑 FreeRTOS**(ESP-IDF 底层就是 FreeRTOS),micro-ROS 用 `micro_ros_platformio` 集成,配 PlatformIO 工具链正合适。
- 任务划分参考:
  - 任务A:micro-ROS 通信(收发话题)
  - 任务B:电机 PID 闭环(高频,如 1kHz)
  - 任务C:采 IMU / ADC,组织消息
- 固件框架、micro-ROS 集成、任务划分我可以帮你写;烧录/看波形/调参你在硬件配合。

---

## 6. 上位机端架构(你说"跑通了但不知道怎么结合")

一句话:**各模块独立跑,靠话题/动作连接,mm_task 在顶层用 action 串。**

启动逻辑(mm_bringup 聚合):
```
1. 机器人描述   robot_state_publisher(URDF→TF) + Gazebo/真机驱动
2. 定位         先 SLAM 建图存图;之后导航用 AMCL + 已存地图
3. 导航         Nav2(amcl/planner/controller/bt/costmap)
4. 机械臂       MoveIt move_group + ros2_control
5. 感知         mm_perception(ArUco + 抓取识别)
6. 任务         mm_task 状态机(最后启动,调度以上全部)
```

信息流闭环示例(一次取放):
```
mm_task: ArUco初始化 → /initialpose 给 AMCL
mm_task: goToPose(货架) → Nav2 → /cmd_vel → 底盘 → 到位
mm_task: 触发感知 → /perception/object_pose(抓取期间连续发布)
mm_task: 抓取三段(粗定位闭环→精修伺服→末段相对直插,见 §7.2 S4)→ 气泵吸 → 放 tray
mm_task: goToPose(目标货架) → 放下 → 循环
```

---

## 7. 真机全链路启动顺序与状态机(施工图)

> 与第 6 节仿真侧对应。核心纪律:**严格自底向上启动,上层依赖下层的话题/TF/action 已就绪,状态机永远最后起。**

### 7.1 启动顺序(六阶段)

**阶段 A — 硬件层桥接(最先,其它都依赖它)**
```
1. micro_ros_agent (Jetson 上)   ← 起了它,ESP32 的 micro-ROS client 才连入 ROS 图
     ESP32 节点这才可见:订阅 /cmd_vel、气泵 I/O;发布 /wheel_odom、/imu、/battery
     注:ESP32 固件上电即跑,但不 agent 先行则话题不可见,不需要单独 run 固件
2. CAN 驱动桥 (arm_control/can_bridge)   连机械臂 CAN 总线 → 暴露 ros2_control 硬件接口
```

**阶段 B — 传感器驱动**
```
3. rplidar_ros        → /scan (frame Link_12)
4. 车体两路 USB 相机 (mm_perception/cameras.launch.py):
     cam_a (Link_13, ArUco) / cam_b (Link_14, 监视), 均装反。
     每路**只起 usb_cam**, 发 /cam_x/image_raw(+/compressed) + camera_info, 到此为止。
     ⚠️ 本 launch 不做转正 (2026-08-03/08-04 精简)。转正有两个各自独立的去处:
        看画面 -> web_video_server 的 invert=1 服务端转正 (见 §7.4)
        ArUco  -> aruco_real.launch.py 自带 image_rotator -> /cam_a/image_rot
     故 cam_x_rotation 参数已不起作用, 仅保留签名。
5. 手眼深度相机 D435i (Link_30, 同上 launch): realsense2_camera
     ⚠️ align_depth.enable **刻意关掉** (2026-08-03 起, 与早前文档相反):
        本机彩色内参硬件层就坏 (rs-enumerate-devices -c 每个彩色分辨率 PPX/PPY = -nan),
        对齐图全废。故 yolo_box_detector 改走 use_raw_depth=true, 直接吃
        depth/image_rect_raw + 深度模块内参 (fx=fy=428.403) 反投影。
        开着则驱动每秒 30 次把 848x480 深度重投影到 1280x720 彩色画幅, 而
        /aligned_depth_to_color/image_raw 实测**订阅者 0** —— 无人订阅的计算没必要做。
        ⚠️ 但别指望它省 CPU: 实测 realsense 节点 67.9% -> 79.6%, 并没有下降。
        真要用对齐图, 前提是先把彩色内参标好。
     pointcloud.enable:=false —— 点云既不过网也不在机上白算.
```

**阶段 C — 状态估计 + 机器人描述**
```
6. robot_state_publisher   读 mm_robot.urdf → 发 /robot_description + 各 link 固定 TF
7. ekf_node (robot_localization)
     订阅 /wheel_odom + /imu → 融合 → 发 /odom + odom→base_footprint TF(独占此段 TF)
```

**阶段 D — 定位与导航**
```
8. (前置离线一次) slam_toolbox 订阅 /scan /odom → 建图 → 存 .pgm/.yaml。任务时不跑 SLAM
9. map_server         加载已存地图 → 发 /map
10. AMCL              订阅 /scan /map /tf + /initialpose → 发 map→odom TF
11. Nav2 栈           planner_server / controller_server(MPPI) / behavior_server / bt_navigator
12. lane_navigator    车道导航节点(调 spin + follow_path action)
```

**阶段 E — 机械臂规划执行**
```
13. controller_manager + JTC   ros2_control,经 CAN 桥驱动真实关节
14. move_group (MoveIt)         订阅 /joint_states + 规划场景 → 提供 FollowJointTrajectory
    (真机无头运行,RViz 不启动)
```

**阶段 F — 感知 + 任务**
```
15. mm_perception:
      yolo_box_detector  → /perception/object_pose(盒顶中心 xyz+yaw)
                         + /object_poses(可抓盒数组, 长度=剩余可抓数)
                         + /object_point_cam(相机系, ②精修闭环追它)
                         + /object_axis_angle(长轴图像角, 朝向对齐判据)
                         + /object_class + /object_thickness
        必须走它自己的 launch (要把 yaml 里的相对模型名拼成 share 绝对路径), 别裸 ros2 run
      aruco_localizer    → 广播 aruco_<id> TF
15b. mm_grasp:
      servo_node + grasp_node
      → /grasp/execute 一轮抓放, /grasp/look 摆看货姿势并回报可抓数,
        /grasp/ready 收身, /grasp/unload_tray 整盘卸货, /grasp/reset_stack 清堆叠状态
      (servo_node 仍起着但抓取主链路已不用它, 见 interface_contract §5 实机修正①)
16. mm_task 状态机         最后启动,调度以上全部
```

一条命令起完(整机自主):
```
ros2 launch mm_bringup real_bringup.launch.py use_cameras:=true use_perception:=true \
    run_mission:=true mission_file:=<mm_task share>/config/mission_real.yaml
```
三个开关连带: `use_cameras` 出图 → `use_perception` 出位姿 → `run_mission` 串流程,少一个抓取跑不了。

### 7.2 状态机运行流程

```
[S0 初始化定位]
  发 /initialpose (mission.yaml 里配的已知位姿) 给 AMCL
  → 循环补发并等 map->base_link 出现才放行 (AMCL 的 /initialpose 订阅是 VOLATILE,
    建 publisher 后立即连发会赶在发现完成前被丢; 且没等收敛就进 S1 会被 lane_navigator
    判 None -> 整轮误判 FAILED)
  → 调 /grasp/reset_stack 清抓取堆叠状态与残留碰撞体 (上一轮记的"盒还在托盘上"会让
    本轮一开抓就 TRAY_FULL; 残留 placed_* 还会挡住放置直下段)
  → 调 /grasp/ready: 底盘行进前先收身, 不拖着伸出的臂走
  (原设计走 ArUco 反推初值: aruco→base_footprint + 预写死 map→aruco → 反推 map→base_footprint.
   当前是配置已知位姿的简化版, ArUco 那条路留待 S2 精对位一起做.)

[S1 导航到货架]
  mm_task 调 lane_navigator → Dijkstra 出路网路径 → 拆成 spin + follow_path 逐段执行
  → MPPI 读 /odom + /scan 发 /cmd_vel → ESP32 → 底盘运动,中途 MPPI 实时避障(vy 横移)

[S2 到点精对位]  ⚠️ 当前是 no-op 直接放行, 待做
  设计: 车体 ArUco 相机看货架标记 → 算相对位姿 → 底盘伺服微调 /cmd_vel 对准(替代开环,防漂移)

[S3 识别货物]
  等 /perception/object_pose 出"新鲜且够得着"的帧 (age<1s 且 |x|<0.5 |y|<0.6, base_link 系)
  —— 不是抓第一帧: 太远的帧意味着盒还没进可达范围, 粗定位残差大到精修收不回

[S4 抓取] —— 分段混合,对机械臂零位偏差脱敏(详见 interface_contract.md §5)
  S3a 先调 /grasp/look 摆看货姿势 (ready + J1+90°, 让相机转向货物),
      服务 message 回报 "pickable=N, tray_free=M" —— 状态机据此决定这个货架抓几次
  然后对每个盒调一次 /grasp/execute, grasp_node 内部:
  ① 粗定位(闭环,MoveIt 规划):取 object_pose → TCP 规划到盒上方 12cm
  ② 精修(闭环,相机系笛卡尔步进):追 /object_point_cam 的横向 (y,z) 到标定目标值, 容差 3mm
     (原设计的 moveit_servo + base_link 坐标已放弃, 理由见契约 §5 实机修正①)
  ②b 朝向对齐(闭环,转腕):追 /object_axis_angle 的图像角到标定常数, 容差 1°
  ③ 末段(开环,相对当前姿态短距离直插):沿吸盘接近轴(suction_tip, Link_29 -Z)相对下插
  → 气泵吸取(/pump_cmd=1, 吸住后发 0 关泵靠密封保负压)
  → 按类别映射到对应托盘 → 爬到 transit 高度 → 垂直直下到该层释放高度 → 脉冲式开阀释放
  → **收尾停在 look 位**(不回 ready): 下一轮直接就能识别, 省一趟空行程
  ⚠️ 纪律:末段严禁用 FK 重算盒子 base_link 绝对坐标再 setPoseTarget,否则零位偏差加回末端

[S4' 同一货架连抓] —— 一个货架有 1~4 个盒, 几个只有识别了才知道
  故不在 mission.yaml 里按盒数写死多条任务, 而是一条 grasp 任务内部循环 detect+execute.
  三个出口, 前两个都算**成功**(后续卸货任务照常跑, 不中止序列):
    ① pickable 归 0 —— 这个货架抓空了
    ② 托盘装满 —— execute 返回 "TRAY_FULL:..." 或 tray_free=0, 该去卸货了, **不是故障**
       判据是本类别映射到的那个盘满没满, 不是总余量: 4 个同类盒只进一个盘(容量 2),
       装 2 个就得走, 另一个盘空着也没用
    ③ 达到 max_picks_per_shelf 上限 —— 纯防死循环兜底
  离开货架前才调 /grasp/ready 收身(底盘不拖着伸出的臂走)—— "要不要走"只有状态机知道

[S4'' 卸货 /grasp/unload_tray] —— 整盘卸到地面两个卸货点, 平铺不堆叠, 全程不用视觉
  为什么不用视觉: 托盘上的盒高于感知端 pick_z_max 会被整批滤掉, 走视觉根本拿不到目标.
  而取盒目标其实已经知道 —— 就是当初放它时的吸盘释放位姿(grasp_node 运行时存着),
  取放走完全对称的同一套几何, "放得进去就取得出来". 顺序后进先出(先取栈顶).
  卸哪个盘由参数传 (Trigger 带不了数值): 状态机先 SetParameters 设 unload_tray, 再调服务.
  空盘返回"空的"按跳过处理不算失败 —— 这一轮装了几个盘取决于货架上有几个盒.

[S5 循环]
  回 S1 导航到下一货架 → 直到搬运序列完成
```

### 7.2b 状态机与 grasp_node 的职责切分(实机调试后定型)

| 谁做 | 做什么 |
|---|---|
| `mission_manager`(状态机) | **只编排**: 何时导航、何时看货、抓几次、何时收身、何时去卸货 |
| `grasp_node`(C++) | **只执行**: 一次调用完成一个完整动作, 无状态机语义, 不知道任务序列 |

两条纪律:
- **臂姿收尾归状态机管**。`/grasp/execute` 收尾停在 look 位而不是 ready —— 因为连抓时下一轮
  还要在 look 位识别。收身(`/grasp/ready`)由状态机在**真要走**时才调。
  ⚠️ 别为"图省事"把 execute 收尾改回 ready: 状态机侧的同货架循环依赖它停在 look。
- **grasp_node 通过 message 回报机器可读状态**(`pickable=N`/`tray_free=M`/`TRAY_FULL:` 前缀),
  而不是靠 success 布尔。状态机要分得出"托盘满了该去卸货"(正常转移)和"规划失败"(真故障),
  这两种在 `Trigger` 里都是 `success=false`。
  解析不出这些键时状态机按 **-1(未知)** 处理而不是 0 —— 报 0 会让连抓循环立刻收工。

### 7.3 分层:任务列表 vs 调度系统
- **第一版**:状态机吃一个**写死的任务列表**(货架序列),把 S1~S5 跑通。
- **后续**:上面再加一层**调度系统**,负责按货物流动动态生成任务列表喂给状态机;状态机本身不改。
- 两层解耦:先做确定性的执行层,再叠智能的决策层。

### 7.4 分布式调试部署(Nano ↔ 本机分工 + 数据流)

真机调试时**跨局域网双机跑**:ROS2 底层 DDS 天生支持多机,两机接同一 LAN、设**相同 `ROS_DOMAIN_ID`**、同 RMW,节点自动发现,话题/TF/服务/action 跨机透明,无需任何桥接代码。

**分工原则(一句话):摸硬件的 + 高带宽原始数据 + 延迟敏感闭环 → 就地在 Nano 跑;人看的可视化 + 粗粒度调度命令 → 本机跑。**
一个节点满足以下任一条即必须在 Nano:(a) 直连硬件(USB/串口/CAN);(b) 与硬件构成紧实时闭环(底盘控制环、视觉伺服);(c) 吃高带宽原始流、只吐小结果(把相机/点云留在本地)。

| 节点 | 位置 | 原因 |
|---|---|---|
| micro_ros_agent | **Nano** | (a) USB 串口连 ESP32 |
| arm ros2_control + JTC + can_bridge | **Nano** | (a)(b) 100Hz CAN 实时环 |
| rplidar_ros / 相机驱动 | **Nano** | (a) USB;相机 (c) 高带宽 |
| robot_state_publisher | **Nano** | 产 URDF 静态 TF + /robot_description,大家的地基 |
| ekf_node (robot_localization) | **Nano** | (c) 吃高频 /wheel_odom+/imu,产 /odom + odom→base_link TF |
| mm_perception (object_detector / aruco_localizer) | **Nano** | (c) 吃相机原始流,只吐 /perception/object_pose(小)+ aruco TF |
| moveit_servo (servo_node) | **Nano** | (b) 视觉伺服紧闭环,过 WiFi 抖 |
| grasp_node | **Nano** | (b) 与 servo/object_pose 紧耦合 |
| Nav2 栈(amcl/planner/controller/behavior/bt)+ lane_navigator | **Nano** | (b) 控制环 scan→cmd_vel;WiFi 掉线也不断驱动环 |
| cmd_vel_smoother | **Nano** | (b) 末端安全:WiFi 掉线时仍在机上斜坡归零 |
| **RViz** | **本机** | 人看的可视化;只读消费 Nano 的 TF/场景 |
| **web_video_server** | **Nano** | 三路画面转 HTTP/MJPEG;本机用**浏览器**看,不经 ROS(见 §7.4) |
| **浏览器 (monitor.html)** | **本机** | 三路监视画面;**本机不起任何图像 ROS 节点** |
| **mm_task 状态机** | **本机** | 粗粒度调度(发 /go_to /initialpose、调 /grasp/*,全是小消息) |

**跨 LAN 数据流(只有这些过 WiFi):**
```
本机 → Nano (小消息, 粗指令):
  /go_to(String)              本机 mm_task → Nano lane_navigator
  /initialpose(latched)       本机 mm_task → Nano AMCL
  Trigger 服务: /grasp/look /grasp/execute /grasp/ready /grasp/unload_tray /grasp/reset_stack
  /grasp_node/set_parameters  设 unload_tray=<N> (Trigger 带不了数值)
Nano → 本机 (只读可视化 + 状态回报):
  /tf /tf_static /robot_description            → 本机 RViz
  /scan /map /global_costmap /local_costmap /odom /plan /arm_joint_states  → 本机 RViz
  /lane_navigator/status(String)   → 本机 mm_task(S1 完成回报)
      格式 "<seq> <target>:SUCCEEDED|FAILED", seq 从 1 起单调递增。
      ⚠️ 该话题**刻意不用 latched(TRANSIENT_LOCAL)**: 终态是事件不是状态, 锁存会把上一轮
         终态重放给晚订阅者 -> 状态机重启后 S1 瞬间假成功、车没动就进 S3。订阅方必须按
         seq 过滤"下发目标之前已见的终态", 不能只比字符串。
  /perception/object_pose(PoseStamped, 小)  → 本机 mm_task(S3 判新鲜可达)

⚠️ **图像一律不走 DDS 过网** (2026-08-04 定案)。三路画面走 Nano 上 web_video_server 的
   HTTP/MJPEG, 浏览器直连 —— 那是普通 TCP, 不在上面这张 ROS 话题表里。详见 §7.4。
```
**在 Nano 本地闭环(绝不过 WiFi,这就是分割的意义):**
```
相机原始图/点云 → object_detector/aruco_localizer          (感知闭环就地)
/scan → Nav2 controller → /cmd_vel_nav → smoother → /cmd_vel → agent → ESP32  (驱动闭环就地)
ESP32 → agent → /wheel_odom + /imu → EKF → /odom + TF        (状态估计就地)
object_pose + servo TwistStamped → servo_node → 稠密关节指令 → JTC → can_bridge → CAN  (抓取闭环就地)
```

**启动方式:**
- 分布式调试:Nano 起 `ros2 launch mm_bringup nano_bringup.launch.py`;本机起 `ros2 launch mm_bringup dev_bringup.launch.py`(默认只 RViz,手动派命令;`run_mission:=true` 才自动整轮跑)。
- 单机整机自主(不拆分):`ros2 launch mm_bringup real_bringup.launch.py run_mission:=true`(无头,mm_task 与全栈同机)。`nano_bringup` 本质即 `real_bringup` 强制 `run_mission:=false`(mm_task 移到本机),二者共用同一实现不漂移。

**四个必须注意的坑:**
1. **发现机制**:默认 DDS 走多播,不少 WiFi/路由屏蔽多播 → 两机发现不了。确认路由放行多播,或配 Fast-DDS Discovery Server(单播)。有线网基本无此问题。
2. **时钟同步**:两机必须 NTP/chrony 对时。TF 用时间戳,钟一漂 tf2 立刻报 extrapolation、Nav2/MoveIt 全乱(实机 `use_sim_time=false`,靠真实钟)。
3. **DOMAIN_ID / RMW 一致**:`ROS_DOMAIN_ID` 与 `RMW_IMPLEMENTATION` 两机不一致就互相看不见。
4. **带宽/延迟兜底**:感知与所有闭环已就地在 Nano,过网的只剩 TF/可视化/小指令。

   **⚠️ 铁律:本机(或任何跨机进程)绝不订阅图像话题。看画面开浏览器。**(2026-08-04 定案)

   这是长期"画面卡顿"的**唯一真凶**,且它会连带把整条链路上所有流一起拖垮:
   `usb_cam` 的 `image_transport::CameraPublisher` 同时发 `image_raw`(未压缩) 和
   `image_raw/compressed`。`image_raw` 是普通 ROS 发布者,**跨机订它**时 DDS 就把
   640x480 rgb8 @30Hz ≈ 27MB/s 推上 WiFi,两路直接打满。
   判据(不是推测):停掉两个 usb_cam,Orin 网卡发送量 **15312 KB/s → 24 KB/s**。

   **正确做法** —— Nano 上 `web_video_server` 转 HTTP/MJPEG,浏览器直连:
   图像流全留在 Nano 机内(机内订阅不过网),过网只有一条普通 TCP,丢包由浏览器扛。
   本机 `dev_bringup.launch.py` 默认 `xdg-open` 打开 `mm_bringup/web/monitor.html`,
   三路 URL/画质/invert 都写在那个 html 里。
   实测三路并发(320x240 quality=60,臂栈+yolo+servo 同跑):各 29.9/29.6/28.1 fps,
   本机网卡合计 1041 KB/s,Orin load 4.03/6 核;与机上 camera_info 测得的采集率一致。
   - 车体两路装反,URL 加 `invert=1` **服务端**转正 —— 本机不必起 image_rotator。
   - 别把分辨率往上调:640x480 要 3.6~5.4MB/s,且三路里总有一路掉到 20fps(争抢)。
     要画面大就靠浏览器/CSS 拉伸,MJPEG 就是张 `<img>`,拉伸不花带宽也不花 Orin 算力。
   - 别从 web_video_server 首页点链接:那些链接**不带缩放参数**,点进去是原生
     1280x720,实测 5566 KB/s 一路就吃掉大半 WiFi,表现就是"打不开"。
   - `type=ros_compressed` 是零编码开销的原样转发,但**忽略 width/height/quality**
     (实测 2.9MB/s 一路)。为这 10 倍带宽差,宁可让 Nano 多编一次。

   **⚠️ 测量方法论**(三条早前的错误结论都是量错导致的):
   - 量跨机带宽只能用 `cat /sys/class/net/<iface>/statistics/rx_bytes` 前后差。
     `ros2 topic bw/hz` **自己就是订阅者**,而 DDS 单播是每订阅者一份独立拷贝 ——
     用它量跨机流量会把结果翻倍(早前那个"5.5 倍放大/重传"就是这么来的测量假象)。
   - 量真实**采集**帧率要在**机上**量 `camera_info`:它只有几十字节,与图像同一次
     publish,不受传输丢包和编码开销影响。量 `image_raw`(921KB/帧)量的是传输,不是相机。

   已作废的旧方案(别照着改回去):本机 `compressed_image_transport` + `republish` 解码 +
   `image_rotator` 转正 + `rqt_image_view`/`image_view` 三级链。它能出画面,但前提正是上面
   那条铁律禁止的事。`rqt_image_view` 还有个自身缺陷:它选 transport 的唯一入口是话题串
   本身,启动时选中的 compressed 会在后台刷新话题列表后**自己滑回 raw**,现场表现是
   "好好的突然全卡了"。**深度图不看**,深度 raw ~15MB/s 过网太重,`compressedDepth` 的
   republish 另有 bug。**点云绝不过网**。

### 7.5 双机部署 Checklist(施工卡片)

> 上真机双机调试照此走。**图像不进这张 ROS 话题清单** —— 三路画面走 Nano 上
> `web_video_server` 的 HTTP/MJPEG,浏览器直连(§7.4 的铁律)。**点云绝不过网**。

**① 装依赖**
```
# 仅 Nano(贴硬件运行时依赖, 刻意不写进 mm_bringup 的 exec_depend ——
#   写了会让本机没装这些驱动时整个 mm_bringup 编不过, 而本机压根不需要它们)
sudo apt install ros-humble-robot-localization ros-humble-rplidar-ros \
                 ros-humble-realsense2-camera ros-humble-usb-cam
# 仅 Nano: 画面转 HTTP/MJPEG(会连带装 ros-humble-async-web-server-cpp)
sudo apt install ros-humble-web-video-server
#   micro-ROS 代理: 独立 ws ~/microros_ws(用时 source)
# 本机什么图像包都不用装 —— 看画面用浏览器(§7.4)。
#   早前"两机都装 compressed_image_transport 给 rqt 看"已作废: 那条路正是卡顿的成因。
```

**② 网络 & 环境(两机必须一致)**
```
export ROS_DOMAIN_ID=42                      # 两机同一个数
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp   # 两机同一 RMW
```
- NTP 对时(chrony):不对时 → tf2 报 extrapolation、Nav2/MoveIt 乱。
- 多播可达(WiFi 常屏蔽 → 发现失败,改配 Fast-DDS Discovery Server 单播);优先有线/5GHz。

**③ CAN(仅 Nano,机械臂前置)**

⚠️ **不是 socketcan。** 走 PEAK 的 **PCAN 库**(`libpcanbasic` + 字符设备
`/dev/pcanusb32`,通道 `PCAN_USBBUS1`),`can_interface.py` 直接调库。
早前这里写的 `ip link set can0 up type can bitrate 1000000` **已过时且会误导** ——
`can0` 这个网络接口压根不存在,照着敲只会得到 "Cannot find device"。
```
ls -l /dev/pcanusb32          # 设备节点在 = 驱动已加载
cat /proc/pcan                # 看 read/write 计数: write 涨而 read 冻住 = 发出去没人回
```
波特率由 `robot_arm_config.json` 里的配置在建连接时下发,不用手工设。
⚠️ PEAK 驱动是外挂内核模块,**内核升级后会失效**(`pcan.ko` 留在旧版本目录),
表现是 `/dev/pcanusb32` 消失。需重装 9.2.0(必须指定 gcc-12,且 `make install`
不能跳);长期解法是改 dkms,尚未做。安装脚本见 `docs/scripts/install_pcan.sh`。

**④ 启动(Nano 先,本机后)**
```
# Nano(硬件+自主全栈, 无头). 抓取要跑就得开这两个开关
source ~/microros_ws/install/setup.bash && source install/setup.bash
ros2 launch mm_bringup nano_bringup.launch.py use_cameras:=true use_perception:=true
# 本机(可视化+调度). run_mission 默认关, 先手动派命令验; 开则默认吃 mission_real.yaml
source install/setup.bash
ros2 launch mm_bringup dev_bringup.launch.py view_cameras:=true
```

**⑤ 验证(本机上跑)**
```
ros2 node list                              # 看到 Nano 节点=发现 OK
ros2 topic hz /scan                         # 传感器过来了
ros2 run tf2_ros tf2_echo map base_link     # TF 跨机可用
ros2 topic pub /go_to std_msgs/String "{data: p2}"   # 手动派一段导航
# RViz 看模型/TF/costmap; 相机画面看浏览器(dev_bringup 会自动开 monitor.html)
# ⚠️ 别用 ros2 topic hz/bw 量图像话题: 它自己是订阅者, 一量就把图像流拉过网(§7.4)
```

**⑥ 相机监视要点**:rqt 打开后**必须在 Transport 下拉选压缩类型**(默认 raw 打满带宽);卡顿降 Nano 端分辨率/帧率;要点云在 Nano 本地看,不拉过网。

---

## 8. 仿真验证策略(mock 感知 / 假吸附 / 逻辑级全流程)

> 定义仿真里"验什么、不验什么",以及感知如何用 mock 顶替。核心:**仿真验运动与流程逻辑,感知与吸力留真机。**

### 8.1 分层原则:感知层不仿,逻辑层仿

| 层 | 仿真里怎么办 | 原因 |
|---|---|---|
| 感知层(AI 盒子识别 / ArUco 识别) | **不做**,用 mock 顶替 | 模型拿真实货物训练,sim-to-real 不迁移;仿真里重训一套是白工 |
| 逻辑/运动层(任务状态机 / MoveIt / 三段抓取 / 可达性) | **做** | 几何随 URDF 迁移,真机试错贵(撞架/掉货),仿真近零成本迭代 |

**关键认知:Gazebo 是上帝视角,自己知道每个物体真值位姿。仿真里没有"识别"难题**——识别是"从像素反推位姿",只在真机存在;仿真直接把真值当识别结果发出去。

### 8.2 mock 感知:同话题、同类型、假来源

mock 节点 = 把真感知节点的"读图像→AI 推理"整段省掉,**直接发真值到同一话题**。下游(grasp_node/定位)订阅同名话题,分不出真假(接口一致)。

| 真节点(队友) | mock 替身(本人,仿真用) | 输出话题(不变) |
|---|---|---|
| `object_detector`(读深度图 AI 推理) | `mock_object_detector`(查 Gazebo 盒子真值打包) | `/perception/object_pose` |
| `aruco_localizer`(读图像解 ArUco) | `mock_aruco`(发预设 `base_link→aruco_<id>` TF) | `aruco_<id>` TF |

**主用力度:放物体+喂真值(推荐)** —— 世界里**真放盒子模型**,不跑相机 AI,从 Gazebo 查该盒子真值发布 → 验臂对真实几何(可达/碰撞/抓取点)。

> 快速冒烟(只验状态机/规划链通不通)时可退化为**纯喂坐标**:世界不放物体,直发死坐标。
> §5 精修段要"新鲜观测",mock 需 **≥10Hz 连续发**;要测伺服鲁棒性可在真值上叠噪声。
> mock 节点放 `mm_bringup` 的 sim-only 目录,**不进队友的 `mm_perception`**;仿真 launch 起 mock,真机 launch 起真节点,上层无感(同 §4)。

### 8.3 假吸附:验抓放动作,不验吸力

Gazebo 不建模真空吸力。用**假吸附插件**(`gazebo_grasp_fix` 类):吸盘 TCP 接触盒子时打一个 fixed joint 把盒子"粘"住,放置时解开。

- **验的是**:抓→抬→移→放的**动作序列**和状态机时序。
- **不验**:真实吸力、能否吸住、保压够不够——只能真机测。

### 8.4 逻辑级全流程仿真(本人整合)

起现有 Gazebo(整车已 spawn)+ MoveIt,按 §7.2 状态机跑,**全程感知用 mock**:

```
[S0] mock_aruco 发预设 aruco TF → 反推 map→base → /initialpose 给 AMCL
[S1] lane_navigator 导航到货架(真跑 Nav2/MPPI,不 mock)
[S2] mock_aruco 发货架标记 TF → 底盘伺服对位(验对位控制律)
[S3] mock_object_detector 发盒子真值 → /perception/object_pose(≥10Hz)
[S4] grasp_node 三段抓取(真跑 MoveIt)→ 假吸附粘盒 → 放 tray
[S5] 回 S1 循环下一货架
```

- **不同货物**:换不同尺寸/位置/朝向的盒子模型 + 喂各自真值,测运动对多规格的鲁棒性(**不是**测 AI 分不分得清)。
- **单独调抓取**(不带导航):车原地不动,货架前放盒子,只起 S3~S4。

### 8.5 验收矩阵:仿真 vs 真机

| 能力 | 仿真验 | 只能真机验 |
|---|---|---|
| 任务状态机时序、S0~S5 串联 | ✅ | |
| MoveIt 规划链、三段抓取运动 | ✅ | |
| 货架前臂可达性/自碰撞/撞架 | ✅ | |
| ArUco→map→base 反推数学、伺服控制律 | ✅ | |
| AI 盒子识别精度(sim-to-real) | | ✅ 真货物 |
| 真实吸力/保压 | | ✅(假吸附只验动作) |
| ArUco 真实精度/光学系 90°/内参 | | ✅ |
| 手眼标定外参 | | ✅ |

**真机侧已跑通(2026-07-26~07-31,详见 `real_machine_test_log.md`):** 双托盘按类别堆叠放置、
相机系精修闭环收敛、图像角朝向对齐、整盘卸货到地面两点、已放盒碰撞体避让。
这批把仿真里"假吸附只验动作"的部分补成了真的 —— 也暴露出仿真验不到的一整类问题:
**手眼/零位误差随构型变**(逼着精修改用相机系)、**视觉管线延迟**(摆臂刚停时缓存里还是运动中的画面)、
**近正方形盒长短边互换**(θ_img 在 2θ 空间抵消出一个任何单帧都不存在的假值)。

### 8.6 分工(本轮更新)
- **本人**:逻辑级全流程仿真(nav+臂整合)、mock 感知节点(`mm_bringup`)、假吸附、grasp_node 三段抓取。
- **队友**:实机标定(手眼标定填 URDF §3、真货物检测模型训练、ArUco 现场 id 登记)。
