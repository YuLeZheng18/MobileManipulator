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
3. rplidar_ros        → /scan
4. RGB 相机驱动        → /camera/color/image_raw + /camera/color/camera_info
5. 深度相机驱动        → /camera/depth/...(点云,eye-in-hand 抓取用)
   (车体 ArUco 相机 Link_13 若独立,在此一并启动)
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
      object_detector    → /perception/object_pose(盒子顶面中心 xyz+yaw)
      aruco_localizer    → 广播 aruco_<id> TF
      grasp_node(抓取位姿转换,本人侧)
16. mm_task 状态机         最后启动,调度以上全部
```

### 7.2 状态机运行流程

```
[S0 初始化定位]
  aruco_localizer 识别车体相机看到的 ArUco
  → 沿 TF 树 aruco→base_footprint + 预写死 map→aruco → 反推 map→base_footprint
  → 发 /initialpose 给 AMCL → 收敛,此后 map→odom 由 AMCL 持续维护

[S1 导航到货架]
  mm_task 调 lane_navigator → Dijkstra 出路网路径 → 拆成 spin + follow_path 逐段执行
  → MPPI 读 /odom + /scan 发 /cmd_vel → ESP32 → 底盘运动,中途 MPPI 实时避障(vy 横移)

[S2 到点精对位]
  车体 ArUco 相机看货架标记 → 算相对位姿 → 底盘伺服微调 /cmd_vel 对准(替代开环,防漂移)

[S3 识别货物]
  mm_task 按预设搬运顺序触发 object_detector → 发 /perception/object_pose(base_link 系)

[S4 抓取] —— 三段混合,对机械臂零位偏差脱敏(详见 interface_contract.md §5)
  ① 粗定位(闭环,MoveIt 规划):grasp_node 取 object_pose → TCP 规划到盒子上方预抓取位(约 10–15cm)
  ② 精修(闭环,moveit_servo 视觉伺服):持续读新鲜 object_pose → Cartesian jog 对准顶面中心 xy+yaw → 逼近到约 5–8cm
  ③ 末段(开环,相对当前姿态短距离直插):沿吸盘接近轴(suction_tip, Link_29 -Z)相对下插固定行程
  → 气泵 I/O 吸取 → 规划到 tray(Link_11)上方 → 释放 → 放下
  ⚠️ 纪律:末段严禁用 FK 重算盒子 base_link 绝对坐标再 setPoseTarget,否则零位偏差加回末端

[S5 循环]
  回 S1 导航到下一货架 → 直到搬运序列完成
```

### 7.3 分层:任务列表 vs 调度系统
- **第一版**:状态机吃一个**写死的任务列表**(货架序列),把 S1~S5 跑通。
- **后续**:上面再加一层**调度系统**,负责按货物流动动态生成任务列表喂给状态机;状态机本身不改。
- 两层解耦:先做确定性的执行层,再叠智能的决策层。
