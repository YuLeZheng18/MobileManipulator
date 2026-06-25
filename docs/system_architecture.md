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
/odom                     Odometry    ← 里程计(仿真 planar_move / 真机 EKF 输出)
/imu                      Imu         ← IMU(真机 HWT101 经下位机)
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
| `/cmd_vel` 接收 | Gazebo planar_move 插件 | STM32 下位机(micro-ROS 订阅) |
| `/odom` 发布 | planar_move(完美真值) | STM32 编码器正解 → EKF 融合 |
| `/scan` | Gazebo 雷达插件 | 思岚 A3 直连 Nano,rplidar_ros |
| `/imu` | Gazebo imu 插件 | HWT101 经下位机发原始数据 |
| 机械臂 | ros2_control + Gazebo | ros2_control + CAN 驱动桥 |

**上面三层(任务/感知/规划)对此无感。** 这是分层架构的核心价值。

---

## 5. 实机下位机架构(STM32H7 + micro-ROS)

> 对应总规划 Phase 4。这是并行支线,主线先推进仿真。micro-ROS 你自己先找资料,这里给模块划分施工图。

### 5.1 职责边界(很重要)
**STM32 只管运动控制 + 自身板载传感器。雷达不走 32。**

- 雷达(思岚 A3):**直连 Jetson Orin Nano 的 USB/串口**,跑 `rplidar_ros` 发 `/scan`。让 32 转发雷达是给自己加负担(数据量大、实时性高),不要做。
- STM32 负责:四轮电机 PID 闭环、编码器里程计、IMU 读取转发、电源 ADC。

### 5.2 STM32 ↔ Nano 通信:micro-ROS
下位机直接当一个 ROS2 节点,收发话题:

```
STM32 订阅:
  /cmd_vel (Twist)         → omni 逆解 → 四轮目标转速 → PID 闭环

STM32 发布:
  /odom (Odometry)         ← 编码器测速 → omni 正解 → 积分位姿
  /imu  (Imu)              ← HWT101 原始数据(角速度+加速度+姿态),不在32积分
  /battery (BatteryState)  ← ADC 采电池分压电压
```

### 5.3 全向轮正逆解 + PID(标准 mecanum/omni 运动学)
- **逆解(收 cmd_vel→轮速)**:由 (vx, vy, ωz) + 轮距参数算出四个轮子目标转速
- **PID 闭环**:编码器测每轮实际转速 → PID 调到目标转速
- **正解(轮速→odom)**:四轮实际转速 → 反算实际 (vx, vy, ωz) → 积分得位姿 → 发 /odom
- 这套我能帮你写(运动学 + PID 框架),真机调参(PID 整定)你在硬件上配合。

### 5.4 IMU:发原始,不在 32 积分
- `sensor_msgs/Imu` 就是发原始的角速度 + 线加速度(+ HWT101 自带的融合姿态四元数)。
- **积分成位姿的活交给上位机 robot_localization EKF**。32 自己积分会漂且无法和轮速融合。
- HWT101 经 SPI 或串口 DMA 读数据,填进 Imu 消息发出即可。

### 5.5 里程计标定(你问的"标定是什么")
轮速里程计"以为走了1米"和"实际走了1米"有系统误差(轮径/轮距不准)。
**标定 = 让车实际走/转固定距离,对比里程计读数,算修正系数填进固件。**
参考 m3pro `calibration` 包的 `calibrate_linear.py` / `calibrate_angular.py`。
流程:先标定让单纯轮速里程计尽量准 → 再上 EKF 融合 IMU(双保险防漂)。

### 5.6 电源监测
32 用 ADC 采电池分压电压 → micro-ROS 发 `/battery`(BatteryState 或 Float32)。加一个 publisher 即可。

### 5.7 FreeRTOS
- **建议用,且 micro-ROS 在 STM32 上的标准形态就是跑在 FreeRTOS 上**(官方有 micro_ros FreeRTOS 模板)。
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
mm_task: 触发感知 → /perception/object_pose
mm_task: setPoseTarget → MoveIt 规划执行 → 气泵吸 → 放 tray
mm_task: goToPose(目标货架) → 放下 → 循环
```
