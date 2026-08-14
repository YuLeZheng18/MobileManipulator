# MobileManipulator

移动机械臂(底盘 + 机械臂)ROS 2 工作空间:自主导航 + 视觉感知 + MoveIt 抓取的整机方案,支持 Gazebo 仿真与实机(Jetson Orin + ESP32-S3 下位机)双主线。

## 功能概览

- **导航**:Nav2 + AMCL 定位,车道式路网导航
- **感知**:车体相机 ArUco 定位、深度相机 eye-in-hand YOLO 盒子识别、手眼标定
- **抓取**:MoveIt 2 规划 + 三段式抓取(闭环粗定位 → 视觉伺服精修 → 相对直插)
- **任务编排**:mm_task 状态机串联导航/感知/抓取全流程
- **实机下位机**:ESP32-S3 + micro-ROS 四轮里程计与电机控制固件

## 目录结构

| 目录 | 说明 |
|---|---|
| `mm_description` | 整车 URDF / 网格 / 坐标系定义 |
| `mm_bringup` | 聚合启动(仿真/实机)、mock 感知节点 |
| `mm_navigation` | Nav2 配置与车道导航节点 |
| `mm_perception` | 视觉感知:ArUco 定位、YOLO 盒子识别、手眼标定 |
| `mm_grasp` | MoveIt 抓取节点(`grasp_node`)、抓放位姿标定(`config/place.yaml`)、手柄示教 |
| `mm_task` | 任务状态机(`mission_manager`),串联导航/感知/抓取全流程 |
| `arm` | 机械臂 MoveIt 配置(`arm_moveit_config`)与 CAN 控制桥(`arm_control`) |
| `pymoveit2` | 第三方 MoveIt 2 Python 封装(vendored,BSD) |
| `third_party` | 第三方驱动(vendored):`rplidar_ros` |
| `firmware` | ESP32-S3 底盘固件(micro-ROS / PlatformIO) |
| `docs` | 接口契约、系统架构文档与排障脚本(`docs/scripts/`) |

## 文档

- [`docs/interface_contract.md`](docs/interface_contract.md) — 模块间接口契约(话题/TF/坐标系)
- [`docs/system_architecture.md`](docs/system_architecture.md) — 系统架构、数据流、启动顺序、状态机
- [`docs/pcan_setup.md`](docs/pcan_setup.md) — 机械臂 CAN(PEAK)驱动安装
- [`docs/real_machine_test_log.md`](docs/real_machine_test_log.md) — 实机测试记录

### 排障脚本 `docs/scripts/`

真机调试期沉淀的独立诊断工具,每个脚本头部写明了判读方法:

| 脚本 | 用途 |
|---|---|
| `wheel_diag.py` | 间歇性单轮失效定位(纯平移时四轮速度代数和应为 0,失衡即报假 wz) |
| `odom_check.py` | 里程计标定复核(EKF `/odom` 与轮式 `/wheel_odom` 位移/航向累计对比) |
| `imu_probe.py` | IMU yaw 异常分型(持续零偏漂移 vs 离散跳变) |
| `scan_probe.py` | 雷达扫描角度/盲区实测 |
| `lane_graph_check.py` | 路点净距校验(EDT 精确距离变换,判断节点能否原地自转) |
| `lane_graph_viz.py` | 路点表可视化 |

## 环境

- ROS 2(Humble)、MoveIt 2、Nav2
- Jetson Orin(JetPack 6),YOLO 推理走系统 GPU torch(见 `mm_perception/mm_perception/_tv_stub.py` 的 torchvision 兜底)
- Gazebo(仿真主线)

## 构建

```bash
cd <workspace>
colcon build
source install/setup.bash
```

## 许可

本仓库自研代码采用 MIT 许可(见 [`LICENSE`](LICENSE))。
第三方 vendored 包保留各自许可:`pymoveit2`(BSD)、`arm` 内含 Apache-2.0 组件。
