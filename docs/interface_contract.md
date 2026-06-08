# 接口契约 — 视觉感知与机械臂对接

> 本文档定义模块间接口,**先于实现**约定。坐标系部分待整车 URDF 定型后回填(标注 `待定`)。
> 分工:队友负责 ① 盒子识别输出抓取位姿 ② 车体相机识别 ArUco 输出 TF ③ 手眼标定参数填 URDF;
> 本人(架构/集成)负责 导航、MoveIt 抓取、任务编排。

---

## 1. 盒子抓取位姿(队友 → MoveIt 抓取)

| 项 | 约定 |
|---|---|
| 话题名 | `/perception/grasp_pose` |
| 消息类型 | `geometry_msgs/msg/PoseStamped` |
| 发布时机 | 收到任务层触发后识别并发布(或持续发布,由任务层取最新) |
| frame_id | **`base_link`**(队友在节点内完成 TF 转换,直接给底盘系;不要给相机系) |
| 位姿语义 | `position` = 盒子几何中心;`orientation` = 末端**垂直向下**接近(top-down),即工具 z 轴朝下指向盒子 |

说明:
- 选 `PoseStamped` 是因为 MoveIt `MoveGroupInterface.setPoseTarget()` 直接吃这个类型,零转换。
- 多目标场景后续再扩展为自定义数组消息;第一版单目标 `PoseStamped` 足够。
- 抓取姿态的精确定义(工具坐标系朝向)**待 URDF 末端 link 确定后回填**,届时统一 top-down 的四元数定义。

---

## 2. ArUco 定位 TF(队友 → 定位/任务层)

车体二维相机识别 ArUco,输出 TF,用于:**上电初始位姿标定** 和 **到点位置精矫正**。

| 项 | 约定 |
|---|---|
| 输出形式 | 向 `/tf` 广播 `TransformStamped` |
| 父坐标系 | 相机 link(`待定`,如 `front_camera_link`) |
| 子坐标系 | `aruco_<id>`(按标记 id 命名) |
| 标记物理尺寸 | `待定`(按实际打印尺寸,如 0.10m),作为节点参数可配 |
| 相机内参来源 | 订阅 `camera_info` 话题 |

下游用法(本人实现,队友无需关心):
- 初始位姿:已知 `map→aruco`(预先标定写死)+ TF 树 `aruco→base_footprint` → 反推 `map→base_footprint` → 发 `/initialpose` 给 AMCL。
- 到点精矫正:用 `aruco` 相对位姿做底盘伺服对位(替代开环平移)。

---

## 3. 手眼标定结果(队友 → URDF)

eye-in-hand(深度相机固连机械臂末端)。

| 项 | 约定 |
|---|---|
| 交付内容 | `arm_tool → camera_link` 的外参 `x y z roll pitch yaw`(6 个数) |
| 交付形式 | 填入 `mm_description` 整车 xacro 的一个 **fixed joint**(或临时用 `static_transform_publisher`) |
| 末端/相机 link 名 | `待定`(URDF 定型后给出准确 link 名) |

---

## 4. 机械臂控制接口(本人,记录备查)

| 项 | 约定 |
|---|---|
| 规划 | MoveIt2 `MoveGroupInterface`,输入 `PoseStamped`(见 §1) |
| 执行 | `FollowJointTrajectory`(ros2_control,仿真与真机一致) |
| 末端执行器 | 气泵吸取,I/O 接口 `待定` |

---

## 待定项清单(URDF 定型后回填)
- [ ] 车体二维相机 link 名(§2 父坐标系)
- [ ] 机械臂末端 tool link 名(§1 姿态定义、§3 父坐标系)
- [ ] 深度相机 camera_link 名(§3 子坐标系)
- [ ] top-down 抓取姿态的精确四元数定义(§1)
- [ ] ArUco 物理尺寸与各标记 id 分配(§2)
- [ ] 气泵 I/O 接口(§4)
