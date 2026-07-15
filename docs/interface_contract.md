# 接口契约 — 视觉感知与机械臂对接

> 本文档定义模块间接口。整车 URDF 已定型(`mm_description/urdf/mm_robot.urdf`),坐标系已回填真实 link 名。
> 分工:队友负责 ① 盒子识别输出抓取位姿 ② 车体相机识别 ArUco 输出 TF ③ 手眼标定参数填 URDF;
> 本人(架构/集成)负责 URDF/Gazebo/MoveIt 集成、导航、MoveIt 抓取、任务编排。

## 整车 link 角色对照(统一术语)

URDF 沿用 CAD 原始命名(`Link_xx`),与角色对照如下:

| link | 角色 | 说明 |
|---|---|---|
| `base_link` | 底盘基坐标 | 抓取位姿、感知输出统一用此系 |
| `Link_13` | **ArUco 识别相机**(车体二维相机) | §2 ArUco 输出的父坐标系 |
| `Link_14` | 行驶监视相机 | 仅监控/遥操,**不进感知 pipeline** |
| `Link_12` | 雷达 | `/scan` 来源 |
| `Link_11` | 托盘 tray | 抓到的盒子暂放处 |
| `Link_20` | 机械臂基座 | MoveIt 规划链 base |
| `Link_29` | 机械臂腕部 | MoveIt 规划链 tip;深度相机固连于此 |
| `Link_30` | **深度相机**(eye-in-hand) | §3 手眼标定对象,经 Joint_17 固连 Link_29 |

> 注:吸盘工具目前 URDF **未建模**,计划作圆柱沿 `Link_29` 的 -Z 固连(ready 位即朝下),接触点单独给一个 `suction_tip` link 作抓取 TCP。本人 TODO,**不影响队友输出**。

---

## 0. 代码位置(队友)

队友所有视觉代码写在 **`mm_perception/mm_perception/`** 包内,每个节点在 `mm_perception/setup.py` 的 `entry_points` 里注册 `console_scripts`:

| 工作 | 建议文件 | 输出 |
|---|---|---|
| 盒子识别(§1) | `object_detector.py` | 发布 `/perception/object_pose` |
| ArUco 识别(§2) | `aruco_localizer.py` | 向 `/tf` 广播 `aruco_<id>` |

手眼标定(§3)交付的是 URDF 数值(改 `Joint_17` origin),不在此包,直接给本人回填。

> 本人侧:订阅 `/perception/object_pose` 做抓取转换+执行的代码在 `mm_task/mm_task/grasp_node.py`(独立 action 节点),与队友无耦合。

---

## 1. 盒子位姿(队友 → 任务层/MoveIt 抓取)

抓取模型:**4-DOF top-down**(俗称 3.5D)。接近方向永远竖直向下,只有 `x y z + yaw` 变化。
队友只测**盒子位姿**,不负责末端/吸盘姿态换算。

| 项 | 约定 |
|---|---|
| 话题名 | `/perception/object_pose` |
| 消息类型 | `geometry_msgs/msg/PoseStamped` |
| 发布时机 | **抓取伺服期间连续、低延迟发布(≥10Hz)**,供本人闭环每周期取最新观测(见 §5);非一次性发布 |
| frame_id | **`base_link`**(队友在节点内完成 TF 转换,直接给底盘系;不要给相机系) |
| `position` | 盒子**顶面中心**(吸取接触点;深度相机直接可测,免去盒高) |
| `orientation` | 盒子平放姿态:**roll=pitch=0**(默认置 0,台面不平也忽略),**yaw=盒子绕竖直轴转角** → 四元数 `(x,y,z,w) = (0, 0, sin(yaw/2), cos(yaw/2))` |

### 职责边界(关键)
- **队友**:只输出盒子顶面中心 `xyz` + `yaw`,打包成上面的 `PoseStamped`。**不碰吸盘/末端朝向**。
- **本人(MoveIt 侧)**:把盒子位姿翻成吸盘竖直向下的末端目标、加吸盘长度偏置、对到 `suction_tip`、规划执行。队友无感。

> 几何上:吸盘沿 `Link_29` 的 -Z(ready 位即朝下),"吸盘朝下"等价于法兰 +Z 朝上,所以盒子的 z-up 姿态可近似直接当末端目标姿态,本人侧换算很轻。

说明:
- 选 `PoseStamped` 是因为 MoveIt `MoveGroupInterface.setPoseTarget()` 直接吃这个类型,零转换。
- 多目标场景后续再扩展为自定义数组消息;第一版单目标 `PoseStamped` 足够。

---

## 2. ArUco 定位 TF(队友 → 定位/任务层)

车体二维相机识别 ArUco,输出 TF,用于:**上电初始位姿标定** 和 **到点位置精矫正**。

| 项 | 约定 |
|---|---|
| 输出形式 | 向 `/tf` 广播 `TransformStamped` |
| 父坐标系 | **`Link_13_optical`**(光学系,本人在 URDF 提供;见下方约定) |
| 子坐标系 | `aruco_<id>`(按标记 id 命名) |
| 标记物理尺寸 | 按实际打印尺寸(默认 0.10m),作为节点参数可配 |
| 相机内参来源 | 订阅 `Link_13` 相机的 `camera_info` 话题 |
| 标记 id 分配 | 按现场货架/工位约定,队友在节点参数里登记 |

### 相机光学系约定(重要,不处理会整体差 90°)
- `Link_13` 是相机**机械安装系**(x 朝前);OpenCV/ArUco 解出的位姿在**光学系**(z 朝前、x 朝右、y 朝下,REP-104),两者朝向不同。
- **本人在 URDF 里提供 `Link_13_optical` 光学系 frame**(`Link_13` 下挂一个固定旋转)。
- 队友做法:**直接把 OpenCV 解出的位姿当 `Link_13_optical → aruco_<id>` 广播即可**,不用自己补旋转。
- 若本人的 `Link_13_optical` 还没就绪,临时方案:父系用 `Link_13`,在节点内自行乘上"光学系→`Link_13`"的固定旋转。

下游用法(本人实现,队友无需关心):
- 初始位姿:已知 `map→aruco`(预先标定写死)+ TF 树 `aruco→base_footprint` → 反推 `map→base_footprint` → 发 `/initialpose` 给 AMCL。
- 到点精矫正:用 `aruco` 相对位姿做底盘伺服对位(替代开环平移)。

---

## 3. 手眼标定结果(队友 → URDF)

eye-in-hand:深度相机已建模为 `Link_30`,经 `Joint_17`(fixed)固连机械臂腕部 `Link_29`。CAD 给的是名义安装位姿,手眼标定用于**修正**这个外参。

| 项 | 约定 |
|---|---|
| 标定对象 | `Link_29 → Link_30` 的外参 `x y z roll pitch yaw`(6 个数) |
| 交付内容 | 手眼标定得到的真实外参,用于修正 `mm_robot.urdf` 中 `Joint_17` 的 `origin` |
| 相机 link 名 | `Link_30`(深度相机 `camera_link` 角色) |
| 腕部 link 名 | `Link_29`(MoveIt 规划链 tip) |
| 备注 | 标定阶段可先用 `static_transform_publisher` 临时发 `Link_29→Link_30` 验证,定型后写回 Joint_17 |

---

## 4. 机械臂控制接口(本人,记录备查)

| 项 | 约定 |
|---|---|
| 规划 | MoveIt2 `MoveGroupInterface`,输入 `PoseStamped`(见 §1) |
| 执行 | `FollowJointTrajectory`(ros2_control,仿真与真机一致) |
| 末端执行器 | 气泵吸取,I/O 接口 `待定` |

---

## 5. 抓取执行策略:闭环粗定位 → 闭环精修 → 开环相对直插(本人侧,记录备查)

**动机**:机械臂增量式,上电即当前位置为零,靠手动摆到 URDF 零位,重复性约 1cm(无绝对编码器/自动 homing)。盒子由腕部 eye-in-hand 深度相机 `Link_30` 识别,`base_link←Link_30` 要过机械臂正运动学(FK),零位偏差从这里进入绝对定位。为对该偏差**脱敏**,抓取分三段:

1. **粗定位(闭环,MoveIt 规划)**:取 `/perception/object_pose`,把 TCP 规划到盒子上方预抓取位(约 10–15cm)。
2. **精修(闭环,`moveit_servo` 视觉伺服)**:持续读新鲜 `object_pose`,用 Cartesian jog 把 TCP 对准盒子顶面中心 `xy + yaw`,逼近到约 5–8cm,直至观测对准误差 < 阈值。
3. **末段(开环,相对当前姿态短距离直插)**:从精修收敛姿态,沿吸盘接近轴(`suction_tip`,`Link_29` -Z)**相对**下插固定行程 + 气泵吸取。

**纪律(写死)**:末段必须是"相对当前实测姿态的短距离运动";**严禁在末段用 FK 重新计算盒子在 `base_link` 的绝对坐标再 `setPoseTarget`**——否则零位偏差原样加回末端。前两段靠新鲜相机反馈把绝对定位误差磨到物理正确位置,末段只承受短行程差分残余(亚毫米~mm 级)。

**残余兜底**:姿态微偏 × 行程会产生横向漂移,故 (a) 精修尽量逼近再插以缩短行程,(b) 靠吸盘/夹爪机械容差(倒角/柔性)吃 mm 级残差。

**对队友无影响**:三段全在 `grasp_node`/MoveIt 本人侧;对队友唯一新增 = §1 的连续发布要求。`moveit_servo` 集成为本人 TODO,细节待验证。

---

## 待定项清单(URDF 定型后,大部分已回填)
- [x] 车体二维相机 link 名(§2)→ `Link_13`
- [x] 深度相机 camera_link 名(§3)→ `Link_30`(固连 `Link_29`)
- [x] 抓取模型 & 位姿语义(§1)→ 4-DOF top-down,队友发盒子顶面中心 `xyz+yaw`,末端换算归本人
- [x] 话题名(§1)→ `/perception/object_pose`(原 `grasp_pose` 改名,语义=盒子位姿非末端姿态)
- [ ] 吸盘工具 link / `suction_tip` TCP(§1)— **本人 TODO**,URDF 补吸盘(沿 Link_29 -Z)后确定
- [ ] `Link_13_optical` 光学系 frame(§2)— **本人 TODO**,URDF 在 `Link_13` 下挂固定旋转
- [ ] ArUco 各标记 id 分配(§2)— 队友按现场登记到节点参数
- [ ] 气泵 I/O 接口(§4)— 实机阶段定
- [x] 抓取执行策略(§5)→ 闭环粗定位 + 闭环精修 + 开环相对直插,对零位偏差脱敏
- [ ] `moveit_servo` 视觉伺服集成(§5)— **本人 TODO**,阈值/行程/伺服参数待验证
