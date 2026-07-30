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

> 注:吸盘工具**已建模**——圆柱沿 `Link_29` 的 -Z 固连(`suction_link`,ready 位即朝下),接触点 TCP 为 `suction_tip`(`Link_29` -Z 方向 9.5cm)。**不影响队友输出**。

---

## 0. 代码位置(队友)

队友所有视觉代码写在 **`mm_perception/mm_perception/`** 包内,每个节点在 `mm_perception/setup.py` 的 `entry_points` 里注册 `console_scripts`:

| 工作 | 实际文件 | 输出 |
|---|---|---|
| 盒子识别(§1) | `yolo_box_detector.py` | 发布 `/perception/object_pose` + §1.1 四路 |
| ArUco 识别(§2) | `aruco_localizer.py` | 向 `/tf` 广播 `aruco_<id>` |

> 盒子识别最终落在 `yolo_box_detector.py`(自训练 YOLO + NCNN 后端,ARM CPU 上约 4.8x 加速),
> 不是原计划的 `object_detector.py`。参数在 `mm_perception/config/yolo_box_detector.yaml`,
> 起法 `ros2 launch mm_perception yolo_box_detector.launch.py`(该 launch 负责把 yaml 里的
> 相对模型名拼成 share 下绝对路径,别裸 `ros2 run`)。

手眼标定(§3)交付的是 URDF 数值(改 `Joint_17` origin),不在此包,直接给本人回填。

> 本人侧:订阅 `/perception/object_pose` 做抓取转换+执行的代码在 `mm_grasp` 包(C++ `grasp_node`,`std_srvs/Trigger` 服务 `/grasp/execute` 触发三段抓取,被 `mm_task` 状态机调用),与队友无耦合。

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

### 1.1 实机新增话题(2026-07 抓取调试期陆续加,均由 `yolo_box_detector` 发)

单目标 `/perception/object_pose` 不够用,实机上又长出四路。**语义口径统一**:凡是"可抓盒",
都指过完 `pick_z_max` 高度滤除之后的候选(托盘上已放的盒比地面盒离相机更近、成像更大,
不滤会被当成抓取目标)。

| 话题 | 类型 | 用途 |
|---|---|---|
| `/perception/object_poses` | `PoseArray` | **所有可抓盒**的位姿数组。**数组长度 = 还剩几个盒可抓** —— 状态机据此决定同一货架连抓几次(见 §7.2 S4),空数组也照发(否则计数永远归不了零,循环停不下来) |
| `/perception/object_point_cam` | `PointStamped` | 目标盒心在**相机机械系 `Link_30`** 的坐标。②精修闭环追的是这个,不是 `base_link` 坐标 —— 后者要过 FK+手眼外参,零位偏差让它偏 2~3cm 且随构型变,闭环追它会变成"臂动→偏差变→目标跑"的追逐 |
| `/perception/object_axis_angle` | `Float32` | 目标盒长轴的**图像角** θ_img(度,折到 (-90,90])。吸盘朝向对齐闭环的判据 —— 相机固连腕部,故"吸盘长轴在图像里占多少度"是与构型无关的常数,对齐 ⇔ θ_img 落进容差 |
| `/perception/object_class` | `Int32` | 目标盒类别(1~4)。决定放哪个托盘 + 用哪个标定厚度 |
| `/perception/object_thickness` | `Float32` | 视觉估的盒厚(米)。**当前仅打印对照不采信**,放置高度用 `place.yaml` 的 `fallback_thickness` 实测标定值 |

> 目标选择:单目标那几路(`object_pose`/`point_cam`/`axis_angle`/`class`)靠 p_cam 最近邻锚定跟踪同一个盒,
> 避免抓取过程中吸盘遮挡导致"最大框"静默换目标(2026-07-30 实证过这个坑)。

---

## 2. ArUco 定位 TF(队友 → 定位/任务层)

车体二维相机识别 ArUco,输出 TF,用于:**上电初始位姿标定** 和 **到点位置精矫正**。

| 项 | 约定 |
|---|---|
| 输出形式 | 向 `/tf` 广播 `TransformStamped` |
| 父坐标系 | **`base_link`**(队友在节点内完成 TF 转换,直接给底盘系;与 §1 一致) |
| 子坐标系 | `aruco_<id>`(按标记 id 命名) |
| 标记物理尺寸 | 按实际打印尺寸(默认 0.10m),作为节点参数可配 |
| 相机内参来源 | 订阅 `Link_13` 相机的 `camera_info` 话题 |
| 标记 id 分配 | 按现场货架/工位约定,队友在节点参数里登记 |

### 队友节点内 TF 换算(重要,不处理会整体差 90°)
- OpenCV/ArUco 解出的位姿在相机**光学系**(z 朝前、x 朝右、y 朝下,REP-104);`Link_13` 是相机**机械安装系**(x 朝前),两者差一个固定旋转。
- 队友在节点内把观测链到 `base_link` 再广播(与 §1 输出 base_link 一致):
  `base_link → aruco_<id>` = `base_link → Link_13`(查 TF 树,`robot_state_publisher` 已发) ∘ `Link_13 → 光学系`(固定旋转,REP-104) ∘ `光学系 → aruco_<id>`(OpenCV 解算)。
- 即:先补"光学系→`Link_13`"固定旋转,再用 TF 查询把位姿转到 `base_link`,最后广播 `base_link → aruco_<id>`。

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
| 执行 | `FollowJointTrajectory`(ros2_control → `arm_control/can_bridge` → 0xFD CAN) |
| 末端执行器 | 气泵吸取,`/pump_cmd`(`std_msgs/Int8`):**0=STOP(关泵关阀) 1=SUCK 2=RELEASE(开阀)** |

**气泵纪律**(实机踩出来的):
- `RELEASE` 是**持续通电开阀**,发完必须补 `STOP`。实测放置后阀一直开着十几分钟烫手,
  故释放实现为脉冲:开阀 `release_duration`(1.0s,破负压几百 ms 够)→ 立刻 `STOP`。
- 吸住后也发 `STOP` 而非一直 `SUCK`:气路封住靠密封维持负压,盒子照样吸着;持续 `SUCK` 泵空转发热且无收益。

---

## 5. 抓取执行策略:闭环粗定位 → 闭环精修 → 开环相对直插(本人侧,记录备查)

**动机**:机械臂增量式,上电即当前位置为零,靠手动摆到 URDF 零位,重复性约 1cm(无绝对编码器/自动 homing)。盒子由腕部 eye-in-hand 深度相机 `Link_30` 识别,`base_link←Link_30` 要过机械臂正运动学(FK),零位偏差从这里进入绝对定位。为对该偏差**脱敏**,抓取分三段:

1. **粗定位(闭环,MoveIt 规划)**:取 `/perception/object_pose`,把 TCP 规划到盒子上方预抓取位(12cm)。
2. **精修(闭环,相机系笛卡尔步进)**:持续读新鲜 `/perception/object_point_cam`,在**相机系**里
   把横向误差 (y,z) 追到标定目标值 `cam_target_y/z`,每步用 `computeCartesianPath` 走一小段,
   收敛容差 3mm。**不用 `moveit_servo`**(见下方"实机修正")。
2b. **吸盘朝向对齐(闭环,转腕)**:追 `/perception/object_axis_angle` 的图像角 θ_img 到标定常数
   `yaw_target_theta_img`,容差 1°。要的是"吸盘长轴与盒长轴共线",否则盒斜贴吸盘、放进带围栏的托盘会蹭卡。
3. **末段(开环,相对当前姿态短距离直插)**:从精修收敛姿态,沿吸盘接近轴(`suction_tip`,`Link_29` -Z)**相对**下插固定行程 + 气泵吸取。

**纪律(写死)**:末段必须是"相对当前实测姿态的短距离运动";**严禁在末段用 FK 重新计算盒子在 `base_link` 的绝对坐标再 `setPoseTarget`**——否则零位偏差原样加回末端。前两段靠新鲜相机反馈把绝对定位误差磨到物理正确位置,末段只承受短行程差分残余(亚毫米~mm 级)。

**残余兜底**:姿态微偏 × 行程会产生横向漂移,故 (a) 精修尽量逼近再插以缩短行程,(b) 靠吸盘/夹爪机械容差(倒角/柔性)吃 mm 级残差。

**对队友无影响**:各段全在 `mm_grasp`/MoveIt 本人侧;对队友唯一新增 = §1 的连续发布要求。

### 实机修正(2026-07 调试后,推翻上面两处原设计)

**① 精修不用 `moveit_servo`,改相机系笛卡尔步进。** 原设计是"读 `object_pose`(base_link 系)
→ servo jog 对准"。实机上这条路走不通:`base_link` 坐标 = FK(6关节) ∘ 手眼外参,而机械臂是
增量式编码器、上电即零位、零位靠手摆(重复性约 1cm),旋转误差经 FK 进来且**随臂构型变**——
闭环追它就成了"臂动→偏差变→目标跑"的追逐(实测 20s 不收敛)。改追相机系的 `p_cam`:它只过
深度与相机内参,臂动它不跳。目标值 `cam_target_y/z` 由人工对准后读回标定,于是手眼偏差**压根不进闭环**。
`servo_node` 仍在 launch 里起着(`mm_grasp/config/servo.yaml`),但抓取主链路没用它。

**② 朝向对齐不用 `object_pose` 里的 base_link 系盒 yaw,改追图像角。** 同一个根因:那个 yaw
也要过 FK+外参。2026-07-27 实测检测报盒 +36.2°,按它转腕后人工还得再补 6° 才与盒边贴合。
现改为闭环追图像角 θ_img —— 相机固连腕部,故"吸盘长轴在图像里占多少度"与构型无关,
是个能在机上直接看到的**终点判据**,腕该转多少不需要事先知道。

**③ 关键标定值全部入库**,在 `mm_grasp/config/place.yaml`(每个值都带实测由来与调整历史):
双托盘位姿/容量、类别→托盘映射、各类别标定厚度、`cam_target_y/z`、`yaw_target_theta_img`、
横移高度候选、卸货点几何。改这些多数要**重启 `grasp_node`**(启动时读进 C++ 成员,`ros2 param set` 不热更)。

---

## 待定项清单(URDF 定型后,大部分已回填)
- [x] 车体二维相机 link 名(§2)→ `Link_13`
- [x] 深度相机 camera_link 名(§3)→ `Link_30`(固连 `Link_29`)
- [x] 抓取模型 & 位姿语义(§1)→ 4-DOF top-down,队友发盒子顶面中心 `xyz+yaw`,末端换算归本人
- [x] 话题名(§1)→ `/perception/object_pose`(原 `grasp_pose` 改名,语义=盒子位姿非末端姿态)
- [x] 吸盘工具 link / `suction_tip` TCP(§1)→ URDF 已建 `suction_link`(Link_29 -Z 圆柱)+ `suction_tip` TCP(Link_29 -Z 9.5cm)
- [x] ArUco 输出坐标系(§2)→ 统一 `base_link`,队友节点内做 TF 换算(与 §1 一致),无需 URDF 建 `Link_13_optical`
- [ ] ArUco 各标记 id 分配(§2)— 队友按现场登记到节点参数
- [x] 气泵 I/O 接口(§4)→ `/pump_cmd` `std_msgs/Int8`,0=STOP 1=SUCK 2=RELEASE(释放须脉冲式,见 §4)
- [x] 抓取执行策略(§5)→ 闭环粗定位 + 相机系精修 + 图像角朝向对齐 + 开环相对直插,已真机跑通
- [x] ~~`moveit_servo` 视觉伺服集成(§5)~~ → **已放弃**,改相机系笛卡尔步进(理由见 §5 实机修正①)
- [x] 多目标输出(§1.1)→ `/perception/object_poses`(PoseArray),长度=剩余可抓盒数
- [ ] S2 到点 ArUco 精对位(架构 §7.2)— 状态机里目前是 no-op 直接放行,**本人 TODO**
