# 真机测试记录

> 跨会话真机调试记录,倒序(最新在上)。每条:`## YYYY-MM-DD 标题` + 验了什么 / 结果 / 遗留。
> 仿真已闭环,这里只记真机。

## 2026-07-31 机械臂工作段收尾:同货架连抓 + 整盘卸货接进状态机

**本轮只改代码不上真机**(PC 侧全部 colcon build 通过 + launch 解析通过,真机行为待验)。
机械臂单点动作在 07-26~07-30 已逐个跑通,本轮把它们接成状态机能驱动的完整流程。

**改了什么:**

1. **同货架连抓(核心)。** 一个货架有 1~4 个盒、几个只有识别了才知道,故不能在 mission.yaml
   里按盒数写死多条 grasp 任务。做法:
   - 感知侧 `yolo_box_detector` 新发 `/perception/object_poses`(PoseArray),内容是**过完
     `pick_z_max` 之后的可抓盒**,长度即"还剩几个"。**空数组也必须发** —— 踩过:发布语句原本
     在 `if not pickable: return` 之后,于是计数永远归不了零,循环停不下来。
   - `grasp_node` 的 `/grasp/look` 与 `/grasp/execute` 在 message 里回报 `pickable=N, tray_free=M`;
     满盘时 `resolvePlaceTarget` 返回 `TRAY_FULL:` 前缀(严重级降为 WARN)。
   - 状态机 `stage_grasp_shelf()` 循环 detect+execute,三个出口: pickable 归 0 / TRAY_FULL 或
     tray_free 归 0 / `max_picks_per_shelf` 兜底。**前两个算成功**,后续卸货任务照常跑。
   - 满盘判据是**本类别映射到的那个盘**满没满, 不是总余量: 4 个同类盒只进一个盘(容量 2)。
   - `/grasp/execute` 收尾改为停在 **look 位**(原来回 ready): 连抓时下一轮直接就能识别,
     省一趟空行程。收身改由状态机在**真要走**时调 `/grasp/ready`。

2. **卸货换 `/grasp/unload_tray`。** 状态机原先调的 `/grasp/unload` 靠视觉从托盘上重新识别,
   而托盘上的盒高于 `pick_z_max` 会被整批滤掉 —— 这条路真机根本拿不到目标。`unload_tray`
   不用视觉,取盒目标就是当初放它时的释放位姿。卸哪个盘走 `SetParameters`(Trigger 带不了数值)。
   空盘返回"空的"按跳过处理不算失败(这一轮装了几个盘取决于货架上有几个盒)。

3. **S0 加 `/grasp/reset_stack`**(失败只 warn): 不清则上一轮记的"盒还在托盘上"会让本轮
   一开抓就 TRAY_FULL,残留 `placed_*` 碰撞体还会挡住放置直下段。原先靠人记着手动调。

**顺带修的 launch/依赖问题(都是"从来没在本机编过/起过"才没暴露):**
- `real_bringup` 感知段起的 `mm_perception/object_detector` **是个不存在的可执行**
  (实际叫 `yolo_box_detector`),即 `use_perception:=true` 必然启动失败。改为 include 它自己的
  launch(要把 yaml 里的相对模型名拼成 share 绝对路径,不能裸 Node 起),传 `with_rsp:=false`。
- **RealSense 从来没有任何 launch 起过**,一直手动补。并进 `cameras.launch.py`,
  `align_depth.enable:=true` 硬编(事后 param set 改不了)、`pointcloud.enable:=false`。
- `mm_task/package.xml` 依赖 `pymoveit2`(已删的 Python grasp_node 遗留)→ 本机编不过。
- `mm_bringup/package.xml` 依赖 `rplidar_ros`(Nano apt 装的)→ 整包在本机编不过。
  `install/mm_bringup` 时间戳比该提交还早,证明本机从未编过它。已移出 exec_depend 并注明。
- 删掉 `mm_task/mm_task/grasp_node.py`(288 行 MVP 版 Python 抓取节点)及其 launch/config/entry_point,
  功能早已被 C++ `mm_grasp/grasp_node` 完全取代。

**新增** `mm_task/config/mission_real.yaml`(真机任务表,超时放宽: grasp 240s 因为一条 grasp
任务内部要连抓多盒)。`initial_pose` 留 TODO 待现场标。

**下次上真机要验的:**
1. `/grasp/look` 的 `pickable=N` 是否与货架上实际盒数一致(改了感知侧发布位置)
2. 连抓循环:摆 3~4 个盒,看是否抓到托盘满自动停、且不误判为故障中止
3. `unload_tray` 经状态机 SetParameters 驱动是否正常(此前只手动 param set 验过)
4. 一条命令整机自主: `use_cameras:=true use_perception:=true run_mission:=true`
5. **Nano 需重编 `mm_grasp`**(grasp_node.cpp 改了),其余改动都在 PC 侧

## 2026-07-29 验碰撞体修复(2026-07-28 蹭飞事故 fix)

**背景:** 2026-07-28 放右托盘时机械臂规划路径横扫蹭飞左托盘已放盒。当日在 grasp_node.cpp 加"已放盒持久碰撞体":`placed_ids_`/`placed_poses_` 按托盘分组,`pushLayer` 落盒时登记 BOX 碰撞体,`placeAtPose` 下直段前 `hideTrayBoxes(本盘)`、出口 `showTrayBoxes(本盘)`,**别盘的盒全程留场景当障碍**。`/grasp/reset_stack` 一并清,`/grasp/seed_placed` 重启后补登。本机已 colcon build,代码 21:08 二进制已编。

**部署状态(本次会话核查):**
- `grasp_node.cpp` / `yolo_box_detector.py`:本地 vs Nano md5 一致 ✅(上次会话已 scp)
- `place.yaml`:本地多一条 `release_duration: 1.0`,其他标定值全一致 → 本次补同步这一行
- `yolo_box_detector.yaml`:Nano `show_window: true` vs 本地 `false`(headless),保留 Nano 现状不动

**验证步骤(真盒,DOMAIN_ID=42,全栈在 Nano):**
1. 起全栈:arm real_bringup → move_group → RealSense + yolo → `ros2 launch mm_grasp grasp.launch.py use_sim_time:=false` → micro_ros_agent
2. `/grasp/reset_stack` 清零(上轮空走留了托盘1第1层假堆叠)
3. 真盒放左托盘(类别2或3,对应 tray=1)→ 验 `pushLayer` 登记碰撞体
4. 再放右托盘(类别3或4,对应 tray=0)→ **关键:规划路径应避开左盘已放盒**,不再蹭飞
5. 验 yaw 闭环重做(追 θ_img*):`stageAlignYaw` 迭代收敛,θ_img 进容差
6. 验 release_duration 1.0:开阀破真空 ~1s 后关阀,节拍比 3.0 快

**遗留:** 验过 → 提交 4 文件(不加 co-author) + 补本地进度日志。

**⚠️ 起栈前关键纪律(2026-07-29 用户现场交代):**
- 机械臂当前在**零位**(上电即零位,增量编码器无 homing),不是 ready 位。
- 抓取流程必须严格 **ready → look → execute** 三步走:
  1. 先 `/grasp/ready` 把臂从零位开到 ready 姿态(工作预备位)
  2. 再 `/grasp/look` 开到 look 姿态(相机俯视盒子上方,锁定 θ_img)
  3. 最后 `/grasp/execute` 跑三段抓取(粗定位→精修伺服→直插吸取)
- **跳过 ready 直接 look/execute 会从零位大角度规划,有撞限位/机构风险**。

**本次操作模式:** Claude 经 SSH 全程跑,用户在现场盯安全(e-stop 待命)。
