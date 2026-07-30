# 真机测试记录

> 跨会话真机调试记录,倒序(最新在上)。每条:`## YYYY-MM-DD 标题` + 验了什么 / 结果 / 遗留。
> 仿真已闭环,这里只记真机。

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
