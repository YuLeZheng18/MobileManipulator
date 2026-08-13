#!/bin/bash
# 一键遥控全栈的起/停/核实。跑在 Nano 上, 也可从本机 ssh 调:
#   ssh dong@<nano> 'bash ~/Desktop/moveit/install/mm_bringup/share/mm_bringup/scripts/teleop_stack.sh restart'
#
#   start    起栈 (nohup 到 logs_d29/, 立即返回)
#   stop     清整机全栈(臂+导航, 含孤儿), 打印残留, 残留为 0 时顺手删 /dev/shm 孤儿段
#   shm      单独跑一次 /dev/shm 孤儿段清理 (手动杀过进程之后用)
#   check    核实单实例 + 关键参数 + CAN + 臂反馈频率 + /dev/shm 段数
#   restart  stop -> start -> 等 50s -> check
#
# ⚠️ 为什么这些动作必须写成脚本文件, 不能内联进 ssh 命令:
#   pkill -f / pgrep -f / ps|grep 都匹配**整条命令行**, 而内联时自己那条 `bash -c ...`
#   里就含着这些进程名 ——
#     - pkill -f 会把自己连 ssh 会话一起杀掉 (2026-08-03 踩, 清栈清到一半断连, 栈成半死态)
#     - pgrep -fc 每项虚高 1, "单实例核实"全报 2 (同日踩, 差点当成重复启动去查)
#   写进文件后, 命令行只有脚本名, 不含进程名, 两个坑同时消失。
#
# 刻意**不加 set -u**: /opt/ros/humble/setup.bash 里引用了未定义的
# AMENT_TRACE_SETUP_FILES, 开了 -u 会在 source 那一行直接退出。

WS="${MM_WS:-$HOME/Desktop/moveit}"
SESSION=mmstack

# 清栈要覆盖的进程名。⚠️ can_bridge 必须在列: 2026-08-03 漏掉它, 旧栈那个变成 init 孤儿
# 继续占着 /dev/pcanusb32, 新栈的 can_bridge 起来直接报
#   CAN 初始化失败: 0x8000000 ... An operation is not allowed due to the current configuration
# 而臂**照样能动**(旧进程还在收命令), 于是新旧混跑却看不出来。
# 同理杀 `ros2 launch` 壳子不会带走 robot_state_publisher / ros2_control_node, 必须逐个点名。
#
# ⚠️ usb_cam_node_exe / image_rotator 同日补入, 与 can_bridge 完全同一个形状:
# 漏掉它们时旧栈的 usb_cam 变孤儿继续占着 /dev/video8, 新栈的起来打不开设备就
#   terminate called after throwing an instance of 'char*'  -> exit code -6
# 而 cam_b **照样有画面**(旧进程还在发), cam_a 没有旧进程就彻底黑掉 ——
# 于是"两路相机一路好一路坏"看着像设备问题, 其实是清栈没清净。
#
# ⚠️ 2026-08-13 补入**导航那半边**。此前 PATS 只有机械臂一半, 于是 `stop` 完 nav2 / amcl /
# ekf_node / lane_navigator / rplidar 全都还活着变成孤儿。后果不只是"新旧混跑":
# 它们每个都是一个 DDS participant, 各自占着 /dev/shm/fastrtps_* 段, 被下一轮 pkill -9
# 收掉时**跳过析构**, 段就永久留在那 —— 这就是"总是泄漏、来回折腾"的直接来源
# (段涨到几十个后服务 response 投不出去, 表现成假死锁, 见记忆 dev_shm_fastrtps_leak)。
# 所以本脚本的 stop 是**停整机全栈**(臂 + 导航), 不是只停 teleop 那几个。
PATS="grasp_node joy_arm_teleop move_group ros2_control_node robot_state_publisher \
micro_ros_agent servo_node yolo_box_detector realsense2_camera joy_node teleop_node \
teleop_twist_joy can_bridge republish ros2launch usb_cam_node_exe image_rotator \
lane_navigator mission_manager aruco_localizer chassis_diag_logger \
controller_server planner_server bt_navigator behavior_server smoother_server \
waypoint_follower velocity_smoother lifecycle_manager map_server amcl \
scan_to_scan_filter_chain twist_mux ekf_node rplidar_composition"

# 残留核实用的匹配式。除了点名的进程, 还兜一层"凡是从 ros 安装空间起来的东西",
# 这样连没点名的节点(临时手起的、rviz2)也会被算进残留 —— 删 /dev/shm 段前必须确认
# 一个 participant 都不剩, 漏算一个就会把活着的进程打断。
RESIDUAL_RE='grasp_node|joy_arm|move_group|ros2_control|state_publisher|micro_ros|realsense|yolo_box|can_bridge|servo_node|teleop|usb_cam|image_rotator|lane_navigator|mission_manager|aruco|nav2_|amcl|ekf_node|twist_mux|rplidar|scan_to_scan|/opt/ros/humble/lib/'

# check 用的进程清单 (期望各 1 个)
SINGLETONS="can_bridge grasp_node joy_arm_teleop move_group ros2_control_node \
servo_node yolo_box_detector micro_ros_agent robot_state_publisher"

_src() {
  source /opt/ros/humble/setup.bash
  # micro_ros_agent 在独立 ws, 不 source 则整个 launch 起不来(找不到可执行) —— 最容易漏的一条
  source "$HOME/microros_ws/install/setup.bash"
  source "$WS/install/setup.bash"
  export ROS_DOMAIN_ID=42
}

_residual() {
  ps -eo pid,ppid,etimes,cmd --no-headers 2>/dev/null \
    | grep -Ei "$RESIDUAL_RE" | grep -v grep | grep -v teleop_stack.sh
}

# 删 Fast-DDS 共享内存段。**只在残留为 0 时执行**。
# 这些段由每个 participant 在构造时创建、在**析构**时删除; kill -9 与节点崩溃都跳过析构,
# 段就成了孤儿留在 /dev/shm。反复起停栈 = 每次留一批, 攒到几十个之后服务的 response
# 投不出去(服务端回调明明跑完了, 客户端 future 永不完成, 干等到超时), 看着像死锁。
# 处置就是删段, **不用重启整机**。
# ⚠️ 必须先确认没有活着的 participant: 删掉活进程正在用的段, 会当场打断它。
do_shmclean() {
  local before after left
  before=$(ls /dev/shm 2>/dev/null | grep -c fastrtps)
  left=$(_residual | wc -l)
  if [ "$left" -ne 0 ]; then
    echo "!!! 仍有 $left 个 ROS 进程活着, 跳过 /dev/shm 清理 (当前 fastrtps 段 $before 个)"
    echo "    删掉活进程正在用的段会当场打断它。先关掉上面列出的进程(rviz2 也算), 再跑: $0 shm"
    return 1
  fi
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
  after=$(ls /dev/shm 2>/dev/null | grep -c fastrtps)
  echo "=== /dev/shm fastrtps 段: $before -> $after (健康是 0; 起栈后个位数正常) ==="
}

do_stop() {
  # tmux 会话: 先往里送 Ctrl-C 走正常 shutdown, 等它自己收完再 kill-session。
  # 直接 kill-session 等于抽掉进程组, launch 里的节点全部跳过析构 —— 段就是这么漏的。
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux send-keys -t "$SESSION" C-c 2>/dev/null
    sleep 5
    tmux kill-session -t "$SESSION" 2>/dev/null
  fi
  # SIGINT 而非默认 SIGTERM: rclcpp/rclpy 把 SIGINT 接成"优雅退出", 会跑完析构
  # (DDS participant 析构才会删掉自己的 /dev/shm 段)。SIGTERM 走的是不保证的路径。
  pkill -INT -f "ros2 launch" 2>/dev/null
  sleep 4                                   # 给 launch 时间把 SIGINT 传下去并等子进程收尾
  for p in $PATS; do pkill -INT -f "$p" 2>/dev/null; done
  sleep 4
  # 到这一步还赖着不走的才动 -9。顺序不能反: 一上来 pkill -9 就是漏段的元凶。
  for p in $PATS; do pkill -9 -f "$p" 2>/dev/null; done
  sleep 2
  echo "=== 残留 (应为空) ==="
  _residual | cut -c1-95
  do_shmclean
  echo "=== STOPPED ==="
}

do_start() {
  _src
  mkdir -p "$WS/logs_d29"
  local log="$WS/logs_d29/teleop_stack_$(date +%m%d_%H%M).log"
  cd "$WS" || exit 1
  nohup ros2 launch mm_bringup teleop_stack.launch.py > "$log" 2>&1 &
  echo "PID=$! LOG=$log"
  echo "(分阶段起到 t=22s, 约 50s 后再 check)"
}

# 起在 tmux 会话里: 断开 SSH 栈照跑, 重连后能看到实时滚动日志、能直接 Ctrl-C 停栈。
# 与 do_start 的区别只是"跑在哪": nohup 那种看不到实时输出, 只能事后 tail 日志文件。
do_tmux() {
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "会话 '$SESSION' 已存在。接入: tmux attach -t $SESSION   (先停: $0 stop)"
    return 1
  fi
  mkdir -p "$WS/logs_d29"
  local log="$WS/logs_d29/teleop_stack_$(date +%m%d_%H%M).log"
  # 在会话内重新 source: tmux server 继承的是**第一次**建会话时的环境, 不能指望它对.
  # 2>&1 | tee 让日志既滚在屏幕上又落盘 (事后排查要文件, 现场看要实时).
  tmux new-session -d -s "$SESSION" -c "$WS" \
    "source /opt/ros/humble/setup.bash; source \$HOME/microros_ws/install/setup.bash; \
     source $WS/install/setup.bash; export ROS_DOMAIN_ID=42; \
     ros2 launch mm_bringup teleop_stack.launch.py 2>&1 | tee '$log'; \
     echo; echo '--- 栈已退出, 回车关闭会话 ---'; read"
  echo "会话 '$SESSION' 已起。LOG=$log"
  echo "接入: ssh -t dong@$(hostname -I | awk '{print $1}') 'tmux attach -t $SESSION'"
  echo "(会话内 Ctrl-B 再按 D 脱离, 栈继续跑; Ctrl-C 才是停栈)"
}

do_check() {
  _src
  # 段数放最前面: 服务超时/假死锁时这是第一个该看的数, 不是节点代码。
  # 起栈后个位数正常; 几十个 = 前几轮没杀干净, 服务 response 随时会投不出去。
  echo "=== /dev/shm fastrtps 段数 (个位数正常, 几十个=该 stop 一次) ==="
  ls /dev/shm 2>/dev/null | grep -c fastrtps
  echo "=== 单实例 (应全为 1) ==="
  for p in $SINGLETONS; do
    n=$(ps -eo cmd | grep -F "$p" | grep -v grep | grep -v teleop_stack.sh | wc -l)
    printf "%-22s %s\n" "$p" "$n"
  done
  echo "=== grasp_node 关键参数 ==="
  for k in suck_duration release_duration cam_target_y cam_target_z insert_shortfall; do
    printf "%-20s %s\n" "$k" "$(ros2 param get /grasp_node "$k" 2>&1 | tail -1)"
  done
  echo "=== CAN ==="
  grep -hiE "CAN 控制器状态|CAN 初始化失败|CAN接口初始化失败" \
    "$(ls -t "$WS"/logs_d29/teleop_stack_*.log 2>/dev/null | head -1)" 2>/dev/null | tail -3
  # /arm_joint_states 是 can_bridge 发的真实反馈; /joint_states 有可能是命令回显, 别拿它当证据
  echo "=== /arm_joint_states (真实 CAN 反馈, 期望 ~50-120Hz) ==="
  timeout 8 ros2 topic hz /arm_joint_states 2>&1 | grep -E "average|WARNING" | head -2
}

case "${1:-check}" in
  start)   do_start ;;
  tmux)    do_tmux ;;
  stop)    do_stop ;;
  shm)     do_shmclean ;;
  check)   do_check ;;
  restart) do_stop; do_start; sleep 50; do_check ;;
  retmux)  do_stop; do_tmux; sleep 50; do_check ;;
  *) echo "用法: $0 {start|tmux|stop|shm|check|restart|retmux}"; exit 1 ;;
esac
