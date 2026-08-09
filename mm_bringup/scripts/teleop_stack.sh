#!/bin/bash
# 一键遥控全栈的起/停/核实。跑在 Nano 上, 也可从本机 ssh 调:
#   ssh dong@<nano> 'bash ~/Desktop/moveit/install/mm_bringup/share/mm_bringup/scripts/teleop_stack.sh restart'
#
#   start    起栈 (nohup 到 logs_d29/, 立即返回)
#   stop     清栈 (含孤儿), 打印残留
#   check    核实单实例 + 关键参数 + CAN + 臂反馈频率
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
PATS="grasp_node joy_arm_teleop move_group ros2_control_node robot_state_publisher \
micro_ros_agent servo_node yolo_box_detector realsense2_camera joy_node teleop_node \
teleop_twist_joy can_bridge republish ros2launch usb_cam_node_exe image_rotator"

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

do_stop() {
  # 连 tmux 会话一起收掉, 否则会话空转着, 下次 tmux 子命令会报"已存在"而拒起。
  tmux kill-session -t "$SESSION" 2>/dev/null
  pkill -f "ros2 launch" 2>/dev/null
  sleep 2
  for p in $PATS; do pkill -f "$p" 2>/dev/null; done
  sleep 3
  for p in $PATS; do pkill -9 -f "$p" 2>/dev/null; done
  sleep 2
  echo "=== 残留 (应为空) ==="
  ps -eo pid,ppid,etimes,cmd | grep -Ei 'grasp_node|joy_arm|move_group|ros2_control|state_publisher|micro_ros|realsense|yolo_box|can_bridge|servo_node|teleop|usb_cam|image_rotator' \
    | grep -v grep | grep -v teleop_stack.sh | cut -c1-95
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
  check)   do_check ;;
  restart) do_stop; do_start; sleep 50; do_check ;;
  retmux)  do_stop; do_tmux; sleep 50; do_check ;;
  *) echo "用法: $0 {start|tmux|stop|check|restart|retmux}"; exit 1 ;;
esac
