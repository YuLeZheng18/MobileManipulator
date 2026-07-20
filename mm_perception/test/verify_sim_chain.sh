#!/usr/bin/env bash
# 无显示验证合成相机 -> aruco_localizer 整条链路.
# 拉起两个节点(输出各自重定向到日志), 采集话题/帧率/两条TF, 再回收进程.
# 注: 不用 set -u —— ROS 的 setup.bash 会引用未定义的 AMENT_TRACE_SETUP_FILES.
set -e
WS=/home/dong/Desktop/moveit
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

LOG=/tmp/aruco_sim
mkdir -p "$LOG"
: > "$LOG/cam.log"; : > "$LOG/loc.log"

# 标记正对静止, 便于比对稳态误差
ros2 run mm_perception sim_aruco_camera --ros-args \
  -p static_pose:=true > "$LOG/cam.log" 2>&1 &
CAM=$!
ros2 run mm_perception aruco_localizer --ros-args \
  --params-file "$WS/mm_perception/config/aruco_localizer.yaml" > "$LOG/loc.log" 2>&1 &
LOC=$!

cleanup() { kill "$CAM" "$LOC" 2>/dev/null; wait 2>/dev/null; }
trap cleanup EXIT

sleep 4
echo "===== 话题 ====="
ros2 topic list | grep -Ei 'camera|image' || echo "(无相机话题)"

echo "===== 图像帧率 (3s) ====="
timeout 4 ros2 topic hz /camera/image_raw 2>&1 | grep -m1 average || echo "(无图像)"

echo "===== 真值 TF  Link_13 -> gt_aruco_7 ====="
timeout 5 ros2 run tf2_ros tf2_echo Link_13 gt_aruco_7 2>&1 | grep -A8 -m1 Translation || echo "(无真值TF)"

echo "===== 解算 TF  Link_13 -> aruco_7 ====="
timeout 5 ros2 run tf2_ros tf2_echo Link_13 aruco_7 2>&1 | grep -A8 -m1 Translation || echo "(无解算TF)"

echo "===== 相机节点日志尾 ====="
tail -n 6 "$LOG/cam.log"
echo "===== 定位节点日志尾 ====="
tail -n 6 "$LOG/loc.log"
