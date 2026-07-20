#!/usr/bin/env bash
# 自包含验证 dolly(前后往返) 模式: 起节点->探测距离曲线->回收.
WS=/home/dong/Desktop/moveit
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
LOG=/tmp/dolly; mkdir -p "$LOG"

pkill -f sim_aruco_camera 2>/dev/null; pkill -f aruco_localizer 2>/dev/null
sleep 1

ros2 run mm_perception sim_aruco_camera --ros-args \
  -p motion_mode:=dolly -p dolly_near:=0.3 -p dolly_far:=1.0 -p dolly_period:=8.0 \
  > "$LOG/cam.log" 2>&1 &
CAM=$!
ros2 run mm_perception aruco_localizer --ros-args \
  --params-file "$WS/mm_perception/config/aruco_localizer.yaml" \
  > "$LOG/loc.log" 2>&1 &
LOC=$!

sleep 3
echo "===== 相机启动日志 ====="
grep -v RTPS_TRANSPORT "$LOG/cam.log" | grep 启动 || echo "(相机未启动)"
echo "===== 距离曲线 ====="
python3 "$WS/mm_perception/test/probe_dolly.py" 2>/dev/null

kill "$CAM" "$LOC" 2>/dev/null
wait 2>/dev/null
exit 0
