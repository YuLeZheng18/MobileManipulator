#!/usr/bin/env bash
# 自包含验证真机相机链路: 起 usb_cam+定位 -> 测帧率/编码/检测 -> 回收.
# 用法: bash verify_real_cam.sh [video设备, 默认/dev/video2]
WS=/home/dong/Desktop/moveit
DEV="${1:-/dev/v4l/by-id/usb-Generic_PC_Camera_A2_200901010001-video-index0}"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
LOG=/tmp/real2; mkdir -p "$LOG"

pkill -9 -f usb_cam 2>/dev/null; pkill -9 -f aruco_localizer 2>/dev/null
sleep 1

ros2 launch mm_perception aruco_real.launch.py \
  video_device:="$DEV" with_viewer:=false > "$LOG/launch.log" 2>&1 &
LP=$!
sleep 7

echo "===== 图像帧率 (4s) ====="
timeout 5 ros2 topic hz /camera/image_raw 2>&1 | grep -m1 average || echo "(无图像)"
echo "===== 图像编码 ====="
timeout 4 ros2 topic echo /camera/image_raw --field encoding --once 2>/dev/null || echo "(取不到)"
echo "===== 相机内参 K (应 fx=fy=600, 非全0) ====="
timeout 4 ros2 topic echo /camera/camera_info --field k --once 2>/dev/null || echo "(取不到)"
echo "===== usb_cam / 定位 关键日志 ====="
grep -v -iE "RTPS_TRANSPORT" "$LOG/launch.log" \
  | grep -iE "usb_cam|error|failed|已获取|已广播|等待|无法|不支持" | tail -12

kill "$LP" 2>/dev/null
pkill -9 -f usb_cam 2>/dev/null; pkill -9 -f aruco_localizer 2>/dev/null
wait 2>/dev/null
exit 0
