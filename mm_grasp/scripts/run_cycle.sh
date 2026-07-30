#!/bin/bash
# 一键连跑 N 个盒子: ready 一次, 然后每盒 look -> execute. 任一步失败即停.
#
# 用法: ./run_cycle.sh [盒数, 默认 4]
# 工作区路径用 MM_WS 覆盖 (Nano 默认 ~/Desktop/moveit, 本机是 ~/MobileManipulator/workplace).
#
# ⚠️ 背靠背连调暴露过 look 段缺 settle 的 bug (臂未停稳时感知坐标随 TF 滑动).
#    手动一步步调有人为间隔, 只有这个脚本能复现, 故留作回归手段.
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
MM_WS=${MM_WS:-~/Desktop/moveit}
source /opt/ros/humble/setup.bash
source "$MM_WS/install/setup.bash"
N=${1:-4}
call() { ros2 service call "$1" std_srvs/srv/Trigger 2>&1 | tail -2; }
echo "==== ready ===="
call /grasp/ready
for i in $(seq 1 "$N"); do
  echo "==== 第 $i 盒: look ===="
  out=$(call /grasp/look); echo "$out"
  echo "$out" | grep -q "success=True" || { echo "look 失败, 停"; exit 1; }
  echo "==== 第 $i 盒: execute ===="
  out=$(call /grasp/execute); echo "$out"
  echo "$out" | grep -q "success=True" || { echo "execute 失败, 停"; exit 1; }
done
echo "==== 全部 $N 盒完成 ===="
