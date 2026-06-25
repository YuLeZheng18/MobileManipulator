#!/usr/bin/env bash
# 从 mm_robot_gazebo.urdf.xacro 重新生成内联整车的 Gazebo world 文件.
# 改了 xacro(机械臂/底盘/插件/摩擦等)后跑这个脚本, 再 colcon build 即可.
#
# 流程: xacro 展开 -> 去 XML 注释(humble gz_ros2_control bug #503)
#       -> gz sdf 转 SDF -> model:// 换成绝对 file://(gazebo classic 不认 model://)
#       -> 注入 room.world(带车头 yaw=180° 初始 pose) -> 写 mm_world.world
set -e

WS=/home/fishros/MobileManipulator/workplace
cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash

MODEL="$WS/install/mm_description/share/mm_description/urdf/mm_robot_gazebo.urdf.xacro"
MESH_DIR="$WS/install/mm_description/share/mm_description"
SRC_WORLD="$WS/src/mm_description/world/room.word"
OUT_WORLD="$WS/src/mm_description/world/mm_world.world"
SPAWN_POSE="0 0 0.05 0 0 3.14159265"   # x y z roll pitch yaw, yaw=π 让车头朝开阔方向

python3 - "$MODEL" "$MESH_DIR" "$SRC_WORLD" "$OUT_WORLD" "$SPAWN_POSE" <<'PY'
import re, sys, subprocess, tempfile, os
import xacro

model, mesh_dir, src_world, out_world, spawn_pose = sys.argv[1:6]

# 1) xacro -> urdf, 去注释
x = xacro.process_file(model).toxml()
x = re.sub(r'<!--.*?-->', '', x, flags=re.DOTALL)
with tempfile.NamedTemporaryFile('w', suffix='.urdf', delete=False) as f:
    f.write(x); urdf_path = f.name

# 2) urdf -> sdf
sdf = subprocess.check_output(['gz', 'sdf', '-p', urdf_path], text=True,
                              stderr=subprocess.DEVNULL)

# 3) model://mm_description -> 绝对 file://
sdf = sdf.replace('model://mm_description', 'file://' + mesh_dir)

# 4) 抽 <model> 块, 加 spawn pose
m = re.search(r'(<model name=.mm_robot.>.*</model>)', sdf, re.DOTALL)
block = m.group(1).replace("<model name='mm_robot'>",
                           f"<model name='mm_robot'>\n    <pose>{spawn_pose}</pose>", 1)
block = '\n'.join(('    ' + ln) if ln.strip() else ln for ln in block.splitlines())

# 5) 注入 room.world 的 </world> 前
room = open(src_world).read()
out = room.replace('  </world>', block + '\n  </world>', 1)
open(out_world, 'w').write(out)
os.unlink(urdf_path)

print(f'  生成 {out_world}')
print(f'  plugin 数: {out.count("<plugin")}  mesh file:// 数: {out.count("file://" + mesh_dir)}')
print(f'  车头 yaw=π: {"3.14159265" in out}')
PY

echo "完成. 现在跑: colcon build --packages-select mm_description --symlink-install"
