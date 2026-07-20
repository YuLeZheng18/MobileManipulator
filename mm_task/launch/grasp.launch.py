"""抓取节点 (MVP) launch.

前提 (本 launch 不代起, 需已在跑):
  - 真机臂 bringup (arm_controller + TF);
  - move_group (MoveIt 规划服务, 如 arm_moveit_config move_group.launch.py);
  - yolo_box_detector (发 /perception/object_pose);
  - 底盘气泵固件 (订阅 /pump_cmd).

用法:
  ros2 launch mm_task grasp.launch.py                    # 等 /grasp 服务触发
  ros2 launch mm_task grasp.launch.py dry_run:=true      # 只规划不动真机(验证)
  然后: ros2 service call /grasp std_srvs/srv/Trigger

真机会真动: 执行前确认吸盘朝下、周围无遮挡、急停就位.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    default_params = os.path.join(
        get_package_share_directory('mm_task'), 'config', 'grasp.yaml')
    params_file = LaunchConfiguration('params_file').perform(context) or default_params
    overrides = {}
    dr = LaunchConfiguration('dry_run').perform(context)
    if dr:
        overrides['dry_run'] = dr.lower() in ('1', 'true', 'yes', 'on')
    ag = LaunchConfiguration('auto_grasp').perform(context)
    if ag:
        overrides['auto_grasp'] = ag.lower() in ('1', 'true', 'yes', 'on')
    mo = LaunchConfiguration('move_only').perform(context)
    if mo:
        overrides['move_only'] = mo.lower() in ('1', 'true', 'yes', 'on')
    cg = LaunchConfiguration('contact_gap').perform(context)
    if cg:
        overrides['contact_gap'] = float(cg)
    params = [params_file] + ([overrides] if overrides else [])
    return [Node(package='mm_task', executable='grasp_node',
                 name='grasp_node', output='screen', parameters=params)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value='',
                              description='参数 yaml (留空用包内 config/grasp.yaml)'),
        DeclareLaunchArgument('dry_run', default_value='',
                              description='true=只规划不执行'),
        DeclareLaunchArgument('auto_grasp', default_value='',
                              description='true=启动即抓一次'),
        DeclareLaunchArgument('move_only', default_value='',
                              description='true=只移到预抓取位就停(不下插/不吸/不抬)'),
        DeclareLaunchArgument('contact_gap', default_value='',
                              description='吸盘尖距盒顶最终间隙(米). 负值=多下插补偿'
                                          'YOLO z偏高(实测停高8cm则设-0.08)'),
        OpaqueFunction(function=_setup),
    ])
