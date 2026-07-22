"""仿真总 launch (sim-only) — 一键起逻辑级全流程栈 (架构 §6/§7.1 自底向上).

分阶段起 (TimerAction 错峰, 上层等下层就绪):
  t=0   Gazebo(整车 spawn + grasp_box + ros2_control)          [mm_description]
  t=10  MoveIt move_group + Nav2(+AMCL+RViz) + mock 感知/吸附 + lane_navigator
  t=16  moveit_servo + grasp_node (依赖 move_group)             [mm_grasp]
  t=25  mm_task 状态机 (最后起, 仅当 run_mission:=true)          [mm_task]

参数:
  use_sim_time (默认 true) — 跟 Gazebo /clock
  run_mission  (默认 false) — false: 只起栈, 手动摆场景后再单独
                 `ros2 launch mm_task mission.launch.py` 触发;
                 true: 起栈后自动跑 S0→S5 (⚠️ 需先保证 grasp_box 在 nav 终点可达)

真机不用本 launch: 真机起队友真感知节点 + 真驱动, 话题接口一致, 上层无感 (架构 §4).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(pkg, rel, **kwargs):
    # GroupAction 作用域隔离: 被包含 launch 里 DeclareLaunchArgument 设的配置(如
    # gazebo_ros gzserver 的 params_file='')不会泄漏到兄弟 include。否则 nav2 的
    # navigation2.launch.py 读到已被置空的 params_file, RewrittenYaml open('') 崩栈。
    # scoped=True(默认) 挡泄漏, forwarding=True(默认) 让 use_sim_time 等父配置仍传入。
    path = os.path.join(get_package_share_directory(pkg), 'launch', rel)
    inc = IncludeLaunchDescription(PythonLaunchDescriptionSource(path), **kwargs)
    return GroupAction([inc])


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    run_mission = LaunchConfiguration('run_mission')

    place_box = LaunchConfiguration('place_box')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='跟 Gazebo /clock'),
        DeclareLaunchArgument('run_mission', default_value='false',
                              description='true=起栈后自动跑 mm_task 状态机; false=只起栈手动触发'),
        DeclareLaunchArgument('place_box', default_value=run_mission,
                              description='起 place_box_helper(到 p2 自动摆盒助绿跑); 默认跟 run_mission'),
    ]

    sim_arg = {'use_sim_time': use_sim_time}.items()

    # 阶段1 (t=0): Gazebo + spawn 整车 + grasp_box + ros2_control
    gazebo = _include('mm_description', 'gazebo_mm.launch.py')

    # 阶段2 (t=10): MoveIt + Nav2(含 AMCL/RViz) + mock 感知/吸附 + lane_navigator
    move_group = _include('arm_moveit_config', 'move_group.launch.py', launch_arguments=sim_arg)
    nav2 = _include('mm_navigation', 'navigation2.launch.py', launch_arguments=sim_arg)
    mock = _include('mm_bringup', 'mock_perception.launch.py')
    lane_navigator = Node(
        package='mm_navigation', executable='lane_navigator.py', name='lane_navigator',
        output='screen', parameters=[{'use_sim_time': use_sim_time}])
    place_box_helper = Node(
        package='mm_bringup', executable='place_box_helper.py', name='place_box_helper',
        output='screen', parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(place_box))
    stage2 = TimerAction(period=10.0,
                         actions=[move_group, nav2, mock, lane_navigator, place_box_helper])

    # 阶段3 (t=16): moveit_servo + grasp_node (依赖 move_group 已起)
    grasp = _include('mm_grasp', 'grasp.launch.py')
    stage3 = TimerAction(period=16.0, actions=[grasp])

    # 阶段4 (t=25): mm_task 状态机, 最后起; 仅当 run_mission:=true
    mission = _include('mm_task', 'mission.launch.py',
                       launch_arguments=sim_arg, condition=IfCondition(run_mission))
    stage4 = TimerAction(period=25.0, actions=[mission])

    return LaunchDescription(args + [gazebo, stage2, stage3, stage4])
