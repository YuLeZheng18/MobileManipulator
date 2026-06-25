import os
import re

import launch
import launch_ros
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_dir = get_package_share_directory('mm_description')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    default_model_path = os.path.join(package_dir, 'urdf', 'mm_robot_gazebo.urdf.xacro')
    # 整车已内联进 world: gzserver 加载 world 时直接执行模型里的 plugin
    # (gazebo_ros2_control + planar_move), 不再用 spawn_entity 注入
    default_world_path = os.path.join(package_dir, 'world', 'mm_world.world')

    # 展开 xacro 并去掉 XML 注释 (humble gz_ros2_control bug #503: 注释会破坏
    # controller_manager 的 robot_description 参数解析, 见 PR #505)
    robot_desc_xml = xacro.process_file(default_model_path).toxml()
    robot_desc_xml = re.sub(r'<!--.*?-->', '', robot_desc_xml, flags=re.DOTALL)

    # 确保 Gazebo 能找到 ROS 插件 (libgazebo_ros2_control.so, planar_move 等)
    set_plugin_path = launch.actions.AppendEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value='/opt/ros/humble/lib',
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc_xml}, {'use_sim_time': True}],
        output='screen',
    )

    # 速度指令最后一级加速度限幅: nav2(Drive)/behavior(Spin) 都发 /cmd_vel, 经此节点限幅
    # 后输出 /cmd_vel_smoothed 给底盘. 电机无加减速斜坡, 这一级补上(尤其治 Spin 起转阶跃抖).
    # linear_accel 设高(≥nav2 velocity_smoother 的 2.5)使直行起步不被二次拖慢; angular 给自转斜坡.
    cmd_vel_smoother_node = launch_ros.actions.Node(
        package='mm_description',
        executable='cmd_vel_smoother.py',
        name='cmd_vel_smoother',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/cmd_vel',
            'output_topic': '/cmd_vel_smoothed',
            'linear_acceleration': 3.0,
            'angular_acceleration': 2.0,
            'rate': 50.0,
        }],
        output='screen',
    )

    gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': default_world_path}.items(),
    )

    # world 加载后 controller_manager 由模型内插件起来, 延迟激活控制器
    load_joint_state_broadcaster = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60'],
        output='screen',
    )

    load_arm_controller = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60'],
        output='screen',
    )

    # 依次激活: 先 joint_state_broadcaster, 退出后再 arm_controller
    delay_arm_after_jsb = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_arm_controller],
        )
    )

    # spawner 会持续轮询等待 controller_manager(timeout 60s), 所以这里早点拉起,
    # controller_manager 一就绪立刻激活, 把上电到锁定的"重力下垂窗口"压到最短
    delayed_jsb = launch.actions.TimerAction(period=4.0, actions=[load_joint_state_broadcaster])

    return launch.LaunchDescription([
        set_plugin_path,
        robot_state_publisher_node,
        cmd_vel_smoother_node,
        gazebo,
        delayed_jsb,
        delay_arm_after_jsb,
    ])
