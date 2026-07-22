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
    # 整车从 robot_description 现起 spawn 注入(spawn_entity), world 里不再内联机器人.
    # 好处: 改 URDF/机械零位后只需重启即生效, 不必重新导出 world 内联 SDF.
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

    # spawn_entity 会把 URDF 的 package://mm_description/meshes/*.stl 转成
    # model://mm_description/meshes/*.stl. 让 gzserver/gzclient 能解析 model:// ,
    # GAZEBO_MODEL_PATH 需含"包含 mm_description 目录的父目录"(= share 目录).
    # 缺此项时所有网格 Failed to find mesh file, 整车在 GUI 里不可见(看似卡住).
    set_model_path = launch.actions.AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.dirname(package_dir),
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

    # 从 robot_description 现起 spawn 整车. 位姿对齐原内联模型: yaw=pi(车头朝 world -x),
    # 使"货物右置"几何(box world +y = base_link -y)保持不变.
    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'mm_robot',
                   '-x', '0', '-y', '0', '-z', '0.05', '-Y', '3.14159265'],
        output='screen',
    )
    # 等 gzserver 的 /spawn_entity 服务起来再注入
    delayed_spawn = launch.actions.TimerAction(period=4.0, actions=[spawn_entity])

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

    # spawn 完成后 gazebo_ros2_control 插件才加载出 controller_manager, 故 jsb 待 spawn 退出再起.
    # spawner 自带 60s 轮询, controller_manager 一就绪立刻激活, 把上电到锁定的"重力下垂窗口"压到最短.
    load_jsb_after_spawn = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    return launch.LaunchDescription([
        set_plugin_path,
        set_model_path,
        robot_state_publisher_node,
        cmd_vel_smoother_node,
        gazebo,
        delayed_spawn,
        load_jsb_after_spawn,
        delay_arm_after_jsb,
    ])
