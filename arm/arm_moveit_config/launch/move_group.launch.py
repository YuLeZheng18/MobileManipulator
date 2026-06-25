from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("arm_description", package_name="arm_moveit_config").to_moveit_configs()

    ld = LaunchDescription()
    # 时间源必须与控制器/硬件一致, 否则时间戳不匹配会让 MoveIt 规划的轨迹被 controller 当作过期丢弃(规划后不动).
    # 仅当有 /clock 发布者(Gazebo)时传 use_sim_time:=true; mock demo 与实机(无 /clock)用默认 false,
    # 否则 controller_manager 时间冻在 0、executor 卡死、控制器起不来.
    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="false"))
    ld.add_action(SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")))

    # 把官方生成的 move_group 启动项并入(SetParameter 在前, 对其生效)
    for action in generate_move_group_launch(moveit_config).entities:
        ld.add_action(action)
    return ld
