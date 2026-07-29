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

    # 放宽 trajectory_execution 的执行时长 upper bound. 默认 multiplier=2.0: planned_duration*2
    # 是控制器执行上限, 超了 move_group 直接 TIMED_OUT 中止动作. 但 MoveIt 的 cartesian path
    # (computeCartesianPath) 不读 setMaxVelocityScalingFactor, 轨迹时间戳按全速算 (~1s),
    # 而控制器有自己的 ros2_controllers.yaml 速度上限, 实际跑 ~2.5s. 2.0 倍率给 2.18s 上限,
    # 偶发超 0.3s 就 TIMED_OUT (2026-07-29 实跑验证). 提到 5.0 给足余量 (5s 上限), 不影响
    # 实际速度, 只放宽 abort 阈值.
    ld.add_action(SetParameter(
        name="trajectory_execution.allowed_execution_duration_multiplier",
        value=5.0))

    # 把官方生成的 move_group 启动项并入(SetParameter 在前, 对其生效)
    for action in generate_move_group_launch(moveit_config).entities:
        ld.add_action(action)
    return ld
