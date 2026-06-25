from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("arm_description", package_name="arm_moveit_config").to_moveit_configs()

    ld = LaunchDescription()
    # 时间源同 move_group: 仅 Gazebo 传 use_sim_time:=true, mock/实机用默认 false,
    # 否则 RViz 的 TF/规划场景时间戳与实际时钟不一致.
    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="false"))
    ld.add_action(SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")))

    for action in generate_moveit_rviz_launch(moveit_config).entities:
        ld.add_action(action)
    return ld
