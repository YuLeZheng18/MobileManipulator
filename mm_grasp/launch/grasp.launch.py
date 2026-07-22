# mm_grasp launch: 起 moveit_servo(servo_node) + grasp_node.
# 前置(用户既有 launch 另行起): gazebo_mm(带 ros2_control/arm_controller) + move_group + mock 感知/吸附.
#   ros2 launch mm_description gazebo_mm.launch.py
#   ros2 launch arm_moveit_config move_group.launch.py   (或 demo.launch.py 带 rviz)
#   ros2 launch mm_bringup mock_perception.launch.py
# 本 launch 只补 servo_node 与 grasp_node 两个 mm_grasp 专属节点.
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, rel_path):
    path = os.path.join(get_package_share_directory(package_name), rel_path)
    with open(path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    # use_sim_time: 仿真跟 Gazebo /clock 传 true; 实机 (无 /clock, real_bringup 起) 传 false.
    use_sim_time = LaunchConfiguration("use_sim_time")

    moveit_config = MoveItConfigsBuilder(
        "arm_description", package_name="arm_moveit_config"
    ).to_moveit_configs()

    servo_yaml = load_yaml("mm_grasp", "config/servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": use_sim_time},
        ],
    )

    grasp_node = Node(
        package="mm_grasp",
        executable="grasp_node",
        name="grasp_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="仿真 true 跟 /clock; 实机 false"),
        servo_node,
        grasp_node,
    ])
