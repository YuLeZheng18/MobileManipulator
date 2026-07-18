"""仿真 sim-only mock launch: 起 mock_object_detector(查真值发 object_pose)
与 mock_suction(假吸附: /pump_cmd 驱动盒子吸附跟随/释放).

真机不用本 launch(真机起队友 mm_perception 真节点 + 真气泵固件); 话题接口一致, 上层无感.
"""

import launch
import launch_ros


def generate_launch_description():
    mock_object_detector = launch_ros.actions.Node(
        package='mm_bringup',
        executable='mock_object_detector.py',
        name='mock_object_detector',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    mock_suction = launch_ros.actions.Node(
        package='mm_bringup',
        executable='mock_suction.py',
        name='mock_suction',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return launch.LaunchDescription([
        mock_object_detector,
        mock_suction,
    ])
