"""仿真 mock 感知 launch: 起 mock_object_detector (查 Gazebo 真值发 object_pose).

真机不用本 launch (真机起队友 mm_perception 真节点); 二者输出话题一致, 上层无感.
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

    return launch.LaunchDescription([
        mock_object_detector,
    ])
