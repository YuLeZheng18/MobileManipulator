"""YOLO 盒子识别 + 抓取位姿 (深度相机 Link_30, 契约 §1 抓取版).

彩色图 YOLO 出框 (自训练模型, 类别 1/2/3/4) + 对齐深度定位, 发布盒子顶面中心
位姿 /perception/object_pose (PoseStamped, base_link 系).

前提 (本 launch 不代起, 需先各自就绪):
  - 深度相机(RealSense)驱动已起, 发 color/image_raw + aligned_depth_to_color/image_raw
    + color/camera_info (align_depth 必须启动时开, 否则彩色框落不到深度上);
  - 整车 robot_state_publisher 已起 (TF 树含 Link_30 -> ... -> base_link),
    否则查不到 TF 只打印像素深度, 不发 pose. 可传 with_rsp:=true 顺带起.

用法:
  ros2 launch mm_perception yolo_box_detector.launch.py
  ros2 launch mm_perception yolo_box_detector.launch.py with_rsp:=true       # 顺带发整车 TF
  ros2 launch mm_perception yolo_box_detector.launch.py model:=/abs/xxx.pt   # 换权重
  ros2 launch mm_perception yolo_box_detector.launch.py params_file:=/path/to/my.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    percep_share = get_package_share_directory('mm_perception')
    default_params = os.path.join(percep_share, 'config', 'yolo_box_detector.yaml')
    params_file = LaunchConfiguration('params_file').perform(context) or default_params
    with_rsp = LaunchConfiguration('with_rsp').perform(context).lower() == 'true'
    with_jsp = LaunchConfiguration('with_jsp').perform(context).lower() == 'true'

    # 模型路径: 传 model:= 则用它(相对名按 models/ 拼接, 绝对路径原样);
    #   留空 -> 不覆盖, 由 params_file(yaml) 的 model_path 决定, 但相对名需拼成 share 路径.
    #   默认 yaml 用 best_ncnn_model (NCNN 目录, ARM CPU 加速 ~4.8x).
    model = LaunchConfiguration('model').perform(context)
    overrides = {}
    if not model:
        # 读 yaml 里的 model_path (相对名), 拼成 share/models/ 下绝对路径
        import yaml as _yaml
        with open(params_file) as f:
            y = _yaml.safe_load(f) or {}
        mp = (y.get('yolo_box_detector', {}).get('ros__parameters', {})
              .get('model_path', 'best_ncnn_model'))
        if not os.path.isabs(mp):
            mp = os.path.join(percep_share, 'models', mp)
        overrides['model_path'] = mp
    else:
        overrides['model_path'] = model if os.path.isabs(model) \
            else os.path.join(percep_share, 'models', model)

    # yaml 里 model_path 是相对文件名, 这里用解析后的绝对路径覆盖 (override 排在后面).
    nodes = [Node(
        package='mm_perception', executable='yolo_box_detector',
        name='yolo_box_detector', output='screen',
        parameters=[params_file, overrides],
    )]

    # 可选整车 TF: 节点要查 Link_30 -> base_link, 需 rsp 把 TF 树发出来.
    #   注意: base_link->Link_30 中间有 6 个 revolute 关节(Joint_11~16, 机械臂),
    #   rsp 对活动关节必须靠 /joint_states 才能算 TF. 无真机 ros2_control/CAN 桥时
    #   /joint_states 无人发 -> 臂链断裂 -> Link_30 与 base_link 不连通 -> 查不到 TF.
    #   故 with_rsp 时自动附带 joint_state_publisher 发默认关节值(默认 0 位)让树连通.
    #   接了真机(有真实 /joint_states)则不需要, 用 with_jsp:=false 关掉避免冲突.
    if with_rsp:
        desc_share = get_package_share_directory('mm_description')
        with open(os.path.join(desc_share, 'urdf', 'mm_robot.urdf')) as f:
            robot_desc = f.read()
        nodes.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            output='screen', parameters=[{'robot_description': robot_desc}],
        ))
        if with_jsp:
            nodes.append(Node(
                package='joint_state_publisher', executable='joint_state_publisher',
                output='screen', parameters=[{'robot_description': robot_desc}],
            ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='YOLO 盒子识别参数 yaml (留空用包内默认 config/yolo_box_detector.yaml)'),
        DeclareLaunchArgument(
            'model', default_value='',
            description='YOLO 权重路径 (留空用包内 share/models/best.pt; 相对名按 models/ 拼接)'),
        DeclareLaunchArgument(
            'with_rsp', default_value='false',
            description='true=同时起 robot_state_publisher 发整车 TF (Link_30->base_link)'),
        DeclareLaunchArgument(
            'with_jsp', default_value='true',
            description='true(默认)=with_rsp 时附带 joint_state_publisher 发默认关节值, '
                        '让含活动关节的臂链连通; 接真机(有真实 /joint_states)时设 false'),
        OpaqueFunction(function=_setup),
    ])
