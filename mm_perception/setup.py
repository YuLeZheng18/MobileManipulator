import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mm_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.rviz')),
        # YOLO 权重: best.pt 随包安装, 供节点按 share 路径加载
        (os.path.join('share', package_name, 'models'),
            glob('models/*.pt')),
        # NCNN 导出目录 (ARM CPU 加速版): 只装文件, 排除 __pycache__ 等子目录
        (os.path.join('share', package_name, 'models', 'best_ncnn_model'),
            [f for f in glob('models/best_ncnn_model/*') if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fishros',
    maintainer_email='a2715187136@qq.com',
    description='视觉感知:ArUco 定位与盒子抓取位姿识别',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_localizer = mm_perception.aruco_localizer:main',
            'hand_eye_calibrator = mm_perception.hand_eye_calibrator:main',
            'hand_eye_camera_check = mm_perception.hand_eye_camera_check:main',
            'hand_eye_verify = mm_perception.hand_eye_verify:main',
            'yolo_box_detector = mm_perception.yolo_box_detector:main',
        ],
    },
)
