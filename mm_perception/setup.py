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
        ('share/' + package_name + '/launch', []),
        ('share/' + package_name + '/config', []),
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
            # 示例(待实现):
            # 'aruco_localizer = mm_perception.aruco_localizer:main',
            # 'grasp_pose_publisher = mm_perception.grasp_pose_publisher:main',
        ],
    },
)
