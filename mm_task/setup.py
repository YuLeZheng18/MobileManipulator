from setuptools import find_packages, setup

package_name = 'mm_task'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/grasp.launch.py']),
        ('share/' + package_name + '/config', ['config/grasp.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fishros',
    maintainer_email='a2715187136@qq.com',
    description='顶层任务状态机:导航→对位→抓取→放托盘→搬运',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grasp_node = mm_task.grasp_node:main',
            # 'mission_manager = mm_task.mission_manager:main',
        ],
    },
)
