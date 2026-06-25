from setuptools import setup

setup(
    name='arm_control',
    version='1.0.0',
    packages=['arm_control'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/arm_control']),
        ('share/arm_control', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='TODO@email.com',
    description='Arm control GUI package using PyQt',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_gui = arm_control.joint_gui:main',
            'can_bridge = arm_control.can_bridge:main',
        ],
    },
)