from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mechllm_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atom',
    maintainer_email='atom@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mechllm_qa = mechllm_bringup.mechllm_qa:main',
            'vision = mechllm_bringup.vision:main',
            'arm_control = mechllm_bringup.arm_control:main',
            'robot_core = mechllm_bringup.robot_core:main',
            'operator_cmd = mechllm_bringup.operator_cmd:main',
            'arm_cmd = mechllm_bringup.arm_cmd:main',
        ],
    },
)
