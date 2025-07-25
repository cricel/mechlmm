from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mechlmm_bringup'

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
    maintainer='cricel',
    maintainer_email='cricel.design@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_core = mechlmm_bringup.robot_core:main',
            'lmm_commander = mechlmm_bringup.lmm_commander:main',
            'function_pool_definition = mechlmm_bringup.function_pool_definition:main',
            'test = mechlmm_bringup.test:main',
        ],
    },
)
