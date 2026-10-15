from setuptools import setup
import os
from glob import glob

package_name = 'rosiwit_perception_ros2'

# 算法核心 (ROS 无关, 与接口包共享同一份源码) — 内联打包, 类比 C++ 的 add_subdirectory
core_pkg_dir = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', '..', '..', 'src', 'rosiwit_perception', 'rosiwit_perception'))

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, 'rosiwit_perception'],
    package_dir={
        package_name: package_name,
        'rosiwit_perception': core_pkg_dir,
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('lib', package_name),
            ['scripts/avm_node_wrapper', 'scripts/ground_texture_mapper_wrapper']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AI Development Team',
    maintainer_email='dev@rosiwit.com',
    description='360 Surround View (AVM) perception package (ROS2 thin interface)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'avm_node = rosiwit_perception_ros2.avm_node:main',
            'ground_texture_mapper = rosiwit_perception_ros2.ground_texture_mapper:main',
        ],
    },
)
