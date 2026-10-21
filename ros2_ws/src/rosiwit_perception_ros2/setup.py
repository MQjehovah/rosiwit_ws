from setuptools import setup
try:
    from setuptools.command.install_data import install_data as _install_data
except ImportError:
    from distutils.command.install_data import install_data as _install_data
import os
from glob import glob

package_name = 'rosiwit_perception_ros2'

_WRAPPERS = [
    'avm_node_wrapper',
    'ground_texture_mapper_wrapper',
    'qr_detector_node_wrapper',
    'obstacle_detector_node_wrapper',
    'perception_manager_wrapper',
]


class InstallDataWithExec(_install_data):
    """确保 libexec wrapper 带可执行位。

    data_files 复制时不一定保留 source 的 mode；某些 git 配置（core.fileMode
    false）也不会记录 mode。这里在安装结束时统一 chmod，避免 ros2 launch 报
    "executable not found"。
    """

    def run(self):
        super().run()
        if not self.install_dir or not os.path.isdir(self.install_dir):
            return
        for root, _dirs, files in os.walk(self.install_dir):
            if os.path.basename(root) != package_name:
                continue
            for name in _WRAPPERS:
                path = os.path.join(root, name)
                if os.path.isfile(path):
                    os.chmod(path, 0o755)

# 算法核心 (ROS 无关, 与接口包共享同一份源码) — 内联打包, 类比 C++ 的 add_subdirectory
core_pkg_dir = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', '..', '..', 'src', 'rosiwit_perception', 'rosiwit_perception'))

setup(
    name=package_name,
    version='0.2.0',
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
            [os.path.join('scripts', name) for name in _WRAPPERS]),
    ],
    cmdclass={'install_data': InstallDataWithExec},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AI Development Team',
    maintainer_email='dev@rosiwit.com',
    description='Modular perception (AVM / ground texture / QR) with unified health',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'avm_node = rosiwit_perception_ros2.avm_node:main',
            'ground_texture_mapper = rosiwit_perception_ros2.ground_texture_mapper:main',
            'qr_detector_node = rosiwit_perception_ros2.qr_detector_node:main',
            'obstacle_detector_node = rosiwit_perception_ros2.obstacle_detector_node:main',
            'perception_manager = rosiwit_perception_ros2.perception_manager:main',
        ],
    },
)
