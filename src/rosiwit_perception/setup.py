from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'rosiwit_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('lib', package_name),
            ['scripts/avm_node_wrapper']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AI Development Team',
    maintainer_email='dev@rosiwit.com',
    description='360 Surround View (AVM) perception package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'avm_node = rosiwit_perception.avm_node:main',
        ],
    },
)
