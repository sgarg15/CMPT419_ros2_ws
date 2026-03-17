import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ros2d2'

# Data files: colcon requires relative paths
launch_files = glob('launch/*.py')
world_files = glob('worlds/*.world')
config_files = glob('config/*.yaml')
rviz_files = glob('config/*.rviz')

data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]
if launch_files:
    data_files.append((os.path.join('share', package_name, 'launch'), launch_files))
if world_files:
    data_files.append((os.path.join('share', package_name, 'worlds'), world_files))
if config_files:
    data_files.append((os.path.join('share', package_name, 'config'), config_files))
if rviz_files:
    data_files.append((os.path.join('share', package_name, 'config'), rviz_files))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='sat.garg03@gmail.com',
    description='ROS2 package ros2d2',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ros2d2_node = ros2d2.ros2d2_node:main',
            'state_fusion_node = ros2d2.nodes.state_fusion_node:main',
            'game_controller_node = ros2d2.nodes.game_controller_node:main',
            'px4_bridge_defender_node = ros2d2.nodes.px4_bridge_defender_node:main',
            'px4_bridge_attacker_node = ros2d2.nodes.px4_bridge_attacker_node:main',
            'drone_markers_node = ros2d2.nodes.drone_markers_node:main',
            'capture_detection_node = ros2d2.nodes.capture_detection_node:main',
        ],
    },
)
