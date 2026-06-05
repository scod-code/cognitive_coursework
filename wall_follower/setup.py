from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'wall_follower'


def files(pattern):
    return [path for path in glob(pattern) if os.path.isfile(path)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
        (
            os.path.join('share', package_name, 'maps'),
            files('maps/*')
        ),
        (
            os.path.join('share', package_name, 'rviz'),
            files('rviz/*')
        ),
        (
            os.path.join('share', package_name, 'urdf'),
            files('urdf/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Wall following, Kalman filtering, mapping, and Nav2 navigation',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wall_follower_node = wall_follower.wall_follower_node:main',
            'amcl_initializer = wall_follower.amcl_initializer:main',
            'topic2_nav2_tf_helper = wall_follower.topic2_nav2_tf_helper:main',
            'topic2_goal_pose_bridge = wall_follower.topic2_goal_pose_bridge:main',
            'topic2_pointcloud_filter = wall_follower.topic2_pointcloud_filter:main',
            'topic2_relay_adapters = wall_follower.topic2_relay_adapters:main',
            'topic2_mapping_tf = wall_follower.topic2_mapping_tf:main',
            'topic2_yolo_counter = wall_follower.topic2_yolo_counter:main',
            'topic2_nav2_traffic_rules = wall_follower.topic2_nav2_traffic_rules:main',
            'topic2_yolo_landmark_bridge = wall_follower.topic2_yolo_landmark_bridge:main',
        ],
    },
)
