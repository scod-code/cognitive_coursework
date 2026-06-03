from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'wall_follower'

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
            glob('maps/*')
        ),
        (
            os.path.join('share', package_name, 'models'),
            glob('models/*')
        ),
        (
            os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')
        ),
        (
            os.path.join('share', package_name),
            glob('README.md')
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
            'scan_to_pointcloud = wall_follower.scan_to_pointcloud:main',
            'pomdp_goal_selector = wall_follower.pomdp_goal_selector:main',
        ],
    },
)
