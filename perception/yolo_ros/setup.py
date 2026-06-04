from glob import glob
from setuptools import setup
import os

package_name = 'yolo_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
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
            glob('launch/*.py')
        ),
    ],
    install_requires=['setuptools', 'psutil'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='YOLOv8 ROS2 integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = yolo_ros.yolo_node:main',
            'sign_controller = yolo_ros.sign_controller:main',
            'goal_publisher = yolo_ros.goal_publisher:main',
            'landmark_db = yolo_ros.landmark_db:main',
            'resource_monitor = yolo_ros.resource_monitor:main',
            'curiosity_explorer = yolo_ros.curiosity_explorer:main',
        ],
    },
)
