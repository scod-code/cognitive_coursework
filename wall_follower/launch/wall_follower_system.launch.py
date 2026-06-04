"""
Launch file for the complete wall-follower navigation system.

This launches:
- Wall follower node (PID control)
- Scan filter (pre-processing)
- OctoMap server (occupancy grid mapping)
- Static transforms (TF tree setup)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    wall_follower_params = DeclareLaunchArgument(
        'wall_follower_params',
        default_value=os.path.join(
            get_package_share_directory('wall_follower'),
            'config', 'wall_follower.yaml'
        ),
        description='Path to wall-follower config file'
    )
    
    # Wall follower node
    wall_follower_node = Node(
        package='wall_follower',
        executable='wall_follower_node',
        name='wall_follower',
        output='screen',
        parameters=[LaunchConfiguration('wall_follower_params')],
    )
    
    # Scan filter node (for pre-processing LIDAR data)
    scan_filter_node = Node(
        package='scan_filter',
        executable='scan_filter_node',
        name='scan_filter',
        output='screen',
    )
    
    # OctoMap server (3D occupancy grid mapping)
    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[
            {'resolution': 0.05},  # 5cm voxels
            {'frame_id': 'map'},
            {'base_frame_id': 'base_link'},
            {'sensor_model/max_range': 3.0},
            {'sensor_model/min_range': 0.1},
            {'ground_filter/enabled': True},
            {'ground_filter/max_height': 0.05},
        ],
        remappings=[
            ('/cloud_in', '/camera/depth/points'),  # or /scan if using LIDAR
        ]
    )
    
    return LaunchDescription([
        wall_follower_params,
        wall_follower_node,
        scan_filter_node,
        octomap_server_node,
    ])
