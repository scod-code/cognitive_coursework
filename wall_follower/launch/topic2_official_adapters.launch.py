from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Expose standard navigation topics for the official Route A atlas robot."""

    relay_adapters = Node(
        package='wall_follower',
        executable='topic2_relay_adapters',
        output='screen',
    )

    scan_adapter = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='topic2_route_a_scan_adapter',
        output='screen',
        remappings=[
            ('cloud_in', '/atlas/rgbd_camera/points'),
            ('scan', '/scan'),
        ],
        parameters=[
            {
                'target_frame': 'atlas/realsense',
                'transform_tolerance': 0.1,
                'min_height': -10.0,
                'max_height': 10.0,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087,
                'scan_time': 0.1,
                'range_min': 0.05,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            }
        ],
    )

    return LaunchDescription([
        relay_adapters,
        scan_adapter,
    ])
