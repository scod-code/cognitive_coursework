from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    scan_filter_node = Node(
        package='scan_filter',
        executable='scan_filter_node',
        name='scan_filter',
        output='screen'
    )

    wall_follower_node = Node(
        package='wall_follower',
        executable='wall_follower_node',
        name='wall_follower',
        output='screen'
    )

    return LaunchDescription([
        scan_filter_node,
        wall_follower_node
    ])
