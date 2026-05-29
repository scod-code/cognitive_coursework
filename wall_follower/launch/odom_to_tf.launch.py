"""
TF (Transform) Chain Setup Launch File

Establishes the complete transform chain needed for navigation:
    map → odom → base_link → realsense_link → realsense_depth_frame

This is CRITICAL for OctoMap + Nav2 integration. Without a complete TF chain,
the pipeline fails silently with coordinate frame errors.

Transform hierarchy:
- map: Global coordinate frame (origin at maze start, fixed)
- odom: Odometry frame (drifts over time, updated by SLAM/odometry nodes)
- base_link: Robot center/base frame (updated by odometry)
- realsense_link: Camera mounting point on robot
- realsense_depth_frame: Actual camera depth frame
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Arguments for transform offsets
    camera_x = DeclareLaunchArgument(
        'camera_x',
        default_value='0.0',
        description='Camera X offset from base_link (meters)'
    )
    camera_y = DeclareLaunchArgument(
        'camera_y',
        default_value='0.0',
        description='Camera Y offset from base_link (meters)'
    )
    camera_z = DeclareLaunchArgument(
        'camera_z',
        default_value='0.05',
        description='Camera Z offset from base_link (meters)'
    )
    
    # Static transform: base_link → realsense_link
    # This defines where the camera is mounted on the robot
    base_to_realsense = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_realsense_tf',
        arguments=[
            LaunchConfiguration('camera_x'), LaunchConfiguration('camera_y'), LaunchConfiguration('camera_z'),
            '0', '0', '0',      # No rotation (pitch, roll, yaw in radians)
            'base_link', 'realsense_link'
        ]
    )
    
    # Static transform: realsense_link → realsense_depth_frame
    # RealSense intrinsic offset (typically 0 for D435)
    realsense_to_depth = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='realsense_to_depth_tf',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'realsense_link', 'realsense_depth_frame'
        ]
    )
    
    # Odometry to TF Publisher
    # This node reads odometry messages and publishes the odom → base_link transform
    # It allows the robot's position/orientation to be tracked in the odometry frame
    odom_to_tf = Node(
        package='tf2_ros',
        executable='odom_to_tf',
        name='odom_to_tf',
        parameters=[
            {'odom_frame': 'odom'},
            {'base_frame': 'base_link'},
        ]
    )
    
    # Map to Odom Transform (static initial transform)
    # This assumes the maze origin (map frame) coincides with the robot's starting position (odom frame)
    # In real SLAM, this would be updated dynamically
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=[
            '0', '0', '0',      # No initial offset
            '0', '0', '0',      # No initial rotation
            'map', 'odom'
        ]
    )
    
    return LaunchDescription([
        camera_x,
        camera_y,
        camera_z,
        base_to_realsense,
        realsense_to_depth,
        map_to_odom,
        # odom_to_tf,  # Uncomment when you have actual odometry data
    ])
