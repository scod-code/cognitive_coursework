"""
OctoMap Server Launch File

Launches the OctoMap server for 3D occupancy grid mapping.
This node builds and maintains a 3D voxel map from depth sensor data.

Key features:
- Ground plane filtering to exclude the floor from the map
- Sensor model configuration (max/min range, occlusion)
- Ray casting for efficient occupancy updates
- 2D projection for Nav2 compatibility
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    resolution = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='OctoMap voxel resolution (meters)'
    )
    
    max_range = DeclareLaunchArgument(
        'max_range',
        default_value='3.0',
        description='Maximum sensor range (meters)'
    )

    cloud_in = DeclareLaunchArgument(
        'cloud_in',
        default_value='/scan_cloud',
        description='Input point cloud topic for OctoMap'
    )

    frame_id = DeclareLaunchArgument(
        'frame_id',
        default_value='odom',
        description='Output frame for OctoMap topics'
    )

    base_frame_id = DeclareLaunchArgument(
        'base_frame_id',
        default_value='base_link',
        description='Robot base frame used by OctoMap'
    )
    
    # OctoMap Server Node
    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'resolution': LaunchConfiguration('resolution'),
                'frame_id': LaunchConfiguration('frame_id'),
                'base_frame_id': LaunchConfiguration('base_frame_id'),
                
                # Sensor model (RealSense D435)
                'sensor_model/max_range': LaunchConfiguration('max_range'),
                'sensor_model/min_range': 0.1,
                'sensor_model/hit': 0.7,      # Log-odds hit probability
                'sensor_model/miss': 0.4,     # Log-odds miss probability
                
                # Ground filtering (CRITICAL for Nav2 compatibility)
                # This prevents the floor from being mapped as occupied
                'ground_filter/enabled': True,
                'ground_filter/max_height': 0.05,      # Maximum height for ground plane (m)
                'ground_filter/min_height': -0.5,      # Minimum height to map
                
                # Map update parameters
                'map/publish_free_space': True,        # Publish free space
                'map/initial_tree_depth': 16,
                
                # Output topics
                'map/occupancy_grid_out': '/projected_map',  # For Nav2
                'map/occupancy_3d_out': '/octomap_full',     # Full 3D map
            }
        ],
        remappings=[
            ('/cloud_in', LaunchConfiguration('cloud_in')),
        ]
    )
    
    return LaunchDescription([
        use_sim_time,
        resolution,
        max_range,
        cloud_in,
        frame_id,
        base_frame_id,
        octomap_server,
    ])
