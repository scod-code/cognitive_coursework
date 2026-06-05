import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('wall_follower')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    resolution = LaunchConfiguration('resolution')
    max_range = LaunchConfiguration('max_range')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation time',
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for OctoMap evidence',
    )

    declare_resolution = DeclareLaunchArgument(
        'resolution',
        default_value='0.08',
        description='OctoMap voxel resolution in meters',
    )

    declare_max_range = DeclareLaunchArgument(
        'max_range',
        default_value='4.0',
        description='Maximum RGB-D point cloud insertion range in meters',
    )

    mapping_tf = Node(
        package='wall_follower',
        executable='topic2_mapping_tf',
        name='topic2_mapping_tf',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    pointcloud_filter = Node(
        package='wall_follower',
        executable='topic2_pointcloud_filter',
        name='topic2_pointcloud_filter',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'min_x': 0.15,
                'max_x': max_range,
                'min_y': -2.5,
                'max_y': 2.5,
                'min_z': -0.10,
                'max_z': 1.50,
                'floor_z': 0.05,
                'downsample_step': 4,
            }
        ],
    )

    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='topic2_official_octomap_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'resolution': resolution,
                'frame_id': 'map',
                'base_frame_id': 'atlas/base_link',
                'sensor_model.max_range': max_range,
                'sensor_model.max': 0.90,
                'sensor_model.min': 0.12,
                'sensor_model.hit': 0.7,
                'sensor_model.miss': 0.4,
                'publish_free_space': True,
                'incremental_2D_projection': True,
                'filter_ground_plane': False,
                'point_cloud_min_z': 0.02,
                'point_cloud_max_z': 2.0,
                'occupancy_min_z': 0.0,
                'occupancy_max_z': 2.0,
            }
        ],
        remappings=[
            ('cloud_in', '/atlas/rgbd_camera/points_filtered'),
            ('octomap_binary', '/octomap_binary'),
            ('octomap_full', '/octomap_full'),
            ('occupied_cells_vis_array', '/occupied_cells_vis_array'),
            ('projected_map', '/projected_map'),
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_topic2_octomap',
        output='screen',
        arguments=[
            '-d',
            os.path.join(pkg_share, 'rviz', 'topic2_octomap.rviz'),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rviz,
        declare_resolution,
        declare_max_range,
        mapping_tf,
        pointcloud_filter,
        TimerAction(period=4.0, actions=[octomap_server, rviz]),
    ])
