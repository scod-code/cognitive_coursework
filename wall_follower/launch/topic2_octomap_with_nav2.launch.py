from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Run OctoMap during Route A Nav2 without publishing Nav2 TF frames."""

    use_sim_time = LaunchConfiguration('use_sim_time')
    resolution = LaunchConfiguration('resolution')
    max_range = LaunchConfiguration('max_range')
    publish_camera_tf = LaunchConfiguration('publish_camera_tf')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation time',
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

    declare_publish_camera_tf = DeclareLaunchArgument(
        'publish_camera_tf',
        default_value='true',
        description=(
            'Publish atlas/base_link -> atlas/realsense if Nav2 does not '
            'already provide the RealSense frame'
        ),
    )

    # This is the only TF published by this launch. Nav2 remains responsible for
    # map -> odom -> atlas/base_link, so this launch can run beside Nav2 safely.
    camera_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='topic2_realsense_static_tf',
        output='screen',
        arguments=[
            '0.10',
            '0.0',
            '0.19',
            '0.0',
            '0.0',
            '0.0',
            'atlas/base_link',
            'atlas/realsense',
        ],
        condition=IfCondition(publish_camera_tf),
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
        name='topic2_nav2_octomap_server',
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

    return LaunchDescription([
        declare_use_sim_time,
        declare_resolution,
        declare_max_range,
        declare_publish_camera_tf,
        camera_static_tf,
        pointcloud_filter,
        TimerAction(period=3.0, actions=[octomap_server]),
    ])
