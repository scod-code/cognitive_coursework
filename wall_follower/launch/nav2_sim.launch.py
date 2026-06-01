import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory('simple_robot_description'),
            'worlds',
            'simple_world.world'
        ),
        description='Gazebo world file'
    )

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    maze_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wall_follower'),
                'launch',
                'maze.launch.py'
            ])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wall_follower'),
                'launch',
                'nav2.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wall_follower'),
                'launch',
                'odom_to_tf.launch.py'
            ])
        ])
    )

    scan_to_pointcloud = Node(
        package='wall_follower',
        executable='scan_to_pointcloud',
        name='scan_to_pointcloud',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'scan_topic': '/scan'},
            {'cloud_topic': '/scan_cloud'},
            {'fixed_z': 0.0}
        ]
    )

    octomap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wall_follower'),
                'launch',
                'octomap.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'cloud_in': '/scan_cloud',
            'frame_id': 'odom',
            'max_range': '4.0',
        }.items()
    )

    return LaunchDescription([
        world,
        use_sim_time,
        maze_launch,
        tf_launch,
        nav2_launch,
        scan_to_pointcloud,
        octomap_launch,
    ])
