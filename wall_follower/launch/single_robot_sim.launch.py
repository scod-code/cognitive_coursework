"""
Single Robot Simulation Launch File

Complete simulation stack for the JetBot maze runner:
- Gazebo simulation
- Robot spawning
- Wall-following navigation
- OctoMap mapping
- TF chain
- RViz visualization
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Arguments
    world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory('simple_robot_description'),
            'worlds', 'diff_drive', 'detection_world.sdf'
        ),
        description='Gazebo world file'
    )
    
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Include maze launch
    maze_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wall_follower'),
                'launch', 'maze.launch.py'
            ])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
        }.items()
    )
    
    # Include TF chain setup
    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wall_follower'),
                'launch', 'odom_to_tf.launch.py'
            ])
        ])
    )
    
    # Include OctoMap
    octomap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wall_follower'),
                'launch', 'octomap.launch.py'
            ])
        ])
    )
    
    # Wall follower node
    wall_follower = Node(
        package='wall_follower',
        executable='wall_follower_node',
        name='wall_follower',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('wall_follower'),
                'config', 'wall_follower.yaml'
            ),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    return LaunchDescription([
        world,
        use_sim_time,
        maze_launch,
        tf_launch,
        octomap_launch,
        wall_follower,
    ])
