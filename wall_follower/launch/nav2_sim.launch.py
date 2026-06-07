from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    maze_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wall_follower'),
                'launch',
                'maze.launch.py'
            ])
        ])
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wall_follower'),
                'launch',
                'nav2.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        maze_launch,
        nav2_launch,
    ])
