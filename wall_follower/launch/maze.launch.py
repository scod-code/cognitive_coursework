"""
Maze Simulation Launch File

Launches the complete Gazebo simulation of the maze environment.

This includes:
- Gazebo server and client
- Simulated JetBot robot
- Maze world with obstacles
- Sensor simulation (camera, LiDAR)
- RViz for visualization
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Arguments
    world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory('simple_robot_description'),
            'worlds', 'maze.sdf'
        ),
        description='Path to maze world file'
    )
    
    headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode (no GUI)'
    )
    
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=[
            FindExecutable(name='gzserver'),
            LaunchConfiguration('world'),
            '--verbose',
        ],
        output='screen',
        env={
            'GAZEBO_RESOURCE_PATH': os.path.join(
                get_package_share_directory('simple_robot_description'),
                'worlds'
            )
        }
    )
    
    # Gazebo client (GUI)
    gzclient = ExecuteProcess(
        cmd=[FindExecutable(name='gzclient')],
        output='screen',
        condition=EqualsCondition(
            LaunchConfiguration('headless'),
            'false'
        )
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'jetbot',
            '-file', os.path.join(
                get_package_share_directory('simple_robot_description'),
                'urdf', 'jetbot.urdf'
            ),
            '-x', '0', '-y', '0', '-z', '0.1',
        ],
        output='screen'
    )
    
    # RViz visualization
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', os.path.join(
                get_package_share_directory('simple_robot_description'),
                'rviz', 'maze_view.rviz'
            )
        ],
        output='screen'
    )
    
    # TF broadcaster for robot state
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[
            os.path.join(
                get_package_share_directory('simple_robot_description'),
                'urdf', 'jetbot.urdf'
            )
        ],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    return LaunchDescription([
        world,
        headless,
        use_sim_time,
        gzserver,
        gzclient,
        spawn_robot,
        robot_state_publisher,
        rviz,
    ])
