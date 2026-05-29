"""
Maze Simulation Launch File

Launches the complete Gazebo simulation of the maze environment.

This includes:
- Gazebo server and client
- Simulated robot
- Maze world with obstacles
- RViz visualization
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, FindExecutable
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory('simple_robot_description'),
            'worlds',
            'simple_world.world'
        ),
        description='Path to maze world file'
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
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            LaunchConfiguration('world'),
            '--verbose',
        ],
        output='screen',
        additional_env={
            'GAZEBO_RESOURCE_PATH': os.pathsep.join([
                os.path.join(get_package_share_directory('simple_robot_description'), 'worlds'),
                '/usr/share/gazebo-11',
                '/usr/share/gazebo-11/media'
            ]),
            'GAZEBO_MODEL_PATH': os.pathsep.join([
                os.path.join(get_package_share_directory('simple_robot_description'), 'models'),
                '/usr/share/gazebo-11/models'
            ]),
            'HOME': os.environ.get('HOME', '/root')
        }
    )

    # Gazebo GUI
    gzclient = ExecuteProcess(
        cmd=[FindExecutable(name='gzclient')],
        output='screen'
        ,
        additional_env={
            'HOME': os.environ.get('HOME', '/root')
        }
    )

    # Spawn robot with delay to let Gazebo initialize
    spawn_robot = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'sleep 5 && source /opt/ros/humble/setup.bash && '
            'ros2 run gazebo_ros spawn_entity.py '
            '-entity jetbot '
            '-file ' + os.path.join(
                get_package_share_directory('simple_robot_description'),
                'urdf',
                'robot.urdf'
            ) + ' '
            '-x 0 -y 0 -z 0.1'
        ],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[
            os.path.join(
                get_package_share_directory('simple_robot_description'),
                'urdf',
                'robot.urdf'
            )
        ],
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
        output='screen'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            os.path.join(
                get_package_share_directory('simple_robot_description'),
                'rviz',
                'robot.rviz'
            )
        ],
        output='screen'
    )

    # Ensure spawn_robot runs (embedded sleep handles timing)
    
    return LaunchDescription([
        world,
        use_sim_time,
        gzserver,
        gzclient,
        spawn_robot,
        robot_state_publisher,
        rviz,
    ])