from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_path = get_package_share_directory('simple_robot_description')

    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf')
    world_file = os.path.join(pkg_path, 'worlds', 'diff_drive', 'detection_world.sdf')
    
    # Set GAZEBO_MODEL_PATH to find sign poster models
    gazebo_model_path = os.path.join(os.path.dirname(pkg_path), '..', '..', 'models')
    env = os.environ.copy()
    env['GAZEBO_MODEL_PATH'] = gazebo_model_path

    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    return LaunchDescription([

        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
            output='screen',
            env=env
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }]
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'simple_robot',
                '-topic', 'robot_description',
                '-x', '0',
                '-y', '0',
                '-z', '0.1'
            ],
            output='screen'
        )
    ])
