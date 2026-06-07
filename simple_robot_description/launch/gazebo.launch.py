from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_path = get_package_share_directory('simple_robot_description')

    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf')
    world_file = os.path.join(pkg_path, 'worlds', 'diff_drive', 'detection_world.sdf')

    # GAZEBO_MODEL_PATH must include our custom models (sign posters)
    gazebo_model_path = os.path.join(pkg_path, 'models')
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    full_model_path = gazebo_model_path + os.pathsep + existing_model_path if existing_model_path else gazebo_model_path

    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    return LaunchDescription([

        # Set model path so Gazebo can find sign poster models
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', full_model_path),

        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose', world_file,
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
            ],
            output='screen',
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
                '-z', '0.1',
                '-timeout', '120',
            ],
            output='screen'
        )
    ])
