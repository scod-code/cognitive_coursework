import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ntu_share = get_package_share_directory('ntu_robotsim')
    robot_share = get_package_share_directory('simple_robot_description')

    world_file = os.path.join(
        ntu_share,
        'worlds',
        'cwmaze',
        'cwmazenew.sdf'
    )

    urdf_file = os.path.join(
        robot_share,
        'urdf',
        'robot.urdf'
    )

    rviz_file = os.path.join(
        robot_share,
        'rviz',
        'robot.rviz'
    )

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Gazebo Fortress / Gazebo Sim needs resource paths for worlds and models.
    gz_resource_path = os.pathsep.join([
        ntu_share,
        os.path.join(ntu_share, 'worlds'),
        os.path.join(ntu_share, 'models'),
        robot_share,
        os.path.join(robot_share, 'models'),
    ])

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path
    )

    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=gz_resource_path
    )

    # Launch the official coursework maze only.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items()
    )

    # Publish your robot description without changing robot specs.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )

    # Spawn YOUR robot URDF into the official maze.
    spawn_robot = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_person_a_robot',
                output='screen',
                arguments=[
                    '-name', 'person_a_robot',
                    '-file', urdf_file,
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.20',
                    '-Y', '0.0'
                ]
            )
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )

    return LaunchDescription([
        use_sim_time,
        set_gz_resource_path,
        set_ign_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        rviz,
    ])
