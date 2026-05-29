import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('wall_follower')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation time'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Nav2 parameters file'
    )

    declare_map_file = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_share, 'maps', 'maze_map.yaml'),
        description='Static map file for AMCL'
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'yaml_filename': map_file
            }
        ]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file]
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[params_file]
    )

    amcl_initializer = Node(
        package='wall_follower',
        executable='amcl_initializer',
        name='amcl_initializer',
        output='screen',
        parameters=[
            {'initial_pose_x': 0.0},
            {'initial_pose_y': 0.0},
            {'initial_pose_yaw': 0.0},
            {'pose_covariance_linear': 0.25},
            {'pose_covariance_angular': 0.785},
            {'max_wait_time': 10.0}
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_map_file,
        map_server,
        amcl,
        amcl_initializer,
        planner_server,
        controller_server,
        bt_navigator,
        behavior_server,
        lifecycle_manager,
    ])
