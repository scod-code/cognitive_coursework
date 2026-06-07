import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch Route A Nav2 on the official atlas robot and saved map."""

    pkg_share = get_package_share_directory('wall_follower')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    robot_description_path = os.path.join(
        pkg_share,
        'urdf',
        'topic2_atlas_rviz.urdf',
    )

    with open(robot_description_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation time',
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value='/work/wall_follower/maps/topic2_nav2_clean_map.yaml',
        description='Clean Route A official Nav2 map yaml',
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'topic2_nav2_params.yaml'),
        description='Route A Nav2 parameters file',
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for Route A Nav2',
    )

    nav2_tf_helper = Node(
        package='wall_follower',
        executable='topic2_nav2_tf_helper',
        name='topic2_nav2_tf_helper',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='topic2_robot_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            },
        ],
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
                'yaml_filename': map_file,
            },
        ],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
        remappings=[('cmd_vel', '/atlas/cmd_vel')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
        remappings=[('cmd_vel', '/atlas/cmd_vel')],
    )

    goal_pose_bridge = Node(
        package='wall_follower',
        executable='topic2_goal_pose_bridge',
        name='topic2_goal_pose_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[params_file],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_topic2_nav2',
        output='screen',
        arguments=[
            '-d',
            os.path.join(pkg_share, 'rviz', 'topic2_nav2.rviz'),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_params_file,
        declare_use_rviz,
        nav2_tf_helper,
        robot_state_publisher,
        map_server,
        planner_server,
        controller_server,
        bt_navigator,
        behavior_server,
        goal_pose_bridge,
        lifecycle_manager,
        rviz,
    ])
