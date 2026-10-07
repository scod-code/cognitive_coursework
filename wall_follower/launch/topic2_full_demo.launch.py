"""
Single-command Topic 2 demo.

Replaces the six-terminal procedure: simulation, Nav2 + RViz, OctoMap,
YOLO, Nav2 traffic rules and the YOLO counter, plus optional cognition
nodes, all started from one ``ros2 launch`` with timed ordering.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start every Topic 2 demo component from one launch file."""
    launch_dir = os.path.join(get_package_share_directory('wall_follower'), 'launch')

    use_rviz = LaunchConfiguration('use_rviz')
    model_path = LaunchConfiguration('model_path')
    nav2_delay = LaunchConfiguration('nav2_delay')
    perception_delay = LaunchConfiguration('perception_delay')
    use_goal_pose_bridge = LaunchConfiguration('use_goal_pose_bridge')
    enable_pomdp = LaunchConfiguration('enable_pomdp')
    pomdp_dry_run = LaunchConfiguration('pomdp_dry_run')
    enable_goal_publisher = LaunchConfiguration('enable_goal_publisher')
    optical_to_body = LaunchConfiguration('optical_to_body')
    enable_curiosity = LaunchConfiguration('enable_curiosity')

    declared_args = [
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz with the Nav2 config'),
        DeclareLaunchArgument(
            'model_path',
            default_value=os.path.expanduser(
                '~/ros2_coursework_ws/results/trafficsignv2/weights/best.pt'),
            description='YOLO weights file passed to yolo_node'),
        DeclareLaunchArgument(
            'nav2_delay', default_value='20.0',
            description='Seconds after launch before Nav2 (and RViz) start; '
                        'keep below perception_delay'),
        DeclareLaunchArgument(
            'perception_delay', default_value='30.0',
            description='Seconds after launch (not after Nav2) before OctoMap, '
                        'YOLO, traffic rules and the counter start; must exceed '
                        'nav2_delay'),
        DeclareLaunchArgument(
            'use_goal_pose_bridge', default_value='true',
            description='Start topic2_goal_pose_bridge (duplicates bt_navigator '
                        '/goal_pose handling)'),
        DeclareLaunchArgument(
            'enable_pomdp', default_value='false',
            description='Start pomdp_goal_selector'),
        DeclareLaunchArgument(
            'pomdp_dry_run', default_value='true',
            description='pomdp_goal_selector dry_run (false dispatches goals)'),
        DeclareLaunchArgument(
            'enable_goal_publisher', default_value='false',
            description='Start goal_publisher (publishes /goal_pose)'),
        DeclareLaunchArgument(
            'optical_to_body', default_value='true',
            description='goal_publisher optical_to_body axis conversion'),
        DeclareLaunchArgument(
            'enable_curiosity', default_value='false',
            description='Start curiosity_explorer (publishes /goal_pose)'),
    ]

    # Step 1: Gazebo world, atlas robot and ros_gz bridge.
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'topic2_official_system.launch.py')),
    )

    # Step 2: Nav2, TF helper and RViz once the sim and bridge are up.
    nav2 = TimerAction(
        period=nav2_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'topic2_nav2.launch.py')),
                launch_arguments={
                    'use_rviz': use_rviz,
                    'use_goal_pose_bridge': use_goal_pose_bridge,
                }.items(),
            ),
        ],
    )

    # Steps 3-6 and optional nodes once Nav2 TF is available.
    octomap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'topic2_octomap_with_nav2.launch.py')),
    )

    yolo_node = Node(
        package='yolo_ros',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
        parameters=[{'model_path': model_path}],
    )

    traffic_rules = Node(
        package='wall_follower',
        executable='topic2_nav2_traffic_rules',
        name='topic2_nav2_traffic_rules',
        output='screen',
    )

    yolo_counter = Node(
        package='wall_follower',
        executable='topic2_yolo_counter',
        name='topic2_yolo_counter',
        output='screen',
    )

    pomdp = Node(
        package='wall_follower',
        executable='pomdp_goal_selector',
        name='pomdp_goal_selector',
        output='screen',
        parameters=[{'dry_run': ParameterValue(pomdp_dry_run, value_type=bool)}],
        condition=IfCondition(enable_pomdp),
    )

    goal_publisher = Node(
        package='yolo_ros',
        executable='goal_publisher',
        name='goal_publisher',
        output='screen',
        parameters=[{
            'optical_to_body': ParameterValue(optical_to_body, value_type=bool),
        }],
        condition=IfCondition(enable_goal_publisher),
    )

    curiosity = Node(
        package='yolo_ros',
        executable='curiosity_explorer',
        name='curiosity_explorer',
        output='screen',
        condition=IfCondition(enable_curiosity),
    )

    perception = TimerAction(
        period=perception_delay,
        actions=[
            octomap,
            yolo_node,
            traffic_rules,
            yolo_counter,
            pomdp,
            goal_publisher,
            curiosity,
        ],
    )

    return LaunchDescription(declared_args + [sim, nav2, perception])
