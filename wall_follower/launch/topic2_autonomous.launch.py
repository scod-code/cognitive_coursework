"""
Topic 2 autonomous survey: one command, no RViz clicks.

Includes ``topic2_full_demo.launch.py`` with every other goal source disabled and
starts ``autonomous_mission`` as the ONLY goal owner.  ``sign_controller`` and
``wall_follower_node`` are not launched.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _warn_delays(context):
    """Warn (not fail) if the mission starts before perception does."""
    try:
        mission = float(context.perform_substitution(LaunchConfiguration('mission_delay')))
        perception = float(context.perform_substitution(LaunchConfiguration('perception_delay')))
    except ValueError:
        return []
    if mission <= perception:
        return [LogInfo(msg='WARNING: mission_delay <= perception_delay; the mission node '
                            'will log readiness warnings until perception is up.')]
    return []


def generate_launch_description():
    """Start the full demo (no goal sources) plus the autonomous mission node."""
    share = get_package_share_directory('wall_follower')
    launch_dir = os.path.join(share, 'launch')

    args = [
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='Start RViz (do not click goals while autonomous)'),
        DeclareLaunchArgument(
            'model_path',
            default_value=os.path.expanduser(
                '~/ros2_coursework_ws/results/trafficsignv2/weights/best.pt'),
            description='YOLO weights file passed to yolo_node'),
        DeclareLaunchArgument('nav2_delay', default_value='20.0'),
        DeclareLaunchArgument('perception_delay', default_value='30.0'),
        DeclareLaunchArgument('mission_delay', default_value='45.0',
                              description='Seconds after launch before the mission node starts'),
        DeclareLaunchArgument(
            'mission_params',
            default_value=os.path.join(share, 'config', 'autonomous_mission.yaml')),
        DeclareLaunchArgument(
            'mission_log_dir',
            default_value=os.path.expanduser('~/ros2_coursework_ws/mission_logs')),
    ]

    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'topic2_full_demo.launch.py')),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'model_path': LaunchConfiguration('model_path'),
            'nav2_delay': LaunchConfiguration('nav2_delay'),
            'perception_delay': LaunchConfiguration('perception_delay'),
            'use_goal_pose_bridge': 'false',
            'enable_pomdp': 'false',
            'pomdp_dry_run': 'true',
            'enable_goal_publisher': 'false',
            'enable_curiosity': 'false',
        }.items(),
    )

    mission = TimerAction(
        period=LaunchConfiguration('mission_delay'),
        actions=[Node(
            package='wall_follower',
            executable='autonomous_mission',
            name='autonomous_mission',
            output='screen',
            parameters=[
                LaunchConfiguration('mission_params'),
                {'use_sim_time': True,
                 'log_dir': LaunchConfiguration('mission_log_dir')},
            ],
        )],
    )

    return LaunchDescription(args + [OpaqueFunction(function=_warn_delays), demo, mission])
