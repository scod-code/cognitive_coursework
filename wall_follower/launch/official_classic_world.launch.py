#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # Paths
    wall_follower_dir = get_package_share_directory('wall_follower')
    simple_robot_description_dir = get_package_share_directory('simple_robot_description')
    
    # Official maze geometry (converted to Gazebo Classic format)
    world_sdf = os.path.join(wall_follower_dir, 'models', 'cwmaze_classic.sdf')
    
    # Your robot URDF
    robot_urdf = os.path.join(simple_robot_description_dir, 'urdf', 'robot.urdf')
    
    # RViz config
    rviz_config = os.path.join(wall_follower_dir, 'rviz', 'nav2_default_view.rviz')
    
    # Read robot URDF
    with open(robot_urdf, 'r') as f:
        robot_description = f.read()
    
    # Gazebo resource paths
    gazebo_resource_path = os.path.expanduser('~/.gazebo/models')
    gazebo_model_path = os.path.join(os.environ.get('HOME', '/root'), '.gazebo', 'models')
    
    # Launch Gazebo Classic with factory plugin
    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_sdf,
            '--verbose'
        ],
        output='screen',
        additional_env={
            'GAZEBO_RESOURCE_PATH': gazebo_resource_path,
            'GAZEBO_MODEL_PATH': gazebo_model_path,
            'HOME': os.environ.get('HOME', '/root')
        }
    )
    
    # Spawn the robot (after gazebo_server starts, with delay)
    spawn_entity = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'sleep 3 && ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity person_a_robot -x 0.0 -y 0.0 -z 0.0'
        ],
        output='screen'
    )
    
    # robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ]
    )
    
    # Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            {'use_sim_time': True}
        ]
    )
    
    
    ld = LaunchDescription()
    
    ld.add_action(gazebo_server)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(spawn_entity)
    ld.add_action(rviz_node)
    
    return ld
