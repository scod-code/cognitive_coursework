import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Start the official Route A Topic 2 simulation stack only."""

    ntu_share = get_package_share_directory('ntu_robotsim')

    cwmaze_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ntu_share, 'launch', 'cwmaze.launch.py')
        ),
    )

    spawn_robot = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ntu_share, 'launch', 'spawn_robot.launch.py')
                ),
                launch_arguments={
                    'robot_name': 'atlas',
                    'sdf_file': 'jetbot/model.sdf',
                    'world': 'cwmaze',
                    'x': '-3.0',
                    'y': '-3.0',
                    'z': '0.2',
                    'roll': '0.0',
                    'pitch': '0.0',
                    'yaw': '0.0',
                    'use_imu': 'true',
                }.items(),
            )
        ],
    )

    bridge = TimerAction(
        period=11.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                    '/atlas/odom_ground_truth@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                    '/atlas/rgbd_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                    '/atlas/rgbd_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
                    '/atlas/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                    '/atlas/rgbd_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
                    '/atlas/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                    '/model/atlas/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                    '/model/atlas/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                    '/atlas/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                ],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        cwmaze_launch,
        spawn_robot,
        bridge,
    ])
