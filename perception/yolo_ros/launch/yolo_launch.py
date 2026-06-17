from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Resolve the final report/demo model path relative to home directory.
    model_path = os.path.join(
        os.path.expanduser('~'),
        'ros2_coursework_ws', 'results', 'trafficsignv3', 'weights', 'best.pt'
    )

    return LaunchDescription([
        # Wall follower: publishes navigation commands to /wall_follower/cmd_vel
        Node(
            package='wall_follower',
            executable='wall_follower_node',
            name='wall_follower',
            output='screen',
        ),
        # YOLO detector: publishes detections to /yolo/detections_json
        Node(
            package='yolo_ros',
            executable='yolo_node',
            name='yolo_node',
            output='screen',
            parameters=[{'model_path': model_path}]
        ),
        # Sign controller: reads /wall_follower/cmd_vel + /yolo/detections_json
        # -> publishes modified speed to /cmd_vel
        Node(
            package='yolo_ros',
            executable='sign_controller',
            name='sign_controller',
            output='screen'
        ),
        Node(
            package='yolo_ros',
            executable='goal_publisher',
            name='goal_publisher',
            output='screen'
        ),
        # Landmark database: stores detected object positions via StoreLandmark service
        Node(
            package='yolo_ros',
            executable='landmark_db',
            name='landmark_db',
            output='screen'
        ),
        # Resource monitor: logs CPU/memory usage to CSV for report figures
        Node(
            package='yolo_ros',
            executable='resource_monitor',
            name='resource_monitor',
            output='screen'
        ),
    ])
