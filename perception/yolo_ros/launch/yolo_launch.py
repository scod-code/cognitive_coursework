from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    """Person B perception/cognition stack.

    Runs alongside the official Topic 2 stack (topic2_official_system +
    topic2_nav2). wall_follower_node needs a LaserScan on /scan, which the
    official atlas bridge does not provide; start
    `ros2 launch wall_follower topic2_official_adapters.launch.py` first to
    derive /scan from the RGB-D point cloud. sign_controller publishes to
    /atlas/cmd_vel and will compete with Nav2 for the robot if both are
    driving at once -- use one or the other for a given demo.
    """
    # Resolve model path relative to home directory (portable across machines)
    model_path = os.path.join(
        os.path.expanduser('~'),
        'ros2_coursework_ws', 'results', 'trafficsignv2', 'weights', 'best.pt'
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
        # -> publishes modified speed to /atlas/cmd_vel
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
