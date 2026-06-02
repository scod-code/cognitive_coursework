import csv
import time
import psutil

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String

class ResourceMonitor(Node):
    def __init__(self):
        super().__init__('resource_monitor')
        self.declare_parameter('csv_path', '/home/somto/ros2_coursework_ws/perception/resource_usage.csv')
        self.csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        self.pub = self.create_publisher(String, '/resource_monitor', 10)
        self._file = open(self.csv_path, 'a', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow(['ts','cpu_percent','mem_percent'])
        self.timer = self.create_timer(1.0, self.poll)
        self.get_logger().info(f'Resource monitor logging to {self.csv_path}')

    def poll(self):
        ts = time.time()
        cpu = psutil.cpu_percent(interval=None)
        mem = psutil.virtual_memory().percent
        self._writer.writerow([ts, cpu, mem])
        self._file.flush()
        msg = String()
        msg.data = f'{ts},{cpu},{mem}'
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ResourceMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
