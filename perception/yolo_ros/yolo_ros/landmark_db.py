import os
import sqlite3
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from yolo_msgs.srv import StoreLandmark
from yolo_msgs.msg import Detection

class LandmarkDB(Node):
    def __init__(self):
        super().__init__('landmark_db')
        self.declare_parameter('db_path', os.path.join(
            os.path.expanduser('~'),
            'ros2_coursework_ws', 'perception', 'landmarks.db'
        ))
        self.db_path = self.get_parameter('db_path').get_parameter_value().string_value
        self._conn = sqlite3.connect(self.db_path, check_same_thread=False)
        self._lock = threading.Lock()
        self._create_table()
        self.srv = self.create_service(StoreLandmark, 'store_landmark', self.store_cb)
        self.get_logger().info(f'Landmark DB ready at {self.db_path}')

    def _create_table(self):
        with self._lock:
            cur = self._conn.cursor()
            cur.execute('''CREATE TABLE IF NOT EXISTS landmarks (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                label TEXT,
                x REAL,
                y REAL,
                z REAL,
                confidence REAL,
                ts REAL
            )''')
            self._conn.commit()

    def store_cb(self, request, response):
        ts = time.time()
        with self._lock:
            cur = self._conn.cursor()
            cur.execute('INSERT INTO landmarks (label,x,y,z,confidence,ts) VALUES (?,?,?,?,?,?)',
                        (request.label, request.x, request.y, request.z, request.confidence, ts))
            self._conn.commit()
            response.success = True
            response.message = 'stored'
        self.get_logger().info(f"Stored landmark {request.label} at ({request.x:.2f},{request.y:.2f},{request.z:.2f})")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LandmarkDB()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
