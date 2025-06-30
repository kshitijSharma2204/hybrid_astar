#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
import numpy as np
import math
from rclpy.qos import QoSProfile

class EnvPublisher(Node):
    def __init__(self):
        super().__init__('env_publisher')

        qos = QoSProfile(depth=1)
        self.map_pub   = self.create_publisher(OccupancyGrid, '/map',   qos)
        self.start_pub = self.create_publisher(PoseStamped,   '/start', qos)
        self.goal_pub  = self.create_publisher(PoseStamped,   '/goal',  qos)

        # publish once after 1 s, then stop
        self.timer = self.create_timer(1.0, self.publish_environment)

    def quaternion_from_yaw(self, yaw: float) -> Quaternion:
      q = Quaternion()
      q.x = 0.0
      q.y = 0.0
      q.z = math.sin(yaw * 0.5)
      q.w = math.cos(yaw * 0.5)
      return q
    
    def publish_environment(self):
        # build a simple 80×80-cell map, 10 pixels per cell
        cell_size = 10
        cols, rows = 80, 80
        width, height = cols * cell_size, rows * cell_size

        # 1) create free-space (255) / obstacle (0) image
        img = np.ones((height, width), dtype=np.uint8) * 255

        # vertical divider at x = half-width, thickness=4
        x0 = width // 2
        for dx in range(-4, 4):
            img[100:height-100, x0 + dx] = 0

        # horizontal bars spaced every 100 px, thickness=4
        y0 = height // 2
        for offset in range(-300, 301, 100):
            yy = y0 + offset
            for dy in range(-4, 4):
                img[yy + dy, 300:width-300] = 0

        # flip the image so that row 0 is at the bottom
        img = np.flipud(img)

        # 2) fill OccupancyGrid (100=occupied, 0=free)
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = 0.1     # meters per pixel
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = 0.0

        flat = img.flatten()  # now correctly oriented
        grid.data = [100 if v == 0 else 0 for v in flat]

        # 3) publish map
        self.map_pub.publish(grid)

        # 4) publish start & goal in meters
        res = grid.info.resolution

        start = PoseStamped()
        start.header = grid.header
        start.pose.position.x = 35.0 * res * cell_size
        start.pose.position.y = 55.0 * res * cell_size
        start.pose.orientation = self.quaternion_from_yaw(math.radians(0.0))
        self.start_pub.publish(start)

        goal = PoseStamped()
        goal.header = grid.header
        goal.pose.position.x = 50.0 * res * cell_size
        goal.pose.position.y = 17.0 * res * cell_size
        goal.pose.orientation = self.quaternion_from_yaw(math.radians(180.0))
        self.goal_pub.publish(goal)

        self.get_logger().info('Published environment, start, and goal.')
        self.timer.cancel()  # no further publishing

def main(args=None):
    rclpy.init(args=args)
    node = EnvPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
