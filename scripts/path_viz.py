#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import math
import os
from matplotlib.transforms import Affine2D

class PathViz(Node):
    def __init__(self):
        super().__init__('path_viz')

        # ROS subscriptions
        self.map_sub  = self.create_subscription(
            OccupancyGrid, '/map',  self.map_cb,  10)
        self.path_sub = self.create_subscription(
            Path,          '/planned_path', self.path_cb, 10)

        # internal storage
        self.grid = None
        self.res  = 1.0   # meters per pixel

        # set up matplotlib once
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(12, 9), dpi=100)
        self.ax.set_axis_off()

        self.im   = None   # image artist for the occupancy grid
        self.line = None   # Line2D for the path
        self.rect = None   # Rectangle patch for the vehicle

    def map_cb(self, msg: OccupancyGrid):
        # unpack the grid into a 2D array
        w, h      = msg.info.width, msg.info.height
        self.res  = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        data_flat = np.array(msg.data, dtype=np.int8)
        grid_2d   = data_flat.reshape((h, w))
        self.grid = grid_2d

        # draw the grid (in meters) on first reception
        xmin, xmax = 0, w * self.res
        ymin, ymax = 0, h * self.res
        if self.im is None:
            self.im = self.ax.imshow(
                self.grid,
                cmap='gray',
                origin='lower',
                extent=(xmin, xmax, ymin, ymax),
                aspect='equal'
            )
            self.ax.set_xlim(xmin, xmax)
            self.ax.set_ylim(ymin, ymax)
        else:
            self.im.set_data(self.grid)
            self.im.set_extent((xmin, xmax, ymin, ymax))

        self.get_logger().info(f'Map received: {w}×{h}')
        self.fig.canvas.draw_idle()

    def path_cb(self, msg: Path):
        if self.grid is None:
            return

        # grab the map‐origin so we can shift into image frame
        ox = self.map_origin_x
        oy = self.map_origin_y

        # extract poses (now in world‐meters)
        poses = []
        for p in msg.poses:
            # world coords in meters from the start
            x = p.pose.position.x + ox
            y = p.pose.position.y + oy
            yaw = 2 * math.atan2(p.pose.orientation.z,
                                 p.pose.orientation.w)
            poses.append((x, y, yaw))

        xs = [pt[0] for pt in poses]
        ys = [pt[1] for pt in poses]

        # update or create the red path
        if self.line is None:
            self.line, = self.ax.plot(xs, ys, 'r-', linewidth=2)
        else:
            self.line.set_data(xs, ys)

        # car half‐dims in meters
        L_m, W_m = 2.5, 1.0

        for i, (cx, cy, th) in enumerate(poses):
            # remove old patch
            if self.rect is not None:
                self.rect.remove()

            # 1) create rectangle centered at the origin
            rect = Rectangle(
                (-L_m, -W_m),    # lower-left corner *relative* to center
                2*L_m,           # full length
                2*W_m,           # full width
                fill=True,
                color='orange',
                alpha=0.6,
                linewidth=0
            )

            # 2) build transform: rotate about (0,0) then translate to (cx,cy)
            transf = Affine2D() \
                        .rotate(th) \
                        .translate(cx, cy) \
                     + self.ax.transData

            rect.set_transform(transf)

            # 3) add to axes and redraw
            self.ax.add_patch(rect)
            self.rect = rect
            self.fig.canvas.draw_idle()
            plt.pause(0.05)

def main(args=None):
    rclpy.init(args=args)
    node = PathViz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
