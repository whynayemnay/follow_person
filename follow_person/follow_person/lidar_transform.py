#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarToCameraFOV(Node):
    def __init__(self):
        super().__init__('lidar_to_camera_fov')

        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10
        )
        self.lidar_pub = self.create_publisher(LaserScan, '/scan/front', 10)

        self.f_x = 277.0
        self.f_y = 277.0
        self.image_width = 320
        self.image_height = 240

        self.horizontal_fov = 2 * np.arctan(self.image_width / (2 * self.f_x))
        self.vertical_fov = 2 * np.arctan(self.image_height / (2 * self.f_y))
        self.lidar_alignment_offset = -np.pi / 2

    def lidar_callback(self, msg):
        total_ranges = len(msg.ranges)
        center_angle = self.lidar_alignment_offset
        half_horizontal_fov = self.horizontal_fov / 2.0

        left_angle = center_angle - half_horizontal_fov
        right_angle = center_angle + half_horizontal_fov

        left_index = int((left_angle - msg.angle_min) / msg.angle_increment) % total_ranges
        right_index = int((right_angle - msg.angle_min) / msg.angle_increment) % total_ranges

        if left_index < right_index:
            front_ranges = msg.ranges[left_index:right_index]
        else:
            front_ranges = msg.ranges[left_index:] + msg.ranges[:right_index]

        front_scan = LaserScan()
        front_scan.header = msg.header
        front_scan.angle_min = msg.angle_min + left_index * msg.angle_increment
        front_scan.angle_max = msg.angle_min + right_index * msg.angle_increment
        front_scan.angle_increment = msg.angle_increment
        front_scan.time_increment = msg.time_increment
        front_scan.scan_time = msg.scan_time
        front_scan.range_min = msg.range_min
        front_scan.range_max = msg.range_max
        front_scan.ranges = front_ranges

        self.lidar_pub.publish(front_scan)


def main(args=None):
    rclpy.init(args=args)
    node = LidarToCameraFOV()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()