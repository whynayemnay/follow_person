#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Trigger
import csv
import time
from math import atan2
from scipy.spatial.distance import euclidean


class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')

        # Initialize waypoint storage
        self.waypoints = []

        # Subscribe to /amcl_pose to get the robot's position
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        # To record pose every 1 second
        self.last_record_time = time.time()

        # Flag to check if recording is active
        self.recording_active = False  # Initially set to False

        # Start and Stop recording services
        self.start_service = self.create_service(Trigger, '/start_recording', self.start_recording_service)
        self.stop_service = self.create_service(Trigger, '/stop_recording', self.stop_recording_service)

        self.get_logger().info("PathRecorder node initialized. Use services to start and stop recording.")

    def pose_callback(self, msg):
        # Only record the pose if recording is active
        if not self.recording_active:
            return

        # Get the current time
        current_time = time.time()

        # Only record the pose every 1 second
        if current_time - self.last_record_time >= 0.5:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            orientation_z = msg.pose.pose.orientation.z
            orientation_w = msg.pose.pose.orientation.w

            # Calculate the theta (yaw angle) from the quaternion
            theta = 2 * atan2(orientation_z, orientation_w)

            # Store the absolute pose (x, y, theta)
            waypoint = (round(x, 2), round(y, 2), round(theta, 2))
            self.waypoints.append(waypoint)
            self.get_logger().info(f"Recorded absolute waypoint: {waypoint}")

            # Update the last record time
            self.last_record_time = current_time

    def start_recording(self):
        """Start recording waypoints."""
        self.recording_active = True
        self.get_logger().info("Recording started.")

    def stop_recording(self):
        """Stop recording waypoints and save them to a file."""
        self.recording_active = False
        self.simplify_waypoints()  # Simplify waypoints using RDP before saving
        self.save_waypoints()
        self.get_logger().info("Recording stopped.")

    def simplify_waypoints(self, epsilon=0.1):
        """Simplify the waypoints using the Ramer-Douglas-Peucker algorithm."""
        def rdp(points, epsilon):
            if len(points) < 3:
                return points

            # Find the point farthest from the line between start and end points
            start, end = points[0], points[-1]
            max_dist = 0
            index = 0

            for i, point in enumerate(points[1:-1], start=1):
                # Compute the perpendicular distance from the point to the line
                num = abs((end[1] - start[1]) * point[0] - (end[0] - start[0]) * point[1] + end[0] * start[1] - end[1] * start[0])
                denom = euclidean(start, end)
                dist = num / denom if denom != 0 else 0

                if dist > max_dist:
                    max_dist = dist
                    index = i

            # If the maximum distance is greater than epsilon, recursively simplify
            if max_dist > epsilon:
                left = rdp(points[:index + 1], epsilon)
                right = rdp(points[index:], epsilon)
                return left[:-1] + right

            # Otherwise, return a straight line approximation
            return [start, end]

        self.get_logger().info(f"Simplifying waypoints using RDP with epsilon={epsilon}...")
        simplified_path = rdp(self.waypoints, epsilon)
        self.get_logger().info(f"Waypoints simplified from {len(self.waypoints)} to {len(simplified_path)} points.")
        self.waypoints = simplified_path

    def save_waypoints(self):
        # Save the recorded waypoints to a CSV file
        file_path = 'recorded_path.csv'
        try:
            with open(file_path, 'w') as f:
                writer = csv.writer(f)
                writer.writerows(self.waypoints)
            self.get_logger().info(f"Waypoints saved to {file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save waypoints: {e}")
            return

        # Clear the waypoints after saving
        self.waypoints.clear()
        self.get_logger().info("Waypoints cleared after saving.")

    def start_recording_service(self, request, response):
        """Service callback to start recording."""
        if self.recording_active:
            response.success = False
            response.message = "Recording is already active."
        else:
            self.start_recording()
            response.success = True
            response.message = "Recording started."
        return response

    def stop_recording_service(self, request, response):
        """Service callback to stop recording."""
        if not self.recording_active:
            response.success = False
            response.message = "Recording is not active."
        else:
            self.stop_recording()
            response.success = True
            response.message = "Recording stopped and waypoints saved."
        return response


def main(args=None):
    rclpy.init(args=args)
    path_recorder = PathRecorder()

    try:
        # Spin the node to process callbacks and keep it running
        rclpy.spin(path_recorder)
    except KeyboardInterrupt:
        print("Interrupted by user, shutting down node.")
    finally:
        if rclpy.ok():
            path_recorder.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()