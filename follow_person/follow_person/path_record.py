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
        """
        Initialize the PathRecorder node. Sets up subscriptions and services for
        recording and managing waypoints.
        """
        super().__init__('path_recorder')

        self.waypoints = []
        self.last_record_time = time.time()
        self.recording_active = False

        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        self.start_service = self.create_service(Trigger, '/start_recording', self.start_recording_service)
        self.stop_service = self.create_service(Trigger, '/stop_recording', self.stop_recording_service)

    def pose_callback(self, msg):
        """
        Callback for receiving the robot's current pose. Records the pose as a waypoint
        if recording is active and the minimum time interval has elapsed.

        Args:
            msg (PoseWithCovarianceStamped): The current pose of the robot.
        """
        if not self.recording_active:
            return

        current_time = time.time()
        if current_time - self.last_record_time >= 0.5:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            orientation_z = msg.pose.pose.orientation.z
            orientation_w = msg.pose.pose.orientation.w
            theta = 2 * atan2(orientation_z, orientation_w)

            waypoint = (round(x, 2), round(y, 2), round(theta, 2))
            self.waypoints.append(waypoint)
            self.last_record_time = current_time

    def start_recording(self):
        """
        Start recording waypoints by activating the recording flag.
        """
        self.recording_active = True

    def stop_recording(self):
        """
        Stop recording waypoints. Simplifies the recorded waypoints and saves them to a file.
        """
        self.recording_active = False
        self.simplify_waypoints()
        self.save_waypoints()

    def simplify_waypoints(self, epsilon=0.1):
        """
        Simplify the recorded waypoints using the Ramer-Douglas-Peucker (RDP) algorithm.

        Args:
            epsilon (float): The maximum distance from the simplified path. 
                             Smaller values retain more waypoints.
        """
        def rdp(points, epsilon):
            if len(points) < 3:
                return points

            start, end = points[0], points[-1]
            max_dist, index = 0, 0

            for i, point in enumerate(points[1:-1], start=1):
                num = abs((end[1] - start[1]) * point[0] - (end[0] - start[0]) * point[1] + end[0] * start[1] - end[1] * start[0])
                denom = euclidean(start, end)
                dist = num / denom if denom != 0 else 0

                if dist > max_dist:
                    max_dist, index = dist, i

            if max_dist > epsilon:
                left = rdp(points[:index + 1], epsilon)
                right = rdp(points[index:], epsilon)
                return left[:-1] + right

            return [start, end]

        simplified_path = rdp(self.waypoints, epsilon)
        self.waypoints = simplified_path

    def save_waypoints(self):
        """
        Save the recorded waypoints to a CSV file and clear the internal waypoint list.
        """
        file_path = 'recorded_path.csv'
        try:
            with open(file_path, 'w') as f:
                writer = csv.writer(f)
                writer.writerows(self.waypoints)
        except Exception as e:
            self.get_logger().error(f"Failed to save waypoints: {e}")
        self.waypoints.clear()

    def start_recording_service(self, request, response):
        """
        Service callback to start recording waypoints.

        Args:
            request (std_srvs.srv.Trigger.Request): The service request.
            response (std_srvs.srv.Trigger.Response): The service response.

        Returns:
            std_srvs.srv.Trigger.Response: Response indicating the success or failure of the operation.
        """
        if self.recording_active:
            response.success = False
            response.message = "Recording is already active."
        else:
            self.start_recording()
            response.success = True
            response.message = "Recording started."
        return response

    def stop_recording_service(self, request, response):
        """
        Service callback to stop recording waypoints.

        Args:
            request (std_srvs.srv.Trigger.Request): The service request.
            response (std_srvs.srv.Trigger.Response): The service response.

        Returns:
            std_srvs.srv.Trigger.Response: Response indicating the success or failure of the operation.
        """
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
        rclpy.spin(path_recorder)
    except KeyboardInterrupt:
        pass
    finally:
        path_recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()