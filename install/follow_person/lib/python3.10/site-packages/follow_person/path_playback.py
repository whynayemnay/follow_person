#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions
from geometry_msgs.msg import PoseWithCovarianceStamped
import csv
import math


class PathPlayback(Node):
    def __init__(self):
        """
        Initialize the PathPlayback node. Sets up subscriptions and services
        for handling path playback and initial pose setup.
        """
        super().__init__("path_playback")

        self.navigator = TurtleBot4Navigator()
        self.initial_pose = None
        self.initial_pose_set = False

        self.initialpose_subscription = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.initialpose_callback, 10
        )

        self.playback_service = self.create_service(
            Trigger, "/start_playback", self.start_playback_service_callback
        )

    def initialpose_callback(self, msg):
        """
        Callback to handle manual initial pose setup from RViz via /initialpose.

        Args:
            msg (PoseWithCovarianceStamped): The initial pose message received from RViz.
        """
        if not self.initial_pose_set:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            orientation_z = msg.pose.pose.orientation.z
            orientation_w = msg.pose.pose.orientation.w
            theta = 2 * math.atan2(orientation_z, orientation_w)

            initial_pose = self.navigator.getPoseStamped([x, y], theta)
            self.initial_pose_set = True
            self.initial_pose = initial_pose

    def load_and_reverse_waypoints(self, file_path):
        """
        Load waypoints from a CSV file, reverse their order, and adjust orientations.

        Args:
            file_path (str): The path to the CSV file containing waypoints.

        Returns:
            list: A list of reversed and adjusted waypoints [(x, y, theta), ...].
        """
        waypoints = []
        try:
            with open(file_path, "r") as f:
                reader = csv.reader(f)
                for row in reader:
                    x, y, theta = float(row[0]), float(row[1]), float(row[2])
                    waypoints.append((x, y, theta))

            waypoints.reverse()

            adjusted_waypoints = []
            for x, y, theta in waypoints:
                reversed_theta = (theta + math.pi) % (2 * math.pi) - math.pi
                adjusted_waypoints.append((x, y, reversed_theta))

            return adjusted_waypoints
        except Exception:
            return []

    def initialise_pose(self):
        """
        Initialize the robot's pose either from a predefined docked position
        or based on manual input. Clears costmaps and ensures SLAM is active.
        """
        if self.initial_pose_set:
            return

        if not self.navigator.getDockedStatus():
            self.navigator.dock()

        initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self.navigator.clearAllCostmaps()
        self.navigator.setInitialPose(initial_pose)
        if not self.is_slam_active():
            self.navigator.waitUntilNav2Active()
        self.navigator.undock()

        self.initial_pose_set = True
        self.initial_pose = initial_pose

    def is_slam_active(self):
        """
        Check if SLAM is active by verifying the presence of key SLAM topics.

        Returns:
            bool: True if SLAM is active, False otherwise.
        """
        slam_topics = [
            "/slam_toolbox/feedback", "/slam_toolbox/graph_visualization",
            "/map_metadata", "/pose", "/slam_toolbox/scan_visualization",
            "/slam_toolbox/update"
        ]
        active_topics = [info[0] for info in self.get_topic_names_and_types()]

        for topic in slam_topics:
            if topic in active_topics:
                return True

        return False

    def playback_path(self):
        """
        Replay a recorded path using waypoints from a CSV file. The robot follows
        the waypoints in reverse order, navigating through each point sequentially.
        """
        if not self.initial_pose_set:
            return

        self.navigator.cancelTask()
        self.navigator.clearAllCostmaps()

        recorded_waypoints = self.load_and_reverse_waypoints("recorded_path.csv")
        if not recorded_waypoints:
            return

        goal_pose = [
            self.navigator.getPoseStamped([x, y], math.degrees(theta))
            for x, y, theta in recorded_waypoints
        ]

        self.navigator.startToPose(goal_pose[0])

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        self.navigator.startThroughPoses(goal_pose[1:])

    def start_playback_service_callback(self, request, response):
        """
        Service callback to start the path playback.

        Args:
            request (std_srvs.srv.Trigger.Request): The service request.
            response (std_srvs.srv.Trigger.Response): The service response.

        Returns:
            std_srvs.srv.Trigger.Response: Response indicating success or failure.
        """
        if not self.initial_pose_set:
            self.initialise_pose()

        self.playback_path()

        response.success = True
        response.message = "Playback started successfully."
        return response


def main(args=None):
    rclpy.init(args=args)
    path_playback = PathPlayback()

    try:
        path_playback.initialise_pose()
        rclpy.spin(path_playback)
    except KeyboardInterrupt:
        pass
    finally:
        path_playback.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()