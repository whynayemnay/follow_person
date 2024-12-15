#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.topic_endpoint_info import TopicEndpointInfo
from std_srvs.srv import Trigger
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist 
import csv
from math import atan2, degrees, radians
import math
import time


class PathPlayback(Node):
    def __init__(self):
        super().__init__("path_playback")

        # Initialize navigator and other variables
        self.navigator = TurtleBot4Navigator()
        self.initial_pose = None
        self.initial_pose_set = False

        # Subscribe to /initialpose to detect manual pose setting in RViz
        self.initialpose_subscription = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose",
            self.initialpose_callback,
            10
        )

        # Create a service to trigger playback
        self.playback_service = self.create_service(
            Trigger, "/start_playback", self.start_playback_service_callback
        )

        self.get_logger().info("PathPlayback node initialized. Waiting for commands.")

    def initialpose_callback(self, msg):
        if not self.initial_pose_set:
            # Log the pose recieved from /initial pose function
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            orientation_z = msg.pose.pose.orientation.z
            orientation_w = msg.pose.pose.orientation.w
            theta = 2 * atan2(orientation_z, orientation_w)

            self.get_logger().info(
                f"Manual pose set: Position(x={x}, y={y}) "
                f"Orientation(z={orientation_z}, w={orientation_w})"
                f"Angle(theta={theta})"
            )

            initial_pose = self.navigator.getPoseStamped([x,y], theta)
            # Update the initial pose in the navigator
            # self.navigator.setInitialPose(initial_pose)
            self.initial_pose_set = True
            self.initial_pose = initial_pose
            self.get_logger().info("Initial pose updated from /initialpose topic.")

    def load_and_reverse_waypoints(self, file_path):
        """Load waypoints from a CSV file, reverse them, and adjust orientation for playback."""
        waypoints = []
        try:
            with open(file_path, "r") as f:
                reader = csv.reader(f)
                for row in reader:
                    x, y, theta = float(row[0]), float(row[1]), float(row[2])
                    waypoints.append((x, y, theta))

            # Reverse the waypoints to follow the path back to the origin
            waypoints.reverse()

            # Adjust orientation for reversed waypoints
            adjusted_waypoints = []
            for x, y, theta in waypoints:
                reversed_theta = theta + math.pi
                reversed_theta = (reversed_theta + math.pi) % (2 * math.pi) - math.pi
                adjusted_waypoints.append((x, y, reversed_theta))

            self.get_logger().info(f"Loaded and reversed {len(adjusted_waypoints)} waypoints from {file_path}.")
            return adjusted_waypoints

        except FileNotFoundError:
            self.get_logger().error(f"Waypoint file {file_path} not found.")
            return []
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")
            return []

    def initialise_pose(self):
        """Check if the initial pose was set manually otherwise set it programatically,
        But The pose is set according to the Simulation dock position, for real robot
        for real scenario use other position mainly the theta angle"""
        if self.initial_pose_set:
            self.get_logger().info("initial pose is already set. Skipping initialization.")
            return

        self.get_logger().info("Initializing pose manually according to the dock")

        if not self.navigator.getDockedStatus():
            self.navigator.info('Docking before initialising pose')
            self.navigator.dock()

        initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self.navigator.clearAllCostmaps()
        self.navigator.setInitialPose(initial_pose)
        if not self.is_slam_active():
            self.navigator.waitUntilNav2Active()
        self.get_logger().info("Initial pose set and ready for playback.")
        self.navigator.undock()

        self.initial_pose_set = True
        self.initial_pose = initial_pose

    def is_slam_active(self):
        """Check if SLAM topics are running."""
        slam_topics = ["/slam_toolbox/feedback", "/slam_toolbox/graph_visualization",
                       "/map_metadata", "/pose", "/slam_toolbox/scan_visualization",
                       "/slam_toolbox/update"
        ]
        active_topics = [info[0] for info in self.get_topic_names_and_types()]

        # Check if any required SLAM topic is in the list of active topics
        for topic in slam_topics:
            if topic in active_topics:
                self.get_logger().info(f"SLAM topic detected: {topic}")
                return True

        self.get_logger().warn("No SLAM topics detected. SLAM might not be running.")
        return False
        
    def playback_path(self):
        """Handle playback of the path."""
        if not self.initial_pose_set:
            self.get_logger().error("Cannot start playback. Initial pose is not set.")
            return
        self.get_logger().info("Starting playback...")

        self.navigator.cancelTask()
        self.navigator.clearAllCostmaps()

        recorded_waypoints = self.load_and_reverse_waypoints("recorded_path.csv")
        if not recorded_waypoints:
            self.get_logger().error("No waypoints available for playback.")
            return

        goal_pose = [
            self.navigator.getPoseStamped([x, y], degrees(theta))
            for x, y, theta in recorded_waypoints
        ]

        self.get_logger().info("Moving to the first waypoint.")
        self.navigator.startToPose(goal_pose[0])

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Reached the first waypoint. Starting full path navigation.")
        self.navigator.startThroughPoses(goal_pose[1:])
        # self.navigator.followWaypoints(goal_pose[1:])
        # self.get_logger().info("Smooth navigation through waypoints complete.")


    def start_playback_service_callback(self, request, response):
        """Service callback to start playback."""
        if not self.initial_pose_set:
            self.get_logger().warn("Initial pose is not set. Initializing pose at the dock.")
            self.initialise_pose()  # Programmatically initialize the pose

        # Proceed with playback after ensuring the pose is set
        self.get_logger().info("Playback service called. Starting playback.")
        self.playback_path()

        response.success = True
        response.message = "Playback started successfully."
        return response


def main(args=None):
    rclpy.init(args=args)
    path_playback = PathPlayback()

    try:
        # Wait for the initial pose to be set
        path_playback.initialise_pose()

        # Call the playback method after initialization
        # path_playback.playback_path()

        # Keep the node running to listen for playback commands
        rclpy.spin(path_playback)
    except KeyboardInterrupt:
        print("Path playback interrupted by user.")
    finally:
        path_playback.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
