#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from ultralytics import YOLO
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException
import cv2
from collections import deque


class TemporalSmoothing:
    """
    Temporal smoothing for position averaging.
    """
    def __init__(self, buffer_size=10):
        self.buffer_size = buffer_size
        self.position_buffer = deque(maxlen=buffer_size)

    def add_position(self, position):
        self.position_buffer.append(position)

    def get_smoothed_position(self):
        if not self.position_buffer:
            return None
        return np.mean(self.position_buffer, axis=0)


class PersonDetectionLidarNode(Node):
    def __init__(self):
        """
        Initialize the node for person detection using RGB images and LIDAR data.
        """
        super().__init__('person_detection_lidar_node')

        self.model = YOLO('yolov8n-pose.pt')

        self.rgb_sub = self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.image_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan/front', self.lidar_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_published_position = None
        self.latest_person_position = None

        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_update', 10)

        self.goal_timer = self.create_timer(1.0, self.check_and_publish_goal)

        self.bridge = CvBridge()

        self.fx = 277.0
        self.fy = 277.0
        self.cx = 160.0
        self.cy = 120.0
        self.image_width = 320
        self.image_height = 240

        self.latest_lidar_ranges = None
        self.latest_lidar_angle_min = None
        self.latest_lidar_angle_increment = None

        self.temporal_smoothing = TemporalSmoothing(buffer_size=10)

    def image_callback(self, rgb_msg):
        """
        Process the incoming RGB image to detect persons, calculate their positions,
        and compute a smoothed map position for publishing.

        Args:
            rgb_msg (sensor_msgs.msg.Image): The received RGB image message.
        """
        cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        results = self.model.predict(cv_image, verbose=False, conf=0.6)
        if not results or len(results[0].keypoints) == 0:
            return

        keypoints = results[0].keypoints.data[0].cpu().numpy()
        ankles, ankle_confidences = self.get_ankles(keypoints)

        if not self.latest_lidar_ranges or ankles is None or len(ankles) == 0:
            return

        ankles_3d, confidences = self.calculate_3d_coordinates(ankles, ankle_confidences)

        if not ankles_3d:
            return

        center_ankle = np.mean(ankles_3d, axis=0)
        self.temporal_smoothing.add_position(center_ankle)
        smoothed_position = self.temporal_smoothing.get_smoothed_position()

        if smoothed_position is not None:
            person_map_position = self.compute_person_map_position_direct(smoothed_position)
            if person_map_position is not None:
                self.latest_person_position = person_map_position

    def lidar_callback(self, scan_msg):
        """
        Store the latest LIDAR data for use in calculating 3D positions.

        Args:
            scan_msg (sensor_msgs.msg.LaserScan): The received LIDAR scan message.
        """
        self.latest_lidar_ranges = scan_msg.ranges
        self.latest_lidar_angle_min = scan_msg.angle_min
        self.latest_lidar_angle_increment = scan_msg.angle_increment

    def get_ankles(self, keypoints, confidence_threshold=0.5):
        """
        Extract ankle keypoints from YOLOv8 detections and filter them by confidence.

        Args:
            keypoints (numpy array): Keypoints detected by YOLOv8.
            confidence_threshold (float): Minimum confidence to consider valid ankles.

        Returns:
            tuple: A list of ankle positions and their confidences.
        """
        if keypoints.shape[0] < 17:
            return [], []

        ankles = keypoints[[15, 16], :2]
        ankle_confidences = keypoints[[15, 16], 2]

        if any(conf < confidence_threshold for conf in ankle_confidences):
            return [], []

        return ankles, ankle_confidences

    def calculate_3d_coordinates(self, ankles, ankles_confidences):
        """
        Calculate 3D coordinates for detected hips using LIDAR data.

        Args:
            hips (list): 2D positions of detected hips in the image.
            hip_confidences (list): Confidence scores for the detected hips.

        Returns:
            tuple: A list of 3D coordinates for the hips and their confidence scores.
        """
        ankles_3d = []
        valid_confidences = []

        horizontal_fov = 2 * np.arctan((self.image_width / 2) / self.fx)
        lidar_rotation_offset = -np.pi / 2
        min_range = 0.2
        max_range = 10.0

        for i, (x, y) in enumerate(ankles):
            confidence = ankles_confidences[i]
            if confidence < 0.5:
                continue

            ankle_angle = ((x - self.cx) / self.image_width) * horizontal_fov
            lidar_angle = ankle_angle + lidar_rotation_offset
            lidar_angle_adjusted = lidar_angle
            lidar_index = int((lidar_angle_adjusted - self.latest_lidar_angle_min) / self.latest_lidar_angle_increment)
            lidar_index = (len(self.latest_lidar_ranges) - 1) - lidar_index
            lidar_index = np.clip(lidar_index, 0, len(self.latest_lidar_ranges) - 1)

            lidar_range_indices = range(max(0, lidar_index - 5), min(len(self.latest_lidar_ranges), lidar_index + 7))
            distances = [
                self.latest_lidar_ranges[idx]
                for idx in lidar_range_indices
                if np.isfinite(self.latest_lidar_ranges[idx]) and min_range <= self.latest_lidar_ranges[idx] <= max_range
            ]

            if not distances:
                continue

            smallest_distance = min(distances)

            x_world = (x - self.cx) * smallest_distance / self.fx
            y_world = (y - self.cy) * smallest_distance / self.fy
            z_world = smallest_distance

            ankles_3d.append((x_world, y_world, z_world))
            valid_confidences.append(confidence)

        return ankles_3d, valid_confidences

    def compute_person_map_position_direct(self, smoothed_position):
        """
        Compute the person's position in the map frame using the robot's position
        and orientation from the TF tree.

        Args:
            smoothed_position (numpy array): Smoothed 3D position of the person.

        Returns:
            tuple or None: Map frame coordinates of the person or None if transformation fails.
        """
        try:
            base_to_map_transform = self.tf_buffer.lookup_transform(
                "map", "oakd_rgb_camera_optical_frame", rclpy.time.Time()
            )

            robot_position_map = base_to_map_transform.transform.translation
            robot_orientation = base_to_map_transform.transform.rotation

            yaw = self.get_yaw_from_quaternion(robot_orientation)

            offset_x = -smoothed_position[0]
            offset_z = smoothed_position[2] - 1

            robot_x = robot_position_map.x
            robot_y = robot_position_map.y

            person_x = robot_x + offset_z * np.cos(yaw) - offset_x * np.sin(yaw)
            person_y = robot_y + offset_z * np.sin(yaw) + offset_x * np.cos(yaw)

            return person_x, person_y

        except TransformException:
            return None

    def check_and_publish_goal(self):
        """
        Publish the person's position as a navigation goal if it differs
        from the last published position.
        """
        if self.latest_person_position is not None:
            if self.last_published_position != self.latest_person_position:
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "map"
                pose_msg.pose.position.x = self.latest_person_position[0]
                pose_msg.pose.position.y = self.latest_person_position[1]
                pose_msg.pose.position.z = 0.0
                pose_msg.pose.orientation.w = 1.0
                self.goal_publisher.publish(pose_msg)
                self.last_published_position = self.latest_person_position

    def get_yaw_from_quaternion(self, quaternion):
        """
        Extract the yaw (rotation around the Z-axis) from a quaternion.

        Args:
            quaternion: Quaternion representing orientation.

        Returns:
            float: Yaw angle in radians.
        """
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectionLidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()