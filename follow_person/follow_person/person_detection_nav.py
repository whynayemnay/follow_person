#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
import message_filters
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import PoseStamped


class PersonDetectionNode(Node):
    def __init__(self):
        """
        Initialize the person detection node with YOLOv8 model for pose detection
        and synchronized RGB and depth image inputs.
        """
        super().__init__('person_detection_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.model = YOLO('yolov8n-pose.pt')

        rgb_sub = message_filters.Subscriber(self, Image, '/oakd/rgb/preview/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/oakd/rgb/preview/depth')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self.combined_callback)

        self.bridge = CvBridge()

        self.last_published_position = None
        self.latest_person_position = None

        self.fx = 277.0
        self.fy = 277.0
        self.cx = 160.0
        self.cy = 120.0
        self.image_width = 320
        self.image_height = 240

        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_update', 10)
        self.goal_timer = self.create_timer(1.0, self.check_and_publish_goal)

    def combined_callback(self, rgb_msg, depth_msg):
        """
        Process synchronized RGB and depth messages, perform YOLOv8 pose detection,
        and compute 3D coordinates for detected persons.
        """
        cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')

        results = self.model.predict(cv_image, verbose=False, conf=0.6)
        if not results or not results[0].keypoints or results[0].keypoints.data is None:
            return
        # Visualize camera if needed
        # self.visualize(cv_image, depth_image, results)
        # self.visualize_depth_overlay(cv_image, depth_image)
        keypoints = results[0].keypoints.data[0].cpu().numpy()
        hips, hip_confidences = self.get_hips(keypoints)
        if hips is None or len(hips) == 0:
            return

        hips_3d, confidences = self.calculate_3d_coordinates(hips, hip_confidences, depth_image)
        if not hips_3d:
            return

        person_map_position = self.compute_person_map_position_direct(hips_3d)
        if person_map_position:
            self.latest_person_position = person_map_position

    def visualize(self, rgb_image, depth_image, results=None):
        """
        Visualize the annotated RGB image and normalized depth image.
        """
        annotated_image = results[0].plot() if results else rgb_image

        min_depth, max_depth = 0.0, 35.0
        clipped_depth = np.clip(depth_image, min_depth, max_depth)
        normalized_depth = ((clipped_depth - min_depth) / (max_depth - min_depth) * 255).astype(np.uint8)

        cv2.imshow("Pose Detection", annotated_image)
        cv2.imshow("Depth Camera Preview", normalized_depth)
        cv2.waitKey(1)

    def visualize_depth_overlay(self, rgb_image, depth_image):
        """
        Visualize the depth measurements overlayed on top of the RGB image
        """
        if depth_image is not None:
            depth_resized = cv2.resize(depth_image, (rgb_image.shape[1], rgb_image.shape[0]))

            depth_resized[np.isnan(depth_resized)] = 0
            depth_resized[np.isinf(depth_resized)] = 0

            depth_normalized = cv2.normalize(depth_resized, None, 0, 255, cv2.NORM_MINMAX)
            depth_normalized = depth_normalized.astype(np.uint8)

            depth_equalized = cv2.equalizeHist(depth_normalized)
            depth_colored = cv2.applyColorMap(depth_equalized, cv2.COLORMAP_JET)

            edges = cv2.Canny(depth_equalized, 50, 150)
            depth_colored[edges > 0] = [255, 255, 255]  # Highlight edges in white

            overlay = cv2.addWeighted(rgb_image, 0.6, depth_colored, 0.4, 0)

            cv2.imshow("Enhanced Depth Overlay", overlay)
            cv2.waitKey(1)

    def check_and_publish_goal(self):
        """
        Publish the latest person's map position as a goal to the /goal_update topic,
        only if it is different from the last published position.
        """
        if self.latest_person_position is not None:
            if self.last_published_position != self.latest_person_position:
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "map"
                pose_msg.pose.position.x = self.latest_person_position[0]
                pose_msg.pose.position.y = self.latest_person_position[1]
                pose_msg.pose.position.z = 0.0
                pose_msg.pose.orientation.x = 0.0
                pose_msg.pose.orientation.y = 0.0
                pose_msg.pose.orientation.z = 0.0
                pose_msg.pose.orientation.w = 1.0
                self.goal_publisher.publish(pose_msg)
                self.last_published_position = self.latest_person_position

    def calculate_3d_coordinates(self, hips, hip_confidences, depth_image):
        """
        Calculate 3D coordinates for detected hips using depth information.

        Args:
            hips: 2D coordinates of detected hips in the image.
            hip_confidences: Confidence scores for the detected hips.
            depth_image: Depth image corresponding to the RGB frame.

        Returns:
            A list of 3D coordinates for the hips and their confidence scores.
        """
        hips_3d = []

        for i, (x, y) in enumerate(hips):
            x = int(np.clip(x, 0, depth_image.shape[1] - 1))
            y = int(np.clip(y, 0, depth_image.shape[0] - 1))

            depth = depth_image[y, x]
            if np.isnan(depth) or depth <= 0:
                continue

            x_world = (x - self.cx) * depth / self.fx
            y_world = (y - self.cy) * depth / self.fy
            z_world = depth

            hips_3d.append((x_world, y_world, z_world))

        return hips_3d, hip_confidences

    def compute_person_map_position_direct(self, hips_3d):
        """
        Compute the person's position in the map frame based on the 3D coordinates
        of the detected hips and the robot's transformation.

        Args:
            hips_3d: List of 3D coordinates of the detected hips.

        Returns:
            The 2D position (x, y) of the person in the map frame.
        """
        try:
            base_to_map_transform = self.tf_buffer.lookup_transform(
                "map", "oakd_rgb_camera_optical_frame", rclpy.time.Time()
            )

            robot_position_map = base_to_map_transform.transform.translation
            robot_orientation = base_to_map_transform.transform.rotation

            yaw = self.get_yaw_from_quaternion(robot_orientation)

            center_hip = np.mean(hips_3d, axis=0)

            offset_x = -center_hip[0]
            offset_z = center_hip[2] - 1

            robot_x = robot_position_map.x
            robot_y = robot_position_map.y

            person_x = robot_x + offset_z * np.cos(yaw) - offset_x * np.sin(yaw)
            person_y = robot_y + offset_z * np.sin(yaw) + offset_x * np.cos(yaw)

            return person_x, person_y

        except TransformException:
            return None

    def get_yaw_from_quaternion(self, quaternion):
        """
        Convert a quaternion to yaw angle in radians.

        Args:
            quaternion: Quaternion representing orientation.

        Returns:
            Yaw angle in radians.
        """
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def get_hips(self, keypoints, confidence_threshold=0.5):
        """
        Extract hip keypoints from YOLOv8 detections and filter them by confidence.

        Args:
            keypoints: Keypoints detected by YOLOv8.
            confidence_threshold: Minimum confidence to consider a detection valid.

        Returns:
            A tuple of hip coordinates and their confidence scores.
        """
        if keypoints.shape[0] < 13:
            return [], []

        hips = keypoints[[11, 12], :2]
        hip_confidences = keypoints[[11, 12], 2]

        if any(conf < confidence_threshold for conf in hip_confidences):
            return [], []

        return hips, hip_confidences


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()