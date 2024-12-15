import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from std_msgs.msg import String
import numpy as np


class PersonDetectionNode(Node):
    def __init__(self):
        super().__init__('person_detection_node')

        # Initialize YOLOv8 model (pretrained on COCO dataset)
        self.model = YOLO('yolov8n-pose.pt')

        # Subscribe to the image topic from your camera
        self.subscription = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self.image_callback,
            10
        )

        # Subscribe to the depth image topic from the camera
        self.depth_subscription = self.create_subscription(
            Image,
            '/oakd/rgb/preview/depth',
            self.depth_callback,
            10
        )

        # Initialize CvBridge to convert ROS Image messages to OpenCV images
        self.bridge = CvBridge()

        # Define image center threshold for centering logic
        self.image_center_threshold = 0.05  # 5% tolerance for centering

        # Publisher for movement commands based on person detection
        self.person_detected_publisher = self.create_publisher(String, 'movement_command', 10)

        # Safe distance parameters (adjustable)
        self.min_safe_distance = 1.5  # Minimum distance before moving backward
        self.max_safe_distance = 2  # Maximum distance before moving forward

        self.get_logger().info('Person Detection Node has been started.')

    def image_callback(self, msg):
        # Convert ROS image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        image_width = cv_image.shape[1]

        # Perform YOLOv8 inference
        results = self.model.predict(cv_image, verbose=False, conf=0.6)

        # Visualize the detection results
        self.display_results(results)

        # Process keypoints and publish the movement command
        self.process_keypoints(results, image_width)

    def depth_callback(self, msg):
        # Convert the depth image message to a NumPy array
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')  # 32-bit float per pixel
        # Define min and max depth values for visualization (e.g., 0–35 meters)
        min_depth = 0.0
        max_depth = 35.0

        # Clip depth values to the defined range
        clipped_depth = np.clip(self.depth_image, min_depth, max_depth)

        # Normalize the depth image to an 8-bit range for display
        normalized_depth = ((clipped_depth - min_depth) / (max_depth - min_depth) * 255).astype(np.uint8)

        # Display the normalized depth image
        cv2.imshow("Depth Camera Preview", normalized_depth)
        cv2.waitKey(1)

    def display_results(self, results):
        """Displays the annotated image with detection results."""
        annotated_image = results[0].plot()
        cv2.imshow("Pose Detection", annotated_image)
        cv2.waitKey(1)
    
    def process_keypoints(self, results, image_width):
        if not results[0].keypoints.has_visible:
            # self.get_logger().info('No keypoints detected, skipping distance calculation.')
            return
        if hasattr(self, 'depth_image'):
            keypoints = results[0].keypoints.data[0].cpu().numpy()

            # Extract hips coordinates and confidence scores
            hips, hip_confidences = self.get_hips(keypoints)

            # Filter hips based on confidence level
            confidence_threshold = 0.5  # Adjust this value as needed
            depth_values = []
            hip_count = 0  # Count of detected hips for validation

            for i, (x, y) in enumerate([(int(hips[0]), int(keypoints[11][1])), (int(hips[1]), int(keypoints[12][1]))]):
                if hip_confidences[i] >= confidence_threshold:
                    if 0 <= x < self.depth_image.shape[1] and 0 <= y < self.depth_image.shape[0]:
                        depth_value = self.depth_image[y, x]
                        if not np.isnan(depth_value) and depth_value > 0:  # Check for valid depth values
                            depth_values.append(depth_value)
                            hip_count += 1

            if hip_count == 1:  # Only one hip is detected, use that for distance
                average_depth = depth_values[0]
                # self.get_logger().info(f'Depth based on one hip: {average_depth:.2f} meters')
            elif hip_count == 2:  # Both hips detected, use average
                average_depth = np.mean(depth_values)
                # self.get_logger().info(f'Average depth based on both hips: {average_depth:.2f} meters')
            else:
                self.get_logger().warning('Bounding box detected without hips, no valid depth data can be found.')
                return

            # Call the unified movement determination function
            self.determine_movement(hips, image_width, average_depth)
        else:
            self.get_logger().info('No person detected or depth image not available.')

    def get_hips(self, keypoints):
        """Extracts the x-coordinates and confidence scores for left and right hips."""
        hips = keypoints[[11, 12], 0]  # Left and right hip x-coordinates
        hip_confidences = keypoints[[11, 12], 2]  # Confidence values for left and right hips
        return hips, hip_confidences

    def determine_movement(self, hips, image_width, average_depth):
        """Determines the overall movement command (rotation, forward, backward, or stop)."""
        person_x = np.mean(hips) if len(hips) == 2 and not np.any(np.isnan(hips)) else hips[0]

        # Normalize the x position and determine rotation direction
        relative_x = person_x / image_width
        image_center = 0.5  # Center is at 0.5 (normalized)
        movement_command = String()

        if relative_x < image_center - self.image_center_threshold:
            movement_command.data = 'left'
            # self.get_logger().info('Person is to the left, publishing rotate left command.')
        elif relative_x > image_center + self.image_center_threshold:
            movement_command.data = 'right'
            # self.get_logger().info('Person is to the right, publishing rotate right command.')
        else:
            # Check the average depth to decide forward or backward movement
            if average_depth < self.min_safe_distance:
                movement_command.data = 'backward'
                # self.get_logger().info('Person too close, moving backward.')
            elif self.min_safe_distance <= average_depth <= self.max_safe_distance:
                movement_command.data = 'stop'
                # self.get_logger().info('Person is at ideal range, staying stationary.')
            else:
                movement_command.data = 'forward'
                # self.get_logger().info('Person too far, moving forward.')

        self.person_detected_publisher.publish(movement_command)


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
