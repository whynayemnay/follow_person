import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from std_msgs.msg import String
import numpy as np
from sensor_msgs.msg import LaserScan



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


        # Subscribe to the front_scan topic from the LiDAR
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan/front',  # Lidar topic from the LiDAR node
            self.lidar_callback,
            10
        )

        # Initialize CvBridge to convert ROS Image messages to OpenCV images
        self.bridge = CvBridge()

        # Define image center threshold for centering logic
        self.image_center_threshold = 0.05  # 5% tolerance for centering

        # Publisher for rotation commands based on person detection
        self.person_detected_publisher = self.create_publisher(String, 'movement_command', 10)

        # LiDAR distance threshold (e.g., 0.5 meters)
        self.safe_distance_threshold = 1.5 # Adjust this value as per your environment
        self.too_close_to_person = False  # Flag for whether the person is too close based on LiDAR

        # Safe distance parameters (adjustable)
        self.min_safe_distance = 1.5  # Minimum distance before moving backward
        self.max_safe_distance = 2  # Maximum distance before moving forward

        self.get_logger().info('Person Detection Node has been started.')

    def image_callback(self, msg):
        # Convert ROS image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        image_width = cv_image.shape[1]

        # Perform YOLOv8 inference
        results = self.model.predict(cv_image, verbose=False)

        # Visualize the detection results
        self.display_results(results)

        # Process keypoints and publish the rotation command
        self.process_keypoints(results, image_width)

    def lidar_callback(self, msg):
        # Process the front-facing LiDAR scan to check for proximity
        if msg.ranges:
            min_distance = min(msg.ranges)
            self.safe_distance_threshold = min_distance  # Update current distance to the closest object (person)

            if min_distance < self.min_safe_distance:  # Too close if less than min_safe_distance
                self.too_close_to_person = True
                # self.get_logger().info(f"Person too close: {min_distance} meters, moving backward.")
            else:
                self.too_close_to_person = False
        else:
            self.too_close_to_person = False

    def display_results(self, results):
        """Displays the annotated image with detection results."""
        annotated_image = results[0].plot()
        cv2.imshow("Pose Detection", annotated_image)
        cv2.waitKey(1)

    def process_keypoints(self, results, image_width):
        """Processes keypoints from the detection results and publishes the movement command."""
        if results[0].keypoints.has_visible:
            keypoints = results[0].keypoints.data[0].cpu().numpy()

            # Extract ankles coordinates and confidence scores
            ankles, ankle_confidences = self.get_hips(keypoints)

            # Determine the person's x position based on visible ankles
            person_x = self.get_person_x_position(ankles, ankle_confidences)
            if person_x is None:
                # self.get_logger().info('No ankles visible, checking LiDAR data for proximity.')
                if self.too_close_to_person:
                    # If the person is too close, publish a backward command
                    self.publish_movement_command('backward')
                else:
                    self.publish_movement_command('stop')  # Stop if not too close
                return

            # Normalize the x position and determine rotation direction
            relative_x = person_x / image_width

            # Check both ankles visibility and proximity before stopping
            if self.too_close_to_person:
                # self.get_logger().info(f"Hips visible but person still too close ({self.safe_distance_threshold} meters). Moving backward.")
                self.publish_movement_command('backward')
            else:
                self.determine_rotation_direction(relative_x)  # Only rotate or stop if not too close
        else:
            # self.get_logger().info('No person detected, checking LiDAR for proximity.')
            if self.too_close_to_person:
                # If the person is too close, publish a backward command
                self.publish_movement_command('backward')
            else:
                self.publish_movement_command('stop')  # Stop if no detection and not too close

    def get_hips(self, keypoints):
        """Extracts the x-coordinates and confidence scores for left and right ankles."""
        ankles = keypoints[[15, 16], 0]  # Left and right ankle x-coordinates
        ankle_confidences = keypoints[[15, 16], 2]  # Confidence values for left and right ankles
        return ankles, ankle_confidences

    def get_person_x_position(self, ankles, ankle_confidences):
        """Returns the x position of the person based on visible ankles."""
        confidence_threshold = 0.5
        valid_hips = ankle_confidences > confidence_threshold

        if valid_hips.all():
            # self.get_logger().info('Both ankles visible, using midpoint for tracking.')
            return np.mean(ankles)
        elif valid_hips[0]:
            # self.get_logger().info('Only left ankle visible, using left ankle for tracking.')
            return ankles[0]
        elif valid_hips[1]:
            # self.get_logger().info('Only right ankle visible, using right ankle for tracking.')
            return ankles[1]
        else:
            return None

    def determine_rotation_direction(self, relative_x):
        """Determines if the person is left, right, or centered, and publishes the rotation command."""
        image_center = 0.5  # Center is at 0.5 (normalized)
        movement_command = String()

        if relative_x < image_center - self.image_center_threshold:
            movement_command.data = 'left'
            self.get_logger().info('Person is to the left, publishing rotate left command.')
        elif relative_x > image_center + self.image_center_threshold:
            movement_command.data = 'right'
            self.get_logger().info('Person is to the right, publishing rotate right command.')
        else:
            if self.too_close_to_person:
                movement_command.data = 'backwards'
                # self.get_logger().info('Person too close, moving backward.')
            elif self.min_safe_distance <= self.safe_distance_threshold <= self.max_safe_distance:
                # if person is centered and not too close and within the ideal range stop all movements
                movement_command.data = 'stop'
                # self.get_logger().info('Person is at ideal range, staying stationary.')
            else:
                movement_command.data = 'forward'
                # self.get_logger().info('Person too far, moving forward')

        self.person_detected_publisher.publish(movement_command)

    def publish_movement_command(self, direction):
        """Publish a movement command (left, right, stop, backwards)"""
        movement_command = String(data=direction)
        self.get_logger().info(f"Publishing {direction} command")
        self.person_detected_publisher.publish(movement_command)


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()