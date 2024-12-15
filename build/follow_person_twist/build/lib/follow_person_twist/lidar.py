import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarFrontPublisher(Node):
    def __init__(self):
        super().__init__('lidar_front_publisher')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.publisher_ = self.create_publisher(LaserScan, '/scan/front', 10)

    def scan_callback(self, msg):
        # Log the total number of ranges
        total_ranges = len(msg.ranges)

        # Shift the center index by 160 points to account for lidar facing 9 o'clock (left) and adjust to 12 o'clock (front)
        center_index = (total_ranges // 4)  # This represents the front-facing (12 o'clock) direction after shifting

        # Adjusted FOV: 40 degrees total, 20 degrees to the left and right of the center index
        fov_half_points = int(20 / 360 * total_ranges // 2)  # 20 degrees on each side of the center
        left_bound = (center_index - fov_half_points) % total_ranges
        right_bound = (center_index + fov_half_points) % total_ranges

        # Calculate the number of front ranges
        if left_bound <= right_bound:
            front_ranges_count = right_bound - left_bound
        else:
            front_ranges_count = total_ranges - left_bound + right_bound

        # Create a new LaserScan message for the front scan
        front_scan = LaserScan()
        front_scan.header = msg.header
        front_scan.angle_min = msg.angle_min + left_bound * msg.angle_increment
        front_scan.angle_max = msg.angle_min + right_bound * msg.angle_increment
        front_scan.angle_increment = msg.angle_increment
        front_scan.time_increment = msg.time_increment
        front_scan.scan_time = msg.scan_time
        front_scan.range_min = msg.range_min
        front_scan.range_max = msg.range_max

        # Copy only the front ranges
        if left_bound < right_bound:
            front_scan.ranges = msg.ranges[left_bound:right_bound]
        else:
            # Handle wrap-around case
            front_scan.ranges = msg.ranges[left_bound:] + msg.ranges[:right_bound]

        # Log the lowest distance in the front ranges (ignoring inf or NaN values)
        valid_ranges = [r for r in front_scan.ranges if r > front_scan.range_min and r < front_scan.range_max]
        if valid_ranges:
            min_distance = min(valid_ranges)
        else:
            min_distance = float('inf')  # In case there are no valid ranges

        # Publish the front scan data
        self.publisher_.publish(front_scan)
        # self.get_logger().info(
        #     f"Total number of ranges: {total_ranges}, "
        #     f"Number of front ranges: {front_ranges_count}, "
        #     f"Left bound: {left_bound}, Right bound: {right_bound}, "
        #     f"Center index: {center_index}, "
        #     f"Lowest distance detected: {min_distance:.2f} meters"
        # )

def main(args=None):
    rclpy.init(args=args)
    lidar_front_publisher = LidarFrontPublisher()
    rclpy.spin(lidar_front_publisher)
    lidar_front_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()