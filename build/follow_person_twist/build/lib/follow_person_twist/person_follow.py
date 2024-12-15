import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger  # Import Trigger service for start/stop commands


class PersonFollowerNode(Node):
    def __init__(self):
        super().__init__('person_follower_node')

        # Initialize the state
        self.following_active = False  # Flag to check if following is active
        self.current_command = 'stop'
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.linear_acceleration = 0.05  # Linear speed increment
        self.angular_acceleration = 0.1  # Angular speed increment
        self.max_linear_speed = 0.46
        self.max_angular_speed = 0.68

        # Subscriber for the movement command
        self.person_detected_subscriber = None  # Subscription is created only when following starts

        # Publisher for controlling robot movement
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Services to start and stop following
        self.start_service = self.create_service(Trigger, '/start_following', self.start_following_service)
        self.stop_service = self.create_service(Trigger, '/stop_following', self.stop_following_service)

        self.get_logger().info('Person Follower Node has been started. Use services to control following.')

    def start_following_service(self, request, response):
        """Service callback to start following."""
        if self.following_active:
            response.success = False
            response.message = "Following is already active."
        else:
            self.following_active = True
            # Create the subscription to the movement command
            self.person_detected_subscriber = self.create_subscription(
                String,
                'movement_command',
                self.movement_command_callback,
                10
            )
            response.success = True
            response.message = "Following started. Listening for movement commands."
            self.get_logger().info(response.message)
        return response

    def stop_following_service(self, request, response):
        """Service callback to stop following."""
        if not self.following_active:
            response.success = False
            response.message = "Following is not active."
        else:
            self.following_active = False
            # Destroy the subscription to stop processing movement commands
            self.destroy_subscription(self.person_detected_subscriber)
            self.person_detected_subscriber = None
            # Stop the robot's movement
            self.stop_robot()
            response.success = True
            response.message = "Following stopped. No longer processing movement commands."
            self.get_logger().info(response.message)
        return response

    def movement_command_callback(self, msg):
        if not self.following_active:
            self.get_logger().info("Following is inactive. Ignoring movement command.")
            return

        # Update the detection status
        self.current_command = msg.data

        # Determine the target velocities
        target_linear_speed = 0.0
        target_angular_speed = 0.0

        if self.current_command == 'left':
            target_angular_speed = self.max_angular_speed  # Rotate counterclockwise
        elif self.current_command == 'right':
            target_angular_speed = -self.max_angular_speed  # Rotate clockwise
        elif self.current_command == 'backward':
            target_linear_speed = -self.max_linear_speed  # Move backward
        elif self.current_command == 'forward':
            target_linear_speed = self.max_linear_speed  # Move forward
        else:
            target_linear_speed = 0.0
            target_angular_speed = 0.0

        # Smooth acceleration/deceleration for linear and angular velocities
        self.current_linear_speed = self.smooth_velocity(
            self.current_linear_speed, target_linear_speed, self.linear_acceleration
        )
        self.current_angular_speed = self.smooth_velocity(
            self.current_angular_speed, target_angular_speed, self.angular_acceleration
        )

        # Create a Twist message with the smoothed velocities
        twist = Twist()
        twist.linear.x = self.current_linear_speed
        twist.angular.z = self.current_angular_speed

        # Publish the velocity command to the robot
        self.cmd_vel_publisher.publish(twist)

    def stop_robot(self):
        """Stop the robot's movement by publishing zero velocities."""
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Robot stopped.")

    def smooth_velocity(self, current, target, acceleration):
        # Smoothly adjust the velocity to avoid sudden jumps
        if current < target:
            current = min(current + acceleration, target)
        elif current > target:
            current = max(current - acceleration, target)
        return current


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()