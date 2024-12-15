#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
import time


class Nav2Client(Node):
    def __init__(self):
        """
        Initialize the Nav2Client node. Sets up action clients, services, and subscriptions.
        """
        super().__init__('nav2_navigate_to_pose_client')

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.navigation_active = False
        self.goal_handle = None
        self.goal_sent_time = None

        self.subscription = None

        self.start_service = self.create_service(Trigger, '/start_navigation', self.start_navigation_service)
        self.stop_service = self.create_service(Trigger, '/stop_navigation', self.stop_navigation_service)

    def start_navigation_service(self, request, response):
        """
        Service callback to start navigation. Activates goal subscriptions.

        Args:
            request (std_srvs.srv.Trigger.Request): The service request.
            response (std_srvs.srv.Trigger.Response): The service response.

        Returns:
            std_srvs.srv.Trigger.Response: Response indicating whether navigation was started successfully.
        """
        if self.navigation_active:
            response.success = False
            response.message = "Navigation is already active."
        else:
            self.navigation_active = True
            self.subscription = self.create_subscription(
                PoseStamped, '/goal_update', self.goal_callback, 10
            )
            response.success = True
            response.message = "Navigation started."
        return response

    def stop_navigation_service(self, request, response):
        """
        Service callback to stop navigation. Cancels the current goal and removes subscriptions.

        Args:
            request (std_srvs.srv.Trigger.Request): The service request.
            response (std_srvs.srv.Trigger.Response): The service response.

        Returns:
            std_srvs.srv.Trigger.Response: Response indicating whether navigation was stopped successfully.
        """
        if not self.navigation_active:
            response.success = False
            response.message = "Navigation is not active."
        else:
            self.navigation_active = False
            self.destroy_subscription(self.subscription)
            self.subscription = None
            self.cancel_current_goal()
            response.success = True
            response.message = "Navigation stopped."
        return response

    def goal_callback(self, msg: PoseStamped):
        """
        Callback for receiving goal updates. Sends the goal if navigation is active.

        Args:
            msg (geometry_msgs.msg.PoseStamped): The new goal position and orientation.
        """
        if self.navigation_active:
            self.send_goal(msg)

    def send_goal(self, pose_msg: PoseStamped):
        """
        Send a navigation goal to the Nav2 action server.

        Args:
            pose_msg (geometry_msgs.msg.PoseStamped): The target goal pose.
        """
        if not self.navigation_active:
            return

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_msg
        goal_msg.behavior_tree = ''

        send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

        self.goal_sent_time = time.time()

    def goal_response_callback(self, future):
        """
        Callback for handling the response to a goal request.

        Args:
            future: The future object containing the result of the goal request.
        """
        if not self.navigation_active:
            return

        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.goal_handle = None
            return

        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Callback for receiving feedback during goal execution.

        Args:
            feedback_msg: Feedback message from the Nav2 action server.
        """
        if not self.navigation_active:
            return

        feedback = feedback_msg.feedback
        distance_remaining = feedback.distance_remaining

        if self.goal_sent_time and time.time() - self.goal_sent_time < 0.5:
            return

        if distance_remaining == 0.0 and self.goal_handle is not None:
            return

        if distance_remaining < 1.0 and self.goal_handle is not None:
            self.cancel_current_goal()

    def result_callback(self, future):
        """
        Callback for handling the result of a navigation goal.

        Args:
            future: The future object containing the result of the goal.
        """
        if not self.navigation_active:
            return

        result_code = future.result().status

        if result_code == 3:
            pass
        elif result_code == 4:
            pass
        elif result_code == 5:
            pass

        self.goal_handle = None

    def cancel_current_goal(self):
        """
        Cancel the currently active navigation goal.
        """
        if self.goal_handle is None:
            return

        cancel_future = self.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_response_callback)

    def cancel_response_callback(self, future):
        """
        Callback for handling the response to a goal cancellation request.

        Args:
            future: The future object containing the cancellation response.
        """
        cancel_response = future.result()

        if cancel_response.return_code == 0:
            pass

        self.goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = Nav2Client()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()