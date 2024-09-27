#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration

class GoalPoseNav(Node):
    def __init__(self):
        super().__init__('goal_pose_nav')
        
        # Create an action client to send the goal to the navigation stack
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, pose: PoseStamped):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        # Wait until the action server is ready
        self._action_client.wait_for_server()

        # Send the goal pose to the navigation action server
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    node = GoalPoseNav()

    # Define the goal pose (example)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = node.get_clock().now().to_msg()

    # Set the position and orientation of the goal pose
    goal_pose.pose.position.x = -0.15  # Example x-coordinate #0.9  -0.15
    goal_pose.pose.position.y = 1.45  # Example y-coordinate  #1.2  1.45
    goal_pose.pose.orientation.z = 0.707  # Example quaternion z
    goal_pose.pose.orientation.w = 0.707  # Example quaternion w

    node.send_goal(goal_pose)

    # Spin the node to execute callbacks
    rclpy.spin(node)

if __name__ == '__main__':
    main()
