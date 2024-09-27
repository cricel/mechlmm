#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from py_trees.behaviours import Success, Failure
from py_trees.composites import Sequence, Fallback
from py_trees.decorators import FailureIsSuccess

class GoToPoseAction(Node):
    def __init__(self, name, pose):
        super().__init__('go_to_pose_' + name)
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pose = pose
        self.goal_reached = False

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.pose
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f'Goal {self.pose} rejected')
            self.goal_reached = False
            return Failure()
        self.get_logger().info(f'Goal {self.pose} accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result:
            self.get_logger().info(f'Goal {self.pose} reached')
            self.goal_reached = True
            return Success()
        else:
            self.get_logger().info(f'Goal {self.pose} failed')
            return Failure()

class TriggerArmAction(Node):
    def __init__(self, name):
        super().__init__('trigger_arm_' + name)
        self.arm_triggered = False

    def trigger(self):
        # Trigger arm action
        self.get_logger().info(f'Triggering arm action {self.get_name()}')
        # Simulating success/failure outcome
        self.arm_triggered = True  # Simulating success
        if self.arm_triggered:
            self.get_logger().info(f'Arm action {self.get_name()} succeeded')
            return Success()
        else:
            self.get_logger().info(f'Arm action {self.get_name()} failed')
            return Failure()

def create_behavior_tree():
    # Define poses
    pose_1 = PoseStamped()
    pose_1.pose.position.x = -0.15
    pose_1.pose.position.y = 1.45
    pose_1.pose.orientation.w = 1.0

    pose_2 = PoseStamped()
    pose_2.pose.position.x = 0.9
    pose_2.pose.position.y = 1.2
    pose_2.pose.orientation.w = 1.0

    pose_3 = PoseStamped()
    pose_3.pose.position.x = 0.0
    pose_3.pose.position.y = 0.0
    pose_3.pose.orientation.w = 1.0

    # Actions
    go_to_pose_action_1 = GoToPoseAction("1", pose_1)
    go_to_pose_action_2 = GoToPoseAction("2", pose_2)
    go_to_pose_action_3 = GoToPoseAction("3", pose_3)
    trigger_arm_action_1 = TriggerArmAction("1")

    # Sequence and fallback structures for behavior tree
    arm_action_seq = Sequence("Arm Success Sequence")
    arm_action_seq.add_child(trigger_arm_action_1)
    arm_action_seq.add_child(go_to_pose_action_3)

    go_to_pose_1_fallback = Fallback("Go to Pose 1 Fallback")
    go_to_pose_1_fallback.add_child(go_to_pose_action_1)
    go_to_pose_1_fallback.add_child(go_to_pose_action_2)

    root = Sequence("Root Sequence")
    root.add_child(go_to_pose_1_fallback)
    root.add_child(arm_action_seq)

    return root

def main(args=None):
    rclpy.init(args=args)

    # Create and run the behavior tree
    root = create_behavior_tree()

    # Run the behavior tree (assuming you integrate with some py_trees runner)
    behaviour_tree = py_trees.trees.BehaviourTree(root)
    behaviour_tree.setup(timeout=15.0)

    # Run tree in a tick loop
    for _ in range(10):
        behaviour_tree.tick_tock(500)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
