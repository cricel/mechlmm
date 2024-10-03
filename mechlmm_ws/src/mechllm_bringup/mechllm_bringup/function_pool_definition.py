import rclpy
from rclpy.node import Node

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

from std_msgs.msg import String
from geometry_msgs.msg import Twist, TwistStamped, Point, Pose, Quaternion, PoseStamped

import time
import threading

from mechlmm_py import DebugCore


class FunctionPoolDefinition(Node):
    def __init__(self, node_name='function_pool_definition'):
        super().__init__(node_name)
        self.debug_log = DebugCore()

        self.dummy_publisher = self.create_publisher(String, '/dummy', 10)
        self.robot_cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_cmd_publisher = self.create_publisher(Twist, '/servo_node/delta_twist_cmds', 10)

        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_goal_handle = None


    #region LLM Function Definition

    def navigation(self, target_name):
        self.debug_log.log_flash(f"===> navigation: {target_name}")

        self.send_goal(1,1,1)
        
    def manipulation(self, target_name):
        self.debug_log.log_flash(f"===> manipulation: {target_name}")

    def move_robot(self, _args):
        self.debug_log.log_flash(f"===> move_robot: {_args}")
  
        thread = threading.Thread(target=self.robot_cmd_thread, args=(_args["direction"],))
        thread.start()

    def trigger_gripper(self, trigger):
        self.debug_log.log_flash(f"===> trigger_gripper: {trigger}")

    def arm_end_effector_rotation_control(self, rotation):
        self.debug_log.log_flash(f"===> arm_end_effector_rotation_control: {rotation}")

        arm_twist = TwistStamped()
        arm_twist.header.frame_id="link2"
        arm_twist.header.stamp = self.get_clock().now().to_msg()

        arm_twist.twist.angular.y = rotation

        self.arm_cmd_publisher.publish(arm_twist)

    def arm_end_effector_control(self, direction):
        self.debug_log.log_flash(f"===> arm_end_effector_control: {direction}")

        arm_twist = TwistStamped()
        arm_twist.header.frame_id="link2"
        arm_twist.header.stamp = self.get_clock().now().to_msg()

        if(direction == "forward"):
            arm_twist.twist.linear.x = 0.2
        elif(direction == "backward"):
            arm_twist.twist.linear.x = -0.2
        elif(direction == "up"):
            arm_twist.twist.linear.z = 0.2
        elif(direction == "down"):
            arm_twist.twist.linear.z = -0.2
        elif(direction == "turn_left"):
            arm_twist.twist.angular.z = 0.2
        elif(direction == "turn_right"):
            arm_twist.twist.angular.z = -0.2

        self.arm_cmd_publisher.publish(arm_twist)

    #endregion


    #region Helper Funcations

    def threading_test_function(self):
        msg = String()
        msg.data = "Hello World!:"
        self.dummy_publisher.publish(msg)

        time.sleep(1)

        msg.data = "Bye World!:"
        self.dummy_publisher.publish(msg)

    def test_publisher(self):
        thread = threading.Thread(target=self.threading_test_function)
        thread.start()


    def robot_cmd_thread(self, direction):
        msg = Twist()

        if(direction == "forward"):
            msg.linear.x = 0.3
            
        elif(direction == "turn_left"):
            msg.angular.z = 0.3

        elif(direction == "turn_right"):
            msg.angular.z = -0.3

        elif(direction == "backward"):
            msg.linear.x = -0.3

        elif(direction == "stop"):
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.robot_cmd_publisher.publish(msg)
        self.debug_log.log_info(msg)

        time.sleep(1)

        msg.linear.x = 0.0
        msg.angular.z = 0.0

        self.debug_log.log_info("-----")
        self.debug_log.log_info(msg)

        self.robot_cmd_publisher.publish(msg)


    def send_goal(self, x, y, z):
        goal_msg = NavigateToPose.Goal()
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z
        goal_pose.pose.orientation.w = 1.0
        
        goal_msg.pose = goal_pose

        self.nav_action_client.wait_for_server()
        future = self.nav_action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.nav_goal_handle = future.result()
        if not self.nav_goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = self.nav_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('Goal reached within 1cm!')
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(result.status))

    #endregion



if __name__ == '__main__':
    pass