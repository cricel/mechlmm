import sys
import cv2
import time
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, TwistStamped, Point, Pose, Quaternion, PoseStamped

from mechlmm_py import DebugCore

class FunctionPoolDefinition(Node):
    def __init__(self, node_name='function_pool_definition'):
        super().__init__(node_name)
        self.debug_log = DebugCore()

        # Create subscribers
        self.dummy_sub = self.create_subscription(
            String, 
            "/dummy", 
            self.dummy_callback, 
            10
        )

        # Create publishers
        self.dummy_pub = self.create_publisher(String, 'dummy', 10)
        self.robot_cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lmm_cmd_publisher = self.create_publisher(Twist, '/base_cmd/input/lmm', 10)
        self.lmm_arm_control_pub = self.create_publisher(Float32MultiArray, '/arm_control/input/lmm', 10)

        self.mux_input = True

        # Wait for publishers to be ready
        time.sleep(0.5)

        ## testing
        # self.move_robot("forward")
        # self.arm_cmd_thread("forward")

    def move_robot(self, _args):
        self.debug_log.log_flash(f"===> move_robot: {_args}")
  
        thread = threading.Thread(target=self.robot_cmd_thread, args=(_args["direction"],))
        thread.start()

    def robot_cmd_thread(self, _direction, _distance = 0.3, _rotation = 0.3):
        msg = Twist()

        if(_direction == "forward"):
            msg.linear.x = _distance
            
        elif(_direction == "turn_left"):
            msg.angular.z = _rotation

        elif(_direction == "turn_right"):
            msg.angular.z = -_rotation

        elif(_direction == "backward"):
            msg.linear.x = -_distance

        elif(_direction == "stop"):
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        if(self.mux_input):
            self.lmm_cmd_publisher.publish(msg)
        else:
            self.robot_cmd_publisher.publish(msg)
            self.debug_log.log_info(msg)

            time.sleep(1)

            msg.linear.x = 0.0
            msg.angular.z = 0.0

            self.debug_log.log_info("-----")
            self.debug_log.log_info(msg)

            self.robot_cmd_publisher.publish(msg)
        
        return msg

    def arm_end_effector_control(self, _args):
        self.debug_log.log_flash(f"===> move_arm: {_args}")
  
        thread = threading.Thread(target=self.arm_pos_control_thread, args=(_args["direction"],))
        thread.start()

    def arm_pos_control_thread(self, _direction, _distance = 0.01):
        # For ROS2, we'll use a simpler approach without MoveIt for now
        # You can integrate MoveIt2 later if needed
        
        if(self.mux_input):
            # Create a simple position control message
            msg = Float32MultiArray()
            # Default position - you can modify this based on your robot's current pose
            current_position = [0.0, 0.0, 0.0]  # x, y, z
            
            if(_direction == "turn_left"):
                current_position[1] += _distance
            elif(_direction == "turn_right"):
                current_position[1] -= _distance
            elif(_direction == "forward"):
                current_position[0] += _distance
            elif(_direction == "backward"):
                current_position[0] -= _distance
            elif(_direction == "up"):
                current_position[2] += _distance
            elif(_direction == "down"):
                current_position[2] -= _distance
            else:
                return

            msg.data = current_position
            self.lmm_arm_control_pub.publish(msg)
        else:
            # For non-mux mode, you can implement direct control here
            self.get_logger().info(f"Direct arm control: {_direction}")

    def trigger_gripper(self, _args):
        self.debug_log.log_flash(f"===> trigger_gripper: {_args}")
        self.get_logger().info("trigger_gripper")

    def dummy_callback(self, _msg):
        self.arm_pos_control_thread(_msg.data)

def main(args=None):
    rclpy.init(args=args)
    
    fpd = FunctionPoolDefinition()
    
    try:
        rclpy.spin(fpd)
    except KeyboardInterrupt:
        fpd.get_logger().info("Shutting down")
    finally:
        fpd.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()