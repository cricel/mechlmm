#!/usr/bin/env python
import sys
import cv2
import time
import threading

import rospy

import moveit_commander
from geometry_msgs.msg import Pose

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, TwistStamped, Point, Pose, Quaternion, PoseStamped

from mechlmm_py import DebugCore

class FunctionPoolDefinition:
    def __init__(self):
        self.debug_log = DebugCore()

        self.dummy_sub = rospy.Subscriber("/dummy",String,self.dummy_callback)

        self.dummy_pub = rospy.Publisher('dummy', String, queue_size=10)

        self.robot_cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.lmm_cmd_publisher = rospy.Publisher('/base_cmd/input/lmm', Twist, queue_size=10)

        self.lmm_arm_control_pub = rospy.Publisher('/arm_control/input/lmm', Float32MultiArray, queue_size=10)
        
        moveit_commander.roscpp_initialize(sys.argv)

        self.moveit_robot = moveit_commander.RobotCommander()
        self.moveit_scene = moveit_commander.PlanningSceneInterface()
        self.moveit_group = moveit_commander.MoveGroupCommander("arm")  

        self.mux_input = True

        rospy.sleep(0.5)

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
        current_pose = self.moveit_group.get_current_pose().pose

        new_position = [
            current_pose.position.x,
            current_pose.position.y,
            current_pose.position.z
        ]
                
        if(_direction == "turn_left"):
            new_position[1] += _distance
        elif(_direction == "turn_right"):
            new_position[1] -= _distance
        elif(_direction == "forward"):
            new_position[0] += _distance
        elif(_direction == "backward"):
            new_position[0] -= _distance
        elif(_direction == "up"):
            new_position[2] += _distance
        elif(_direction == "down"):
            new_position[2] -= _distance
        else:
            return

        if(self.mux_input):
            msg = Float32MultiArray()
            msg.data = new_position
            self.lmm_arm_control_pub.publish(msg)
        else:
            self.moveit_group.set_position_target(new_position)

            plan = self.moveit_group.go(wait=False)

            self.moveit_group.stop()
            self.moveit_group.clear_pose_targets()

    # def navigate(self, _pose):
    #     if(_pose)

    def trigger_gripper(self, _args):
        self.debug_log.log_flash(f"===> trigger_gripper: {_args}")
        print("trigger_gripper")

    def dummy_callback(self, _msg):
        self.arm_pos_control_thread(_msg.data)

def main(args):
    rospy.init_node('function_pool_definition', anonymous=True)

    fpd = FunctionPoolDefinition()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)