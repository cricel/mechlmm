#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

import sys
import time

import moveit_commander

class MuxCommander:
    def __init__(self):
        rospy.init_node('mux_commander', anonymous=True)

        self.keyboard_arm_sub = rospy.Subscriber("/arm_control/input/keyboard",Float32MultiArray, self.keyboard_arm_callback)
        self.lmm_arm_sub = rospy.Subscriber("/arm_control/input/lmm",Float32MultiArray, self.lmm_arm_callback)
        self.static_arm_sub = rospy.Subscriber("/arm_control/input/static",Float32MultiArray, self.static_arm_callback)

        self.keyboard_base_sub = rospy.Subscriber("/base_cmd/input/keyboard",Twist, self.keyboard_base_callback)
        self.lmm_base_sub = rospy.Subscriber("/base_cmd/input/lmm",Twist, self.lmm_base_callback)
        self.static_base_sub = rospy.Subscriber("/base_cmd/input/static",Twist, self.static_base_callback)

        self.lmm_command_sub = rospy.Subscriber("/lmm/command",String, self.lmm_command_callback)

        self.robot_cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        moveit_commander.roscpp_initialize(sys.argv)

        self.moveit_robot = moveit_commander.RobotCommander()
        self.moveit_scene = moveit_commander.PlanningSceneInterface()
        self.moveit_group = moveit_commander.MoveGroupCommander("arm")  


        self.keyboard_arm_data = {"timestamp": None,
                                  "data": None}
        self.lmm_arm_data = {"timestamp": None,
                                  "data": None}
        self.static_arm_data = {"timestamp": None,
                                  "data": None}
        
        self.keyboard_base_data = {"timestamp": None,
                                  "data": None}
        self.lmm_base_data = {"timestamp": None,
                                  "data": None}
        self.static_base_data = {"timestamp": None,
                                  "data": None}
    
    def keyboard_base_callback(self, _msg):
        self.keyboard_base_data["timestamp"] = time.time()
        self.keyboard_base_data["data"] = []
        self.keyboard_base_data["data"].append(_msg.linear.x)
        self.keyboard_base_data["data"].append(_msg.linear.y)
        self.keyboard_base_data["data"].append(_msg.linear.z)
        self.keyboard_base_data["data"].append(_msg.angular.x)
        self.keyboard_base_data["data"].append(_msg.angular.y)
        self.keyboard_base_data["data"].append(_msg.angular.z)
    
    def lmm_base_callback(self, _msg):
        self.lmm_base_data["timestamp"] = time.time()
        self.lmm_base_data["data"] = []
        self.lmm_base_data["data"].append(_msg.linear.x)
        self.lmm_base_data["data"].append(_msg.linear.y)
        self.lmm_base_data["data"].append(_msg.linear.z)
        self.lmm_base_data["data"].append(_msg.angular.x)
        self.lmm_base_data["data"].append(_msg.angular.y)
        self.lmm_base_data["data"].append(_msg.angular.z)

    def static_base_callback(self, _msg):
        self.static_base_data["timestamp"] = time.time()
        self.static_base_data["data"] = []
        self.static_base_data["data"].append(_msg.linear.x)
        self.static_base_data["data"].append(_msg.linear.y)
        self.static_base_data["data"].append(_msg.linear.z)
        self.static_base_data["data"].append(_msg.angular.x)
        self.static_base_data["data"].append(_msg.angular.y)
        self.static_base_data["data"].append(_msg.angular.z)


    def keyboard_arm_callback(self, _msg):
        self.keyboard_arm_data["timestamp"] = time.time()
        self.keyboard_arm_data["data"] = _msg.data

    def lmm_arm_callback(self, _msg):
        self.lmm_arm_data["timestamp"] = time.time()
        self.lmm_arm_data["data"] = _msg.data

    def static_arm_callback(self, _msg):
        self.static_arm_data["timestamp"] = time.time()
        self.static_arm_data["data"] = _msg.data


    def lmm_command_callback(self, _msg):
        if(_msg.data == "none"):
            print("no lmm call")

        elif(_msg.data == "arm"):
            if(self.keyboard_arm_data["timestamp"] == None):
                self.keyboard_arm_data["data"] = None

            if(self.lmm_arm_data["timestamp"] == None):
                self.lmm_arm_data["data"] = None

            if(self.static_arm_data["timestamp"] == None):
                self.static_arm_data["data"] = None
            
            _result = self.dynamic_weight_anylyzer(self.keyboard_arm_data["data"],
                                                self.lmm_arm_data["data"],
                                                self.static_arm_data["data"])
            
            self.moveit_group.set_position_target(_result)

            self.keyboard_arm_data["timestamp"] = None
            self.lmm_arm_data["timestamp"] = None
            self.static_arm_data["timestamp"] = None

            plan = self.moveit_group.go(wait=False)
            self.moveit_group.stop()
            self.moveit_group.clear_pose_targets()

        elif(_msg.data == "base"):
            if(self.keyboard_base_data["timestamp"] == None):
                self.keyboard_base_data["data"] = None

            if(self.lmm_base_data["timestamp"] == None):
                self.lmm_base_data["data"] = None

            if(self.static_base_data["timestamp"] == None):
                self.static_base_data["data"] = None
            
            print(self.keyboard_base_data["data"])

            _result = self.dynamic_weight_anylyzer(self.keyboard_base_data["data"],
                                                self.lmm_base_data["data"],
                                                self.static_base_data["data"])
            
            print(_result)
            
            msg = Twist()
            msg.linear.x = _result[0]
            msg.linear.y = _result[1]
            msg.linear.z = _result[2]
            msg.angular.x = _result[3]
            msg.angular.y = _result[4]
            msg.angular.z = _result[5]

            self.robot_cmd_publisher.publish(msg)

            time.sleep(1)

            msg.linear.x = 0.0
            msg.angular.z = 0.0

            self.robot_cmd_publisher.publish(msg)

    def get_first_non_empty_length(self, *arrays):
        for arr in arrays:
            if arr is not None:
                return len(arr)
        return 0

    def dynamic_weight_anylyzer(self, _operator_input = None, _lmm_input = None, _static_input = None):
        input_size = self.get_first_non_empty_length(_operator_input, _lmm_input, _static_input)

        operator_weight = 0.6
        lmm_weight = 0.1
        static_weight = 0.3

        if(_operator_input == None):
            _operator_input = [0] * input_size

            operator_weight = 0.0

        if(_lmm_input == None):
            _lmm_input = [0] * input_size

            lmm_weight = 0.0

        if(_static_input == None):
            _static_input = [0] * input_size

            static_weight = 0.0

        total_weight = operator_weight + lmm_weight + static_weight

        result = []
        for i in range(input_size):
            weighted_sum = operator_weight/total_weight * _operator_input[i] + lmm_weight/total_weight * _lmm_input[i] + static_weight/total_weight * _static_input[i]
            result.append(weighted_sum)

        print(f"final weight: {result}")
        return result
            
    def run(self):
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Shutting down mux commander.")

if __name__ == '__main__':
    try:
        node = MuxCommander()

        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.shutdown()