#!/usr/bin/env python

import sys

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

import moveit_commander

class KeyboardCommander:
    def __init__(self):
        rospy.init_node('operator_arm_manual_control', anonymous=True)

        self.operator_arm_control_pub = rospy.Publisher('/arm_control/input/keyboard', Float32MultiArray, queue_size=10)
        self.mux_cmd_pub = rospy.Publisher('/base_cmd/input/keyboard', Twist, queue_size=10)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.keyboard_sub = rospy.Subscriber("/keyboard",String, self.keyboard_callback)
        
        moveit_commander.roscpp_initialize(sys.argv)

        self.moveit_robot = moveit_commander.RobotCommander()
        self.moveit_scene = moveit_commander.PlanningSceneInterface()
        self.moveit_group = moveit_commander.MoveGroupCommander("arm")  

        self.arm_move_distance = 0.01
        self.base_linear_speed = 0.2
        self.base_rotate_speed = 0.4

        self.mux_input = True
        

    def keyboard_callback(self, _msg):
        self.arm_key_control(_msg)
        self.base_key_control(_msg)

    def base_key_control(self, _msg):
        move_cmd = Twist()

        if(_msg.data == 'w'):
            move_cmd.linear.x = self.base_linear_speed
            
        elif(_msg.data == 'a'):
            move_cmd.angular.z = self.base_rotate_speed

        elif(_msg.data == 'd'):
            move_cmd.angular.z = -self.base_rotate_speed

        elif(_msg.data == 's'):
            move_cmd.linear.x = -self.base_linear_speed

        elif(_msg.data == 'x'):
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
        else:
            return

        if(self.mux_input):
            self.mux_cmd_pub.publish(move_cmd)
        else:
            self.cmd_pub.publish(move_cmd)
        
    def arm_key_control(self, _msg):
        current_pose = self.moveit_group.get_current_pose().pose
        new_position = [
            current_pose.position.x,
            current_pose.position.y,
            current_pose.position.z
        ]

        if(_msg.data == '4'):
            new_position[1] += self.arm_move_distance
        elif(_msg.data == '6'):
            new_position[1] -= self.arm_move_distance
        elif(_msg.data == '8'):
            new_position[0] += self.arm_move_distance
        elif(_msg.data == '2'):
            new_position[0] -= self.arm_move_distance
        elif(_msg.data == '9'):
            new_position[2] += self.arm_move_distance
        elif(_msg.data == '3'):
            new_position[2] -= self.arm_move_distance
        else:
            return
        
        if(not self.mux_input):
            try:
                self.moveit_group.set_position_target(new_position)
            
                plan = self.moveit_group.go(wait=True)
            
                self.moveit_group.stop()
                self.moveit_group.clear_pose_targets()
            except:
                pass

        msg = Float32MultiArray()
        msg.data = new_position

        self.operator_arm_control_pub.publish(msg)

    def run(self):
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Shutting down keyboard input publisher.")

if __name__ == '__main__':
    try:
        node = KeyboardCommander()

        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.shutdown()