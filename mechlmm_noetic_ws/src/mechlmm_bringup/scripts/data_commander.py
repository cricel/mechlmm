#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

import sys
import time
from datetime import datetime

class DataCommander:
    def __init__(self):
        rospy.init_node('data_commander', anonymous=True)
        self.data_collection_duration = 0.2

        self.static_base_sub = rospy.Subscriber("/cmd_vel",Twist, self.robot_speed_callback)

    def data_speed_gate(self):
        current_time = time.time()
        if (current_time - self.last_saved_time >= self.data_collection_duration):
            self.last_saved_time = current_time
            return True
        return False

    def robot_speed_callback(self, msg):
        if(not self.data_speed_gate()):
            return
        
        linear = msg.linear
        angular = msg.angular
        
        self.csv_writer.writerow([datetime.now().strftime('%Y-%m-%d %H:%M:%S'), linear.x, linear.y, linear.z, angular.x, angular.y, angular.z])
        print("==> cmd_vel")

    def odom_callback(self, msg):
        if(not self.data_speed_gate()):
            return

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.csv_writer.writerow([datetime.now().strftime('%Y-%m-%d %H:%M:%S'), position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w])
        print("==> odom")

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