#!/usr/bin/env python

import rospy
import tf

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

import sys
import time
from datetime import datetime
import json

from mechlmm_py import utilities_core, PostgresCore

class DataCommander:
    def __init__(self):
        rospy.init_node('data_commander', anonymous=True)

        self.postgres_core = PostgresCore()

        self.last_saved_time = time.time()
        self.data_collection_duration = 0.2

        self.cmd_sub = rospy.Subscriber("/cmd_vel",Twist, self.cmd_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Timer(rospy.Duration(1.0), self.timer_callback)
        
        self.tf_listener = tf.TransformListener()
        
    def data_speed_gate(self):
        current_time = time.time()
        if (current_time - self.last_saved_time >= self.data_collection_duration):
            self.last_saved_time = current_time
            return True
        return False

    def cmd_callback(self, _msg):
        if(not self.data_speed_gate()):
            return

        msg_dict = utilities_core.ros_message_to_dict(_msg)
        json_str = json.dumps(msg_dict)
        
        self.postgres_core.post_data_log_db(datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                                            "robot_speed",
                                            json_str
                                            )

    def timer_callback(self, event):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
 
            roll, pitch, yaw = tf.transformations.quaternion_to_euler(rot)
            msg_dict = {
                "position_x": trans[0],
                "position_y": trans[1],
                "position_z": trans[2],
                "rotation_x": roll,
                "rotation_y": pitch,
                "rotation_z": yaw,
            }

            msg_dict = utilities_core.ros_message_to_dict(msg_dict)
            json_str = json.dumps(msg_dict)
            
            print(json_str)
            self.postgres_core.post_data_log_db(datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                                                "robot_pose",
                                                json_str
                                                )
        except:
            pass

    def odom_callback(self, _msg):
        if(not self.data_speed_gate()):
            return

        msg_dict = utilities_core.ros_message_to_dict(_msg)

        msg_dict = {
            "linear_speed_x": _msg.twist.twist.linear.x,
            "linear_speed_y": _msg.twist.twist.linear.y,
            "linear_speed_z": _msg.twist.twist.linear.z,
            "angular_speed_x": _msg.twist.twist.angular.x,
            "angular_speed_y": _msg.twist.twist.angular.y,
            "angular_speed_z": _msg.twist.twist.angular.z
        }

        json_str = json.dumps(msg_dict)
        
        self.postgres_core.post_data_log_db(datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                                            "robot_speed",
                                            json_str
                                            )


    def run(self):
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Shutting down data commander.")

if __name__ == '__main__':
    try:
        node = DataCommander()

        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.shutdown()