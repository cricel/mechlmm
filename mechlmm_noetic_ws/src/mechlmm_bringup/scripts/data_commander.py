#!/usr/bin/env python

import rospy
import tf
# from tf.transformations import quaternion_from_euler, euler_from_quaternion
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

import sys
import time
from datetime import datetime
import json
import threading

from mechlmm_py import utilities_core, PostgresCore, VisionCore, DebugCore

class DataCommander:
    def __init__(self):
        rospy.init_node('data_commander', anonymous=True)

        self.vision_core = VisionCore()
        self.debug_core = DebugCore()
        self.debug_core.verbose = 3
        self.postgres_core = PostgresCore()

        # self.last_saved_time = time.time()
        self.last_saved_time = {}
        self.data_collection_duration = 0.2

        self.bridge = CvBridge()
        
        self.cmd_sub = rospy.Subscriber("/cmd_vel",Twist, self.cmd_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.base_image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.base_image_callback)

        rospy.Timer(rospy.Duration(0.2), self.timer_callback)
        
        self.tf_listener = tf.TransformListener()

        self.lock = threading.Lock()
        self.processing_thread = threading.Thread(target=self.process_frames)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        self.lmm_result = None
        
    def data_speed_gate(self, _callback_name, _frequency):
        current_time = time.time()
        last_time = self.last_saved_time.get(_callback_name, 0)
        if (current_time - last_time >= _frequency):
            self.last_saved_time[_callback_name] = current_time
            return True
        return False
        
    def cmd_callback(self, _msg):
        if(not self.data_speed_gate("cmd", 0.2)):
            return

        msg_dict = utilities_core.ros_message_to_dict(_msg)
        json_str = json.dumps(msg_dict)
        
        self.postgres_core.post_data_log_db(datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                                            "robot_control_input",
                                            json_str
                                            )

    def timer_callback(self, event):
        # try:
        if(not self.data_speed_gate("pose", 1.0)):
            return
        
        self.tf_listener.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(4.0))

        (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))

        roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
        msg_dict = {
            "position_x": trans[0],
            "position_y": trans[1],
            "position_z": trans[2],
            "rotation_x": roll,
            "rotation_y": pitch,
            "rotation_z": yaw,
        }
        json_str = json.dumps(msg_dict)
        
        print(json_str)
        self.postgres_core.post_data_log_db(datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                                            "robot_pose",
                                            json_str
                                            )

    def odom_callback(self, _msg):
        if(not self.data_speed_gate("odom", 1.0)):
            return

        print("odom ==>")
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

    def base_image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            self.vision_core.frame_height, self.vision_core.frame_width, channels = cv_image.shape

            self.vision_core.video_saver(cv_image)

            with self.lock:
                self.latest_frame = cv_image.copy()

        except CvBridgeError as e:
            print(e)

        self.base_cam = cv_image

    def process_frames(self):
        while True:
            with self.lock:
                if hasattr(self, 'latest_frame'):
                    frame = self.latest_frame
                else:
                    frame = None

            if frame is not None:
                self.lmm_result = self.vision_core.frame_analyzer(frame)
            
            time.sleep(0.1)

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