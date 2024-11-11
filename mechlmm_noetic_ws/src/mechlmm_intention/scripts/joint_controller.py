#!/usr/bin/env python

import sys
import numpy as np

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

from intention_core import IntentionCore

class UnityConnector:
    def __init__(self):
        self.skeleton_joints_sub = rospy.Subscriber("skeleton_joints", Float32MultiArray, self.skeleton_joints_callback)
        
        self.recording_trigger_sub = rospy.Subscriber("recording_trigger", Bool, self.recording_trigger_callback)
        self.recording_trigger_pub = rospy.Publisher('recording_trigger', Bool, queue_size=10)

        self.training_info_sub = rospy.Subscriber("training_info", String, self.training_info_callback)
        self.training_info_pub = rospy.Publisher('training_info', String, queue_size=10)

        self.is_recording = False
        self.trainning_counter = 0

        self.training_info_text = []

        self.it = IntentionCore()
        # self.it.folder_creation()

    def skeleton_joints_callback(self, _msg):
        # if(self.is_recording):
        #     try:
        #         my_array = np.array(_msg.data)
        #         if(my_array.size != 0):
        #             self.it.add_trainning_data(my_array, 
        #                                     self.training_info_text[0], 
        #                                     self.training_info_text[1], 
        #                                     str(self.trainning_counter))
                    
        #             print(self.trainning_counter)
        #             self.trainning_counter += 1
        #             if(self.trainning_counter >= 30):
        #                 self.trainning_counter = 0
        #                 self.is_recording = False
        #                 self.recording_trigger_pub.publish(self.is_recording)
        #     except Exception as e:
        #         print("An error occurred:", e)

        my_array = np.array(_msg.data)
        result_msg = self.it.live_prediction(my_array)
        if(result_msg != ""):
            self.training_info_pub.publish(result_msg)


    def recording_trigger_callback(self, _msg):
        self.is_recording = _msg.data

    def training_info_callback(self, _msg):
        self.training_info_text = _msg.data.split(",")
        
def main(args):
    uc = UnityConnector()

    rospy.init_node('unity_connector', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)