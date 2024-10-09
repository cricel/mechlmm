#!/usr/bin/env python

import sys
import time
import rospy
import cv2

import threading

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from langchain_core.utils.function_calling import convert_to_openai_function
from typing import Optional, List
from pydantic import BaseModel, Field
import requests

from mechlmm_py import utilities_core, lmm_function_pool, VisionCore, DebugCore
import function_pool_lmm_declaration
from function_pool_definition import FunctionPoolDefinition

class image_converter:
    def __init__(self):
        self.vision_core = VisionCore()
        self.debug_core = DebugCore()
        self.debug_core.verbose = 3

        self.function_pool_definition = FunctionPoolDefinition()

        self.bridge = CvBridge()
        self.hand_image_sub = rospy.Subscriber("/camera/hand/image_raw",Image,self.hand_callback)
        self.head_image_sub = rospy.Subscriber("/camera/head/image_raw",Image,self.head_callback)
        self.base_image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.base_callback)

        # self.lmm_command_sub = rospy.Subscriber("/lmm/command",String, self.lmm_command_callback)
        self.lmm_command_pub = rospy.Publisher('/lmm/command', String, queue_size=10)

        rospy.Timer(rospy.Duration(1), self.timer_callback)

        self.img_query = False

        self.head_cam = None
        self.hand_cam = None
        self.base_cam = None

        self.llm_tools_map = {
            "arm_end_effector_control": self.function_pool_definition.arm_end_effector_control,
            "move_robot": self.function_pool_definition.move_robot,
            "trigger_gripper": self.function_pool_definition.trigger_gripper,
        }

        self.lock = threading.Lock()
        self.processing_thread = threading.Thread(target=self.process_frames)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        self.lmm_result = None
        
    def timer_callback(self, msg):
        # self.llm_chat_image()
        pass

    def llm_chat_image(self):
        if(not self.img_query):
            self.img_query = True

            print("----")
            if (self.head_cam is None or self.hand_cam is None or self.base_cam is None):
                return

            url = 'http://192.168.1.134:5001/mechlmm/chat'
            
            hand_image_url = utilities_core.opencv_frame_to_base64(self.hand_cam)
            head_image_url = utilities_core.opencv_frame_to_base64(self.head_cam)
            base_image_url = utilities_core.opencv_frame_to_base64(self.base_cam)

            query = """
                    Base on the information and image provided, what are the list of action should do to use end effector to grab the coke can

                    This is a 2 wheel driving robot with a 34 dof arm attach to the top
                    the image provided in order is:
                    1. hand camera which attach to the end effector
                    2. head camera which alway look at the entire robot view from top left angle
                    3. base camera which mount at the base on the robot
                """

            data = {
                'question': query,
                # 'schema': dict_schema,
                'tag': 'head_callback',
                'base_img': [hand_image_url, head_image_url, base_image_url],
                'tools': [convert_to_openai_function(function_pool_lmm_declaration.arm_end_effector_control),
                        #   convert_to_openai_function(function_pool_lmm_declaration.arm_end_effector_rotation_control),
                          convert_to_openai_function(function_pool_lmm_declaration.trigger_gripper),
                          convert_to_openai_function(function_pool_lmm_declaration.move_robot)
                          ]
            }

            response = requests.post(url, json=data)

            if response.status_code == 200:
                _result = response.json()
                print('Success: \n', _result)
                if(_result['type'] == 'tools'):
                    for func in _result['result']:
                        selected_tool = self.llm_tools_map[func['name'].lower()]
                        selected_tool(func['args'])

                    self.lmm_command_pub.publish("llm send")
            else:
                print('Failed:', response.status_code, response.text)

            self.img_query = False

    def hand_callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.hand_cam = cv_image

    def head_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.head_cam = cv_image
    
    def base_callback(self, data):
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

def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)