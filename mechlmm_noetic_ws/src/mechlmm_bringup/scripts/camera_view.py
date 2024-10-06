#!/usr/bin/env python

import sys
import rospy
import cv2

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from langchain_core.utils.function_calling import convert_to_openai_function
from typing import Optional, List
from pydantic import BaseModel, Field
import requests

from mechlmm_py import utilities_core, lmm_function_pool

class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/hand/image_raw",Image,self.hand_callback)
        self.image_sub = rospy.Subscriber("/camera/head/image_raw",Image,self.head_callback)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.base_callback)


        self.hand_cam = False
        self.head_cam = False
        self.base_cam = False

    def hand_callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if(not self.hand_cam):
            self.hand_cam = True

            print("hand_callback --")

            image_url = utilities_core.opencv_frame_to_base64(cv_image)

            url = 'http://192.168.1.134:5001/mechlmm/chat/image'

            # dict_schema = convert_to_openai_function(ListItems)

            # query = """
            #         you are a AI on a 2 wheel robot with a single arm attach to it.
            #         and this a camera on the end effector of the arm
            #         in order to grab the coke can, what action should be taken
            #     """
            
            data = {
                'question': """
                    did you see a coke can in the image
                """,
                # 'schema': dict_schema,
                'tag': 'hand_callback',
                'base_img': image_url
            }

            response = requests.post(url, json=data)

            if response.status_code == 200:
                _result = response.json()
                print('Success: \n', _result)
            else:
                print('Failed:', response.status_code, response.text)
            
            print("hand_callback --")
            self.hand_cam = False

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)


    def llm_chat_image(_frame, _tag):
        url = 'http://192.168.1.134:5001/mechlmm/chat/image'
        
        image_url = utilities_core.opencv_frame_to_base64(_frame)

        data = {
            'question': """
                did you see a coke can in the image
            """,
            # 'schema': dict_schema,
            'tag': 'head_callback',
            'base_img': image_url
        }

        response = requests.post(url, json=data)

        if response.status_code == 200:
            _result = response.json()
            print('Success: \n', _result)
        else:
            print('Failed:', response.status_code, response.text)
        


    def head_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if(not self.hand_cam):
            self.hand_cam = True

            print("head_callback --")

            image_url = utilities_core.opencv_frame_to_base64(cv_image)

            url = 'http://192.168.1.134:5001/mechlmm/chat/image'

            # dict_schema = convert_to_openai_function(ListItems)

            # query = """
            #         you are a AI on a 2 wheel robot with a single arm attach to it.
            #         and this a camera on the end effector of the arm
            #         in order to grab the coke can, what action should be taken
            #     """
            
            data = {
                'question': """
                    did you see a coke can in the image
                """,
                # 'schema': dict_schema,
                'tag': 'head_callback',
                'base_img': image_url
            }

            response = requests.post(url, json=data)

            if response.status_code == 200:
                _result = response.json()
                print('Success: \n', _result)
            else:
                print('Failed:', response.status_code, response.text)
            
            print("head_callback --")
            self.hand_cam = False

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
    
    def base_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if(not self.hand_cam):
            self.hand_cam = True

            print("base_callback --")

            image_url = utilities_core.opencv_frame_to_base64(cv_image)

            url = 'http://192.168.1.134:5001/mechlmm/chat/image'

            # dict_schema = convert_to_openai_function(ListItems)

            # query = """
            #         you are a AI on a 2 wheel robot with a single arm attach to it.
            #         and this a camera on the end effector of the arm
            #         in order to grab the coke can, what action should be taken
            #     """
            
            data = {
                'question': """
                    did you see a coke can in the image
                """,
                # 'schema': dict_schema,
                'tag': 'base_callback',
                'base_img': image_url
            }

            response = requests.post(url, json=data)

            if response.status_code == 200:
                _result = response.json()
                print('Success: \n', _result)
            else:
                print('Failed:', response.status_code, response.text)
            
            print("base_callback --")
            self.hand_cam = False

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)


def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)