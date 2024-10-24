#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import requests
import numpy as np

class CameraPublisher:
    def __init__(self, url):
        self.url = url
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('camera/image_raw', Image, queue_size=10)
        rospy.init_node('camera_publisher', anonymous=True)

    def run(self):
        while not rospy.is_shutdown():
            try:
                response = requests.get(self.url, stream=True)
                bytes_data = b""
                for chunk in response.iter_content(chunk_size=1024):
                    bytes_data += chunk
                    a = bytes_data.find(b'\xff\xd8')  # Start of a JPEG frame
                    b = bytes_data.find(b'\xff\xd9')  # End of a JPEG frame
                    if a != -1 and b != -1:
                        jpg = bytes_data[a:b + 2]
                        bytes_data = bytes_data[b + 2:]

                        # Convert JPEG image to a format suitable for ROS
                        frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")

                        # Publish the image
                        self.pub.publish(ros_image)

            except Exception as e:
                rospy.logerr("Error: %s" % str(e))
                break

if __name__ == '__main__':
    try:
        camera_url = "http://localhost:5000/video_feed"  # URL of the camera stream
        camera_pub = CameraPublisher(camera_url)
        camera_pub.run()
    except rospy.ROSInterruptException:
        pass
