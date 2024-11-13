#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import requests
import numpy as np

from ultralytics import YOLO
import math

class CameraPublisher:
    def __init__(self, url):
        self.url = url
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('camera/image_raw', Image, queue_size=10)
        rospy.init_node('camera_publisher', anonymous=True)

        self.yolo = YOLO('yolov8s.pt')

    def getColours(self, cls_num):
        base_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
        color_index = cls_num % len(base_colors)
        increments = [(1, -2, 1), (-2, 1, -1), (1, -1, 2)]
        color = [base_colors[color_index][i] + increments[color_index][i] * 
        (cls_num // len(base_colors)) % 256 for i in range(3)]
        return tuple(color)

    def is_point_inside_rectangle(self, px, py, x1, y1, x2, y2):
        return x1 <= px <= x2 and y1 <= py <= y2

    def get_center_of_bounding_box(self, x1, y1, x2, y2):
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        return int(cx), int(cy)

    def distance_to_nearest_box_point(self, px, py, x1, y1, x2, y2):
        # If the point is inside, the distance is 0
        if self.is_point_inside_rectangle(px, py, x1, y1, x2, y2):
            return 0

        # Calculate the horizontal distance to the nearest x boundary
        if px < x1:
            nearest_x = x1
        elif px > x2:
            nearest_x = x2
        else:
            nearest_x = px  # Point is between x1 and x2

        # Calculate the vertical distance to the nearest y boundary
        if py < y1:
            nearest_y = y1
        elif py > y2:
            nearest_y = y2
        else:
            nearest_y = py  # Point is between y1 and y2

        # Return the Euclidean distance to the nearest point on the box
        distance = math.sqrt((px - nearest_x) ** 2 + (py - nearest_y) ** 2)
        return distance


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


                        results = self.yolo.track(frame, stream=True)

                        for result in results:
                            # get the classes names
                            classes_names = result.names

                            # iterate over each box
                            for box in result.boxes:
                                # check if confidence is greater than 40 percent
                                if box.conf[0] > 0.4:
                                    # get coordinates
                                    [x1, y1, x2, y2] = box.xyxy[0]
                                    # convert to int
                                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                                    # get the class
                                    cls = int(box.cls[0])
                                    # get the class name
                                    class_name = classes_names[cls]
                                    # get the respective colour
                                    colour = self.getColours(cls)
                                    # draw the rectangle
                                    cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
                                    cv2.putText(frame, f'{classes_names[int(box.cls[0])]} {box.conf[0]:.2f}', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, colour, 2)
                                    
                                    if(classes_names[int(box.cls[0])] == "cup"):
                                        object_center = self.get_center_of_bounding_box(x1, y1, x2, y2)
                                        cv2.line(frame, (0,0), object_center, (0, 255, 0), 2)
                                        dis = self.distance_to_nearest_box_point(0, 0, x1, y1, x2, y2)
                                        cv2.putText(frame, str(dis), (x1 + 3, y1+30), cv2.FONT_HERSHEY_SIMPLEX, 1, colour, 2)
                                        


                        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                        # Publish the image
                        self.pub.publish(ros_image)

            except Exception as e:
                rospy.logerr("Error: %s" % str(e))
                break

if __name__ == '__main__':
    try:
        camera_url = "http://192.168.1.182:5000/video_feed"  # URL of the camera stream
        camera_pub = CameraPublisher(camera_url)
        camera_pub.run()
    except rospy.ROSInterruptException:
        pass
