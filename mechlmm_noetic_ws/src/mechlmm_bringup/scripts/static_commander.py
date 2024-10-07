#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class RedColorDetector:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('red_color_detector', anonymous=True)

        # Initialize a CvBridge object to convert between ROS and OpenCV images
        self.bridge = CvBridge()

        # Subscribe to the image topic (e.g., from a camera)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)

    def image_callback(self, msg):
        # Convert the ROS Image message to a format OpenCV understands
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convert the image to HSV (hue, saturation, value)
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the red color range in HSV
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])

        # Create a mask for detecting red color
        mask = cv2.inRange(hsv_image, lower_red, upper_red)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw bounding boxes around the detected red areas
        for contour in contours:
            # Get the bounding box coordinates for the contour
            x, y, w, h = cv2.boundingRect(contour)

            # Draw the bounding box on the original image
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Green box, 2px thick

        # Display the result
        cv2.imshow("Red Color Detection with Bounding Boxes", cv_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        # Create the RedColorDetector object
        red_color_detector = RedColorDetector()

        # Keep the node alive
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
