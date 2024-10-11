#!/usr/bin/env python

import rospy
import actionlib
import tf

import cv2
import numpy as np

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class RedColorDetector:
    def __init__(self):
        rospy.init_node('red_color_detector', anonymous=True)

        self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)

        self.tf_listener = tf.TransformListener()

        self.test_point = Point()
        self.test_point.x = -1.5409061375463193
        self.test_point.y = -0.5796078003258718
        self.test_point.z = 0.0

        # distance = self.distance_checker(self.test_point)
        # print(distance)
        # self.nav_to_point(self.test_point.x, self.test_point.y, 0.0)

    def distance_3d(self, p1, p2):
        return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2 + (p2[2] - p1[2])**2)

    def distance_2d(self, p1, p2):
        return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

    def point_at_distance_3d(self, p1, p2, distance):
        direction = np.array(p2) - np.array(p1)
        direction_norm = direction / np.linalg.norm(direction)
        return p2 + distance * direction_norm

    def point_at_distance_2d(p1, p2, distance):
        direction = np.array([p2[0] - p1[0], p2[1] - p1[1]])
        direction_norm = direction / np.linalg.norm(direction)
        new_point = np.array([p2[0], p2[1]]) + distance * direction_norm
        return new_point[0], new_point[1]
    
    def distance_checker(self, target_point):
        self.tf_listener.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(5.0))
        (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
        return self.distance_2d([trans[0], trans[1], trans[2]], [target_point.x, target_point.y, target_point.z])
    
    def nav_to_point(self, x_goal, y_goal, yaw_goal):
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")
        client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")

        goal = MoveBaseGoal()
        
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = Point(x_goal, y_goal, 0.0)
        quaternion = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, yaw_goal))
        goal.target_pose.pose.orientation = quaternion

        rospy.loginfo("Sending goal: x=%.2f, y=%.2f, yaw=%.2f", x_goal, y_goal, yaw_goal)
        client.send_goal(goal)

        client.wait_for_result()

        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached successfully!")
            return True
        else:
            rospy.logwarn("Failed to reach goal")
            return False
            
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv_image, lower_red, upper_red)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

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
