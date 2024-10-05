#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Pose

def get_robot_position():
    rospy.init_node('get_robot_position', anonymous=True)
    listener = tf.TransformListener()

    try:
        listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))

        rospy.loginfo("Robot position: x = {}, y = {}, z = {}".format(trans[0], trans[1], trans[2]))
        rospy.loginfo("Robot orientation: x = {}, y = {}, z = {}, w = {}".format(rot[0], rot[1], rot[2], rot[3]))

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("Could not get robot position")

if __name__ == '__main__':
    try:
        get_robot_position()
    except rospy.ROSInterruptException:
        pass
