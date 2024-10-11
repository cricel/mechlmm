#!/usr/bin/env python

### quick end effotor pos check up
# [INFO] [1728297113.791033, 96.706000]: End Effector Pose: header: 
#   seq: 0
#   stamp: 
#     secs: 0
#     nsecs:         0
#   frame_id: "map"
# pose: 
#   position: 
#     x: -1.2409061375463193
#     y: -0.5796078003258718
#     z: 0.2574291294229187
#   orientation: 
#     x: 0.00045537707141373287
#     y: -0.007999632933302858

import rospy
import tf
from geometry_msgs.msg import PointStamped, Pose, PoseStamped

import moveit_commander
import sys

def get_end_effector_pose():
    rospy.init_node('end_effector_position_node')

    # Create a TF listener to get transformations
    listener = tf.TransformListener()
    
    moveit_commander.roscpp_initialize(sys.argv)

    moveit_robot = moveit_commander.RobotCommander()
    moveit_scene = moveit_commander.PlanningSceneInterface()
    moveit_group = moveit_commander.MoveGroupCommander("arm")  

    # Give some time for the listener to accumulate data
    rospy.sleep(1.0)

    try:
        listener.waitForTransform("map", "link1", rospy.Time(), rospy.Duration(4.0))

        # # Lookup transform from 'map' frame to 'end_effector' frame
        # (trans, rot) = listener.lookupTransform('map', 'end_effector_link', rospy.Time(0))

        # print("Translation: ", trans)
        # print("Rotation: ", rot)

        # # You can convert the position and orientation into a PoseStamped message if needed
        # pose = PoseStamped()
        # pose.header.frame_id = "map"
        # pose.pose.position.x = trans[0]
        # pose.pose.position.y = trans[1]
        # pose.pose.position.z = trans[2]
        # pose.pose.orientation.x = rot[0]
        # pose.pose.orientation.y = rot[1]
        # pose.pose.orientation.z = rot[2]
        # pose.pose.orientation.w = rot[3]

        object_position_in_map = PoseStamped()
        object_position_in_map.header.frame_id = "map"
        object_position_in_map.pose.orientation.x = 0.0
        object_position_in_map.pose.orientation.y = 0.0
        object_position_in_map.pose.orientation.z = 0.0
        object_position_in_map.pose.orientation.w = 1.0
        object_position_in_map.pose.position.x = -1.3509061375463193
        object_position_in_map.pose.position.y = -0.5796078003258718
        object_position_in_map.pose.position.z = 0.3574291294229187

        # object_position_in_link1 = listener.transformPoint('link1', object_position_in_map)
        transformed_pose = listener.transformPose("link1", object_position_in_map)

        # (arm_trans, arm_rot) = listener.lookupTransform('link1', 'end_effector_link', rospy.Time(0))

        # print("Translation: ", arm_trans)
        # print("Rotation: ", arm_rot)

        print(transformed_pose)

        moveit_group.set_position_target([transformed_pose.pose.position.x, 
                                          transformed_pose.pose.position.y, 
                                          transformed_pose.pose.position.z])
        plan1 = moveit_group.plan()

        plan = moveit_group.go(wait=True)
        moveit_group.stop()
        moveit_group.clear_pose_targets()
        moveit_commander.roscpp_shutdown()

        # return pose

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("Could not find transform between 'map' and 'end_effector'")
        return None

if __name__ == '__main__':
    try:
        get_end_effector_pose()
        # if pose:
        #     rospy.loginfo("End Effector Pose: %s", pose)
    except rospy.ROSInterruptException:
        pass
