#!/usr/bin/env python

import rospy
import moveit_commander
import sys
from geometry_msgs.msg import Pose

def is_point_within_reach(group, point):
    # Create a Pose from the point (assuming it's in the base_link frame)
    pose = Pose()
    pose.position.x = point[0]
    pose.position.y = point[1]
    pose.position.z = point[2]
    pose.orientation.w = 1.0  # Neutral orientation (no rotation)
    
    # Set the target pose
    group.set_pose_target(pose)
    
    # Plan to that pose
    plan, planning_time = group.plan()  # Unpack the plan and planning time

    # Check if a valid plan was found
    if plan and len(plan.joint_trajectory.points) > 0:
        rospy.loginfo("The point is within reach!")
        return plan
    else:
        rospy.logwarn("The point is not reachable.")
        return None

def get_trajectory_joint_positions(plan):
    joint_trajectory = plan.joint_trajectory

    # Iterate through each point in the trajectory
    joint_positions_list = []
    for point in joint_trajectory.points:
        joint_positions = point.positions  # The joint positions at this point in time
        joint_positions_list.append(joint_positions)
    
    return joint_positions_list

def main():
    rospy.init_node('check_reachability', anonymous=True)

    # Initialize the move_group commander
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    
    # Specify the name of the group of joints you want to plan for
    group_name = "arm"  # Adjust the group name according to your setup
    group = moveit_commander.MoveGroupCommander(group_name)

    # Define the point you want to check (x, y, z)
    point = [-1.52, 0.01, 0.00]  # Example point
    
    # Check if the point is within reach and get the plan
    plan = is_point_within_reach(group, point)

    if plan:
        rospy.loginfo("The robot can reach the point.")
        # Get the joint positions at each point in the motion trajectory
        joint_positions_list = get_trajectory_joint_positions(plan)

        # Log the joint positions for each step in the trajectory
        for i, joint_positions in enumerate(joint_positions_list):
            rospy.loginfo("Step {}: Joint positions: {}".format(i, joint_positions))
    else:
        rospy.logwarn("The point is out of reach.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
