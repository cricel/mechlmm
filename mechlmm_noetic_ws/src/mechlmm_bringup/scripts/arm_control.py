#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

planning_frame = move_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

# print("============ Printing robot state")
# print(robot.get_current_state())
# print("")

pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.x = -0.5207246178761125e-05
pose_goal.orientation.y = -0.01998324692249298
pose_goal.orientation.z = -0.0007608475862070918
pose_goal.orientation.w = 0.9998000264167786
pose_goal.position.x = 0.2517150938510895
pose_goal.position.y = -0.0005048706661909819
pose_goal.position.z = 0.28646329045295715

# move_group.set_pose_target(pose_goal)
# plan = move_group.plan()
# print(plan[0])

# joint_trajectory = plan[1].joint_trajectory.points
# positions_list = [point.positions for point in joint_trajectory]
# print(positions_list)

# joint_goal = move_group.get_current_joint_values()
# joint_goal = positions_list[-1]
# move_group.go(joint_goal, wait=True)
# move_group.stop()


waypoints = []
current_pose = move_group.get_current_pose().pose
waypoints.append(current_pose)
waypoints.append(pose_goal)

max_step = 0.01
jump_threshold = 0.0
(plan, fraction) = move_group.compute_cartesian_path(waypoints, max_step, jump_threshold)



# success = move_group.go(wait=True)
# move_group.stop()
# move_group.clear_pose_targets()