import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

# We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(robot.get_current_state())
print("")

pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.x = -0.5207246178761125e-05
pose_goal.orientation.y = -0.01998324692249298
pose_goal.orientation.z = -0.0007608475862070918
pose_goal.orientation.w = 0.9998000264167786
pose_goal.position.x = 0.2517150938510895
pose_goal.position.y = -0.0005048706661909819
pose_goal.position.z = 10.28646329045295715

move_group.set_pose_target(pose_goal)
plan = move_group.plan()
# print(move_group)
# print(type(move_group.plan()))
# print(move_group.plan())
# print("------")
# print(move_group.plan()[0])
# print(move_group.plan()[1])
print("------===")
print(plan[0])
joint_trajectory = plan[1].joint_trajectory.points
# print(joint_trajectory)
# joint_trajectory = move_group.plan()[1]["joint_trajectory"]
# print(joint_trajectory)
positions_list = [point.positions for point in joint_trajectory]

print(positions_list)

# `go()` returns a boolean indicating whether the planning and execution was successful.
# success = move_group.go(wait=True)
# # Calling `stop()` ensures that there is no residual movement
# move_group.stop()
# # It is always good to clear your targets after planning with poses.
# # Note: there is no equivalent function for clear_joint_value_targets().
# move_group.clear_pose_targets()