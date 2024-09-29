import time

# generic ros libraries
import rclpy
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)

# class MoveEndEffectorUp(Node):
#     def __init__(self):
#         super().__init__('move_end_effector_up')
        
#         # Initialize the MoveIt commander for the specific move group (e.g., 'manipulator')
#         self.move_group = MoveGroupCommander('arm')

#         # Get the current pose of the end effector
#         current_pose = self.move_group.get_current_pose().pose
        
#         # Modify the pose to move 1 cm (0.01 meters) upward in the Z direction
#         target_pose = Pose()
#         target_pose.position.x = current_pose.position.x
#         target_pose.position.y = current_pose.position.y
#         target_pose.position.z = current_pose.position.z + 0.01  # 1 cm up
#         target_pose.orientation = current_pose.orientation  # Keep the orientation the same
        
#         # Plan and execute the motion
#         self.move_group.set_pose_target(target_pose)
#         success = self.move_group.go(wait=True)

#         # Stop the robot after motion and clear targets
#         self.move_group.stop()
#         self.move_group.clear_pose_targets()

#         if success:
#             self.get_logger().info('End effector moved 1 cm up!')
#         else:
#             self.get_logger().error('Failed to move end effector.')

def main(args=None):
    rclpy.init()
    logger = rclpy.logging.get_logger("moveit_py.pose_goal")

    # instantiate MoveItPy instance and get planning component
    panda = MoveItPy(node_name="moveit_py")
    panda_arm = panda.get_planning_component("panda_arm")
    logger.info("MoveItPy instance created")

if __name__ == '__main__':
    main()
