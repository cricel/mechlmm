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

rclpy.init()
logger = rclpy.logging.get_logger("moveit_py.pose_goal")

# instantiate MoveItPy instance and get planning component
panda = MoveItPy(node_name="moveit_py")
panda_arm = panda.get_planning_component("arm")
logger.info("MoveItPy instance created")