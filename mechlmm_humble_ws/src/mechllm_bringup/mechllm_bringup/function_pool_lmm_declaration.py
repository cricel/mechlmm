from pydantic import BaseModel, Field

class navigation(BaseModel):
    """Navigate to the target location"""

    target_name: str = Field(..., description="the name of the target location")

class manipulation(BaseModel):
    """Manipulate the target object"""

    target_name: str = Field(..., description="the name of the target object")

class move_robot(BaseModel):
    """move robot to the target direction for certain distance"""

    direction: str = Field(..., description="the direction of robot is moving, the value can only be: forward, backward, turn_left, turn_right")

class trigger_gripper(BaseModel):
    """control gripper open or close"""

    trigger: bool = Field(..., description="trigger is use to control the gripper open or close, true means open gripper, false means close gripper")

class arm_end_effector_rotation_control(BaseModel):
    """control only the end effector rotation of the robot arm, it can only rotate up and down"""

    rotation: float = Field(..., description="the roattion of robot end effector is rotating, the radian value is range from -1.0472 to 1.0472")

class arm_end_effector_control(BaseModel):
    """move the entire arm by control the end effector position of the robot arm to different direction"""

    direction: str = Field(..., description="the direction of robot arm is moving, the value can only be: forward, backward, up, down, turn_left, turn_right")