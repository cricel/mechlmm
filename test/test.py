import rclpy
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped, Pose

import threading

from mechllm_core import MechLLMCore
from geometry_msgs.msg import Twist

class RobotPose(Node):
    def __init__(self):
        super().__init__('robot_pose_listener')
        self.mechllm_core = MechLLMCore()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.lock = threading.Lock()
        self.is_llm_processing = False

        self.current_pose = Pose()
        self.target_pose = Pose()

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

    def timer_callback(self):
        try:
            

            # Get the transform from 'map' to 'base_link' (robot's frame)
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', now, timeout=rclpy.duration.Duration(seconds=1.0)
            )
            # Extract the position and orientation
            position = transform.transform.translation
            orientation = transform.transform.rotation

            self.current_pose.position.x = position.x
            self.current_pose.position.y = position.y
            self.current_pose.position.z = position.z
            self.current_pose.orientation = orientation
            # Convert quaternion to Euler angles

            print("-111--")
            print(self.current_pose.position.x)
            print("=111==")
            
            orientation_euler = euler_from_quaternion([
                orientation.x, orientation.y, orientation.z, orientation.w
            ])

            # Print out the current robot position and orientation
            self.get_logger().info(f"Robot position: x={position.x}, y={position.y}, z={position.z}")
            self.get_logger().info(f"Robot orientation (yaw): {orientation_euler[2]}")

            thread = threading.Thread(target=self.run_with_lock)
            thread.start()

        except Exception as e:
            self.get_logger().error(f"Could not get transform: {str(e)}")

    def run_with_lock(self):
        with self.lock:
            self.pose_checker()

    def add(self):
        print("-> multiply")

    def multiply(self):
        print("-> multiply")

    def move_base(self, target_direction: str):
        print(target_direction)
        if(target_direction == "forward"):
            msg = Twist()
            msg.linear.x = 0.1
            self.publisher_.publish(msg)
        elif(target_direction == "left"):
            msg = Twist()
            msg.angular.z = -0.1
            self.publisher_.publish(msg)
        elif(target_direction == "right"):
            msg = Twist()
            msg.angular.z = 0.1
            self.publisher_.publish(msg)
        elif(target_direction == "back"):
            msg = Twist()
            msg.linear.x = -0.1
            self.publisher_.publish(msg)

        elif(target_direction == "stop"):
            msg = Twist()
            msg.linear.x = 0.0
            self.publisher_.publish(msg)

        print("-> move_base")

    def move_arm_end_effector(self, target_location: str):
        print("-> move_arm_end_effector")
    
    def arrived_stop(self):
        print("-> arrived_stop")


    def pose_checker(self):
        self.target_pose.position.x = 1.1
        self.target_pose.position.y = -1.0
        self.target_pose.orientation.z = -0.15
        self.target_pose.orientation.w = 0.98

        print("---")
        print(self.current_pose.position.x)
        print("===")

        current_pose_dict = {
            "position": {
                "x": self.current_pose.position.x,
                "y": self.current_pose.position.y,
                "z": self.current_pose.position.z
            },
            "orientation": {
                "x": self.current_pose.orientation.x,
                "y": self.current_pose.orientation.y,
                "z": self.current_pose.orientation.z,
                "w": self.current_pose.orientation.w
            }
        }
        
        target_pose_dict = {
            "position": {
                "x": self.target_pose.position.x,
                "y": self.target_pose.position.y,
                "z": self.target_pose.position.z
            },
            "orientation": {
                "x": self.target_pose.orientation.x,
                "y": self.target_pose.orientation.y,
                "z": self.target_pose.orientation.z,
                "w": self.target_pose.orientation.w
            }
        }

        print("--+++-")
        print (current_pose_dict)
        print (target_pose_dict)
        print("--+++-")

        _query = f"""
                    given the data below, the "arm_end_effector" is attached to the "robot_base" and the position is relative position to the "robot_base". 

                    if I want to grab the "target_obj", what would the sequence of function call, you can call single function multiple time.
                    and you can stop the robot when object is within 1cm

                    current robot pose: {current_pose_dict}

                    target robot pose: {target_pose_dict}
                """
        print(_query)
        _result = self.mechllm_core.chat_tool(_query)
        print("______")
        print(_result.tool_calls)

        try:
            tool_call = _result.tool_calls[0]
            print(tool_call)
            selected_tool = {
                    "add": self.add, "multiply": self.multiply, "move_base": self.move_base, "move_arm_end_effector": self.move_arm_end_effector
                }[tool_call["name"].lower()]

            print(selected_tool)
            # selected_tool.invoke(tool_call["args"])

            print("--")
            
            selected_tool(tool_call["args"]["target_direction"])
        except:
            print("hahah")
        # function_list = []
        # function_arg_list = []
        # for tool_call in _result.tool_calls:
        #     selected_tool = {"add": self.add, "multiply": self.multiply, "move_base": self.move_base, "move_arm_end_effector": self.move_arm_end_effector, "arrived_stop": self.arrived_stop}[tool_call["name"].lower()]
        #     function_list.append(selected_tool)
        #     function_arg_list.append(tool_call["args"])
        #     # tool_output = selected_tool.invoke(tool_call["args"])
        #     # print(ToolMessage(tool_output, tool_call_id=tool_call["id"]))
        #     # print("---")

        # print(function_list)
        # print(function_arg_list)

    #     return function_list, function_arg_list
    # ))

def main(args=None):
    rclpy.init(args=args)
    robot_pose = RobotPose()
    rclpy.spin(robot_pose)
    robot_pose.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
