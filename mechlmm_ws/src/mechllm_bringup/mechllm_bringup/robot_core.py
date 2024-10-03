import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from tf_transformations import quaternion_from_euler
import tf_transformations

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

from mechlmm_py import MechLMMCore, DebugCore, utilities_core
import mechlmm_py

import mechllm_bringup.function_pool_lmm_declaration as function_pool_lmm_declaration
from mechllm_bringup.function_pool_definition import FunctionPoolDefinition

class RedCubeDetector(Node):
    def __init__(self):
        super().__init__('red_cube_detector')
        self.mechlmm_core = MechLMMCore()
        self.debug_core = DebugCore()

        self.function_pool_definition = FunctionPoolDefinition()

        self.bridge = CvBridge()
        self.head_view_sub = self.create_subscription(
            Image, '/pi_camera/image_raw', self.head_view_callback, 10)
        
        self.image_sub = self.create_subscription(
            Image, '/intel_realsense_r200_depth/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/intel_realsense_r200_depth/depth/image_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/intel_realsense_r200_depth/camera_info', self.camera_info_callback, 10)

        self.operator_cmd_sub = self.create_subscription(
            String, 'operator_cmd', self.operator_cmd_callback, 10)
        
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.camera_intrinsics = None
        self.depth_image = None


        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_goal_handle = None

        self.fake_db = {
            "red_cube": {
                "position":{
                    "x": 1.97,
                    "y": 1.04,
                    "z": 0.0
                }
            },
            "home": {
                "position":{
                    "x": -3.0,
                    "y": 1.0,
                    "z": 0.0
                }
            }
        }

        self.fake_db_items = list(self.fake_db.keys())


        self.llm_tools = [function_pool_lmm_declaration.arm_end_effector_control,
                          function_pool_lmm_declaration.arm_end_effector_rotation_control,
                          function_pool_lmm_declaration.manipulation,
                          function_pool_lmm_declaration.move_robot,
                          function_pool_lmm_declaration.navigation,
                          function_pool_lmm_declaration.trigger_gripper
                        ]

        self.llm_tools_map = {
            "arm_end_effector_control": self.function_pool_definition.arm_end_effector_control,
            "arm_end_effector_rotation_control": self.function_pool_definition.arm_end_effector_rotation_control,
            "manipulation": self.function_pool_definition.manipulation,
            "move_robot": self.function_pool_definition.move_robot,
            "navigation": self.function_pool_definition.navigation,
            "trigger_gripper": self.function_pool_definition.trigger_gripper,
        }

        # self.send_goal(self.fake_db[0]["position"]["x"], self.fake_db[0]["position"]["y"], self.fake_db[0]["position"]["z"])
        self.llm_action = False

    def timer_callback(self):
        # self.debug_core.log_info(self.nav_goal_handle)
        pass
        
    def operator_cmd_callback(self, msg):
        self.llm_action = True

        # results = self.mechlmm_core.chat_tool(
        #     self.llm_tools, 
        #     f"""
        #         {msg.data}
        #     """
        #     )

        # self.debug_core.log_info(results)

        # if(results.tool_calls):
        #     tool_call = results.tool_calls[0]
        #     selected_tool = {
        #             "navigation": self.navigation,
        #             "manipulation": self.manipulation
        #         }[tool_call["name"].lower()]

        #     selected_tool(tool_call["args"])

        # elif(results.content):
        #     self.debug_core.log_key(results.content)


    def navigation(self, target_name):
        results, _ = self.mechlmm_core.chat_text(f"""
            find the list of similar items in the list provided: 
            {target_name}

            {self.fake_db_items}

            return the exact name of the matching item, if none found, return None
            Only return the name itself, no need for the reasoning or any additional content.       
            """
        )
        
        self.debug_core.log_info("------ find target object in db ------")
        self.debug_core.log_info(results)

        self.send_goal(self.fake_db[results]["position"]["x"], self.fake_db[results]["position"]["y"], self.fake_db[results]["position"]["z"])

    def manipulation(self, target_name):
        print(target_name)
        print("I am manipulating")

    def detect_color(self, hsv_frame, lower_bound, upper_bound):
        mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        return contours

    def draw_bounding_boxes(self, frame, contours, color_name, box_color):
        for contour in contours:
            if cv2.contourArea(contour) > 500:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x+w, y+h), box_color, 2)
                cv2.putText(frame, color_name, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, box_color, 2)


    def camera_info_callback(self, msg):
        self.camera_intrinsics = np.array(msg.k).reshape((3, 3))

    def head_view_callback(self, msg):
        # print("-=-")
        color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # self.object_tracking(color_image)

        if(self.llm_action):
            self.llm_action = False
            self.action_break(color_image)
        
        cv2.imshow("Detection", color_image)
        cv2.waitKey(1)

    def image_callback(self, msg):
        color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # self.object_tracking(color_image)
        
        cv2.imshow("Detection", color_image)
        cv2.waitKey(1)



    # TODO: Combine the source data of the object and robot end effector, and human input for weight to make decision
    # data calcuate the action (left, right), human intention,( left, right)
    # static data, AI feedback, human input
    def action_break(self, frame):
        base64_image = utilities_core.opencv_frame_to_base64(frame)
        image_url = f"data:image/jpeg;base64,{base64_image}"
        results = self.mechlmm_core.chat_tool(
            self.llm_tools,
            # f"""
            #     the red cube between the arm gripper is the center point of robot arm, it act as the reference point for any action robot is doing

            #     base on the position of blue cube, what should be the action need to do in order for robot arm to grab the blue cube

            #     use the tool function call provided
            #     if no action needed, then explain why
            # """,
            """
                the red cube between the arm gripper is the center point of robot arm, it act as the reference point for any action robot is doing
                what is the position of blue cube in terms of the red cube? (left, right, up, down)?

                base on the position of blue cube, what should be the action need to do in order for robot arm to grab the blue cube
                use the tool function call provided
                if no action needed, then explain why
            """,
            image_url
        )

        self.debug_core.log_info(results)

        if(results.tool_calls):
            tool_call = results.tool_calls[0]
            selected_tool = self.llm_tools_map[tool_call["name"].lower()]

            selected_tool(tool_call["args"])

        elif(results.content):
            self.debug_core.log_key(results.content)
        
    def object_tracking(self, color_image):
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv_image, lower_red, upper_red)

        red_lower = np.array([0, 120, 70])
        red_upper = np.array([10, 255, 255])
        red_contours = self.detect_color(hsv_image, red_lower, red_upper)
        
        green_lower = np.array([36, 50, 70])
        green_upper = np.array([89, 255, 255])
        green_contours = self.detect_color(hsv_image, green_lower, green_upper)
        
        blue_lower = np.array([90, 50, 70])
        blue_upper = np.array([128, 255, 255])
        blue_contours = self.detect_color(hsv_image, blue_lower, blue_upper)

        self.draw_bounding_boxes(color_image, red_contours, "Red", (0, 0, 255))
        self.draw_bounding_boxes(color_image, green_contours, "Green", (0, 255, 0))
        self.draw_bounding_boxes(color_image, blue_contours, "Blue", (255, 0, 0))


        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)

            # cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            center_x, center_y = x + w // 2, y + h // 2

            if self.depth_image is not None:
                depth = self.depth_image[center_y, center_x]

                if self.camera_intrinsics is not None:
                    u, v = center_x, center_y
                    z = depth

                    fx = self.camera_intrinsics[0, 0]
                    fy = self.camera_intrinsics[1, 1]
                    cx = self.camera_intrinsics[0, 2]
                    cy = self.camera_intrinsics[1, 2]

                    x_3d = (u - cx) * z / fx
                    y_3d = (v - cy) * z / fy

                    self.get_logger().info(f"3D Position of Red Cube in Camera Frame: X: {x_3d}, Y: {y_3d}, Z: {z}")

                    self.transform_to_map_frame(x_3d, y_3d, z)


    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")

    def transform_to_map_frame(self, x, y, z):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'camera_depth_frame', rclpy.time.Time())
            
            translation = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
            
            rotation_quat = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
            
            rotation_matrix = tf_transformations.quaternion_matrix(rotation_quat)[:3, :3]  # 3x3 rotation matrix
            
            cube_position_camera_frame = np.array([z, -x, y])

            cube_position_map_frame = np.dot(rotation_matrix, cube_position_camera_frame) + translation

            x_map, y_map, z_map = cube_position_map_frame

            self.get_logger().info(f"Red Cube Position in Map Frame: X: {x_map}, Y: {y_map}, Z: {z_map}")

            self.publish_marker(x_map, y_map, z_map)

        except Exception as e:
            self.get_logger().warn(f"Could not transform to map frame: {e}")

    def publish_marker(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "red_cube"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z

        q = quaternion_from_euler(0, 0, 0)
        marker.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)


    def send_goal(self, x, y, z):
        goal_msg = NavigateToPose.Goal()
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z
        goal_pose.pose.orientation.w = 1.0
        
        goal_msg.pose = goal_pose

        self.nav_action_client.wait_for_server()
        future = self.nav_action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.nav_goal_handle = future.result()
        if not self.nav_goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = self.nav_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('Goal reached within 1cm!')
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(result.status))
        

def main(args=None):
    rclpy.init(args=args)
    node = RedCubeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
