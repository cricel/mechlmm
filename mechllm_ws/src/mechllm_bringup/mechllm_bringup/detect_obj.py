import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from tf_transformations import quaternion_from_euler
import tf_transformations

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class RedCubeDetector(Node):
    def __init__(self):
        super().__init__('red_cube_detector')
        
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/intel_realsense_r200_depth/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/intel_realsense_r200_depth/depth/image_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/intel_realsense_r200_depth/camera_info', self.camera_info_callback, 10)

        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.camera_intrinsics = None
        self.depth_image = None


        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')


    def camera_info_callback(self, msg):
        self.camera_intrinsics = np.array(msg.k).reshape((3, 3))

    def image_callback(self, msg):
        color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv_image, lower_red, upper_red)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)

            cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

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

        cv2.imshow("Red Cube Detection", color_image)
        cv2.waitKey(1)

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
        goal_pose.pose.orientation.w = 1.0  # Adjust as needed for orientation
        
        goal_msg.pose = goal_pose

        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        goal_handle.result().add_done_callback(self.get_result_callback)

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
