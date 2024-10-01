import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Quaternion
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from tf_transformations import quaternion_from_euler
import tf_transformations

class RedCubeDetector(Node):
    def __init__(self):
        super().__init__('red_cube_detector')
        
        # Set up subscribers
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/intel_realsense_r200_depth/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/intel_realsense_r200_depth/depth/image_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/intel_realsense_r200_depth/camera_info', self.camera_info_callback, 10)

        # Set up marker publisher
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

        # Setup tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.camera_intrinsics = None
        self.depth_image = None

    def camera_info_callback(self, msg):
        # Store the camera intrinsic parameters
        self.camera_intrinsics = np.array(msg.k).reshape((3, 3))

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Detect red objects
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv_image, lower_red, upper_red)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Get the largest contour (assuming it's the red cube)
            largest_contour = max(contours, key=cv2.contourArea)
            # Get bounding box for the contour
            x, y, w, h = cv2.boundingRect(largest_contour)

            # Draw the bounding box around the red cube
            cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Get the center of the red cube
            center_x, center_y = x + w // 2, y + h // 2

            # Extract depth at the center of the red cube
            if self.depth_image is not None:
                depth = self.depth_image[center_y, center_x]

                if self.camera_intrinsics is not None:
                    # Convert depth image pixel (u, v) to 3D position
                    u, v = center_x, center_y
                    # z = depth / 1000.0  # Convert mm to meters
                    z = depth  # Convert mm to meters

                    fx = self.camera_intrinsics[0, 0]
                    fy = self.camera_intrinsics[1, 1]
                    cx = self.camera_intrinsics[0, 2]
                    cy = self.camera_intrinsics[1, 2]

                    # 3D Coordinates in camera frame
                    x_3d = (u - cx) * z / fx
                    y_3d = (v - cy) * z / fy

                    self.get_logger().info(f"3D Position of Red Cube in Camera Frame: X: {x_3d}, Y: {y_3d}, Z: {z}")

                    # Now we transform this position to the map frame
                    self.transform_to_map_frame(x_3d, y_3d, z)

        # Display the image with the bounding box
        cv2.imshow("Red Cube Detection", color_image)
        cv2.waitKey(1)  # Refresh the display

    def depth_callback(self, msg):
        # Convert ROS Depth Image message to OpenCV format
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")

    def transform_to_map_frame(self, x, y, z):
        try:
            # Get the transformation from the camera frame to the map frame
            transform = self.tf_buffer.lookup_transform('map', 'camera_depth_frame', rclpy.time.Time())
            
            # Extract the translation (position) from the transform
            translation = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
            
            # Extract the rotation (quaternion) from the transform
            rotation_quat = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
            
            # Convert the quaternion to a rotation matrix
            rotation_matrix = tf_transformations.quaternion_matrix(rotation_quat)[:3, :3]  # 3x3 rotation matrix
            
            # Position of the cube in the camera frame
            cube_position_camera_frame = np.array([z, -x, y])

            # Apply the rotation and then the translation to get the cube's position in the map frame
            cube_position_map_frame = np.dot(rotation_matrix, cube_position_camera_frame) + translation

            # Extract the final map coordinates
            x_map, y_map, z_map = cube_position_map_frame

            self.get_logger().info(f"Red Cube Position in Map Frame: X: {x_map}, Y: {y_map}, Z: {z_map}")

            # Now publish the marker in RViz
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

        # Create a quaternion for orientation (no rotation in this case)
        q = quaternion_from_euler(0, 0, 0)
        marker.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # Set the size of the marker (adjust as needed)
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Set color (red for the red cube)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Publish the marker
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = RedCubeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
