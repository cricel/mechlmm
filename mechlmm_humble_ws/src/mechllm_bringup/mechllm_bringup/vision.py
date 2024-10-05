import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry

import tf_transformations
import tf2_ros

from cv_bridge import CvBridge
    
import cv2

from mechlmm_py import MechLMMCore, PostgresCore, DebugCore, VisionCore, utilities_core

import os
import time
from datetime import datetime
import numpy as np
import threading

class MechVision:
    def __init__(self, ros_enable=False, data_path = "../output"):
        self.mechllm_core = MechLMMCore()
        self.postgres_core = PostgresCore()
        self.debug_core = DebugCore()
        self.vision_core = VisionCore()
        self.debug_core.verbose = 3

        self.node = Node('MechVision')

        self.bridge = CvBridge()
        
        self.image_sub = self.node.create_subscription(
            Image, '/intel_realsense_r200_depth/image_raw', self.raw_image_callback, 10)
        self.depth_sub = self.node.create_subscription(
            Image, '/intel_realsense_r200_depth/depth/image_raw', self.depth_image_callback, 10)
        self.camera_info_sub = self.node.create_subscription(
            CameraInfo, '/intel_realsense_r200_depth/camera_info', self.camera_info_callback, 10)

        self.marker_pub = self.node.create_publisher(Marker, 'visualization_marker', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        
        self.camera_intrinsics = None
        self.depth_image = None

        self.lock = threading.Lock()
        self.processing_thread = threading.Thread(target=self.process_frames)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        self.lmm_result = None

    def process_frames(self):
        while True:
            with self.lock:
                if hasattr(self, 'latest_frame'):
                    frame = self.latest_frame
                else:
                    frame = None

            if frame is not None:
                self.lmm_result = self.vision_core.frame_analyzer(frame)
            
            time.sleep(0.1)

    def raw_image_callback(self, msg):
        current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        self.vision_core.frame_height, self.vision_core.frame_width, channels = current_frame.shape

        self.vision_core.video_saver(current_frame)

        with self.lock:
                self.latest_frame = current_frame.copy()

        if(self.lmm_result):
            temp_bounding_box = self.lmm_result["objects"][0]["position"]
            cv2.putText(current_frame, self.lmm_result["objects"][0]["name"], (temp_bounding_box[0] + 10, temp_bounding_box[1] + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
            cv2.rectangle(current_frame, (temp_bounding_box[0], temp_bounding_box[1]), (temp_bounding_box[2], temp_bounding_box[3]), (255, 0, 0), 2)

        cv2.imshow("Robot Camera", current_frame)
        
        cv2.waitKey(1)


    def camera_info_callback(self, msg):
        self.camera_intrinsics = np.array(msg.k).reshape((3, 3))

    def depth_image_callback(self, msg):
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
            
            rotation_matrix = tf_transformations.quaternion_matrix(rotation_quat)[:3, :3]
            
            cube_position_camera_frame = np.array([z, -x, y])

            cube_position_map_frame = np.dot(rotation_matrix, cube_position_camera_frame) + translation

            x_map, y_map, z_map = cube_position_map_frame

            self.node.get_logger().info(f"Red Cube Position in Map Frame: X: {x_map}, Y: {y_map}, Z: {z_map}")

            self.publish_marker(x_map, y_map, z_map)

        except Exception as e:
            self.node.get_logger().warn(f"Could not transform to map frame: {e}")

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

        q = tf_transformations.quaternion_from_euler(0, 0, 0)
        marker.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)
  
    def spin(self):
        rclpy.spin(self.node)

    def destroy_node(self):
        self.node.destroy_node()

def main(args=None):
    rclpy.init(args=args)

    mech_vision = MechVision(True)
    mech_vision.spin()
    mech_vision.destroy_node()

    rclpy.shutdown()
  
if __name__ == '__main__':
    main()