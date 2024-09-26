import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import tf_transformations
from tf2_geometry_msgs import do_transform_point
from tf_transformations import quaternion_matrix
from tf2_ros import Buffer, TransformListener
import cv2

from mechllm_core import MechLLMCore
from postgres_core import PostgresCore
from debug_core import DebugCore
import utilities_core

import os
import time
from datetime import datetime
import numpy as np
import threading

class VisionCore:
    def __init__(self, ros_enable=False):
        self.mechllm_core = MechLLMCore()
        self.postgres_core = PostgresCore()
        self.debug_core = DebugCore()
        self.debug_core.verbose = 3

        self.clear_old_videos()
        self.video_context_switch_durtion = 20

        self.frame_width = None
        self.frame_height = None

        self.camera_info = None
        self.robot_position = None
        
        if(ros_enable):
            self.node = Node('long_running_function_node')
            
            self.image_subscription = self.node.create_subscription(
                Image, 
                '/intel_realsense_r200_depth/image_raw', 
                self.raw_image_callback, 
                10
            )
            self.image_subscription

            self.depth_subscription = self.node.create_subscription(
                Image, 
                '/intel_realsense_r200_depth/depth/image_raw', 
                self.depth_image_callback, 
                10
            )
            self.depth_subscription

            self.camera_info_subscriber = self.node.create_subscription(
                CameraInfo,
                '/intel_realsense_r200_depth/depth/camera_info',
                self.camera_info_callback,
                10
            )
            self.camera_info_subscriber

            self.point_publisher = self.node.create_publisher(PointStamped, '/detected_object_point', 10)
            self.marker_publisher = self.node.create_publisher(Marker, '/detected_object_marker', 10)

            self.odom_subscriber = self.node.create_subscription(
                Odometry,
                '/odom',
                self.odom_callback,
                10
            )
            self.odom_subscriber

            self.object_position_publisher = self.node.create_publisher(PointStamped, '/object_position', 10)
            
            self.br = CvBridge()
        else:
            self.cam = cv2.VideoCapture(0)

            self.frame_width = int(self.cam.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.frame_height = int(self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.fps = 20.0

        self.saved_video_duration = 10
        self.video_filename = ""

        self.reference_video = None

        self.start_time = int(time.time())
        self.init_time = int(time.time())
        self.elapsed_time = 0

        self.frame_context_list = []

        self.lock = threading.Lock()
        self.is_llm_processing = False

        self.current_depth_frame = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
   

    ########## ROS ##########
    def odom_callback(self, data):
        self.robot_position = data.pose.pose

    def raw_image_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data)
        if(self.frame_width == None):
            self.frame_height, self.frame_width, channels = current_frame.shape

        self.video_saver(current_frame)

        thread = threading.Thread(target=self.run_with_lock, args=(current_frame,))
        thread.start()

        cv2.imshow("Robot Camera", current_frame)
        
        cv2.waitKey(1)

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def depth_image_callback(self, data):
        try:
            depth_image = self.br.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except Exception as e:
            self.node.get_logger().error(f"Failed to convert image: {e}")
            return

        if depth_image is not None:
            test_pixel_x, test_pixel_y = 120, 20

            # location = self.depth_value_processing(depth_image, test_pixel_x, test_pixel_y)
            # print(location)
            # self.rviz_point_label(location, "test point")

            cv2.circle(depth_image, (test_pixel_x, test_pixel_y), 10, (0, 255, 0), 2)

            cv2.imshow('Depth Image', depth_image)
            cv2.waitKey(1)

    def depth_value_processing(self, _depth_image, _pixel_x, _pixel_y):
        depth_image = cv2.normalize(_depth_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_image = np.uint8(depth_image)

        self.current_depth_frame = depth_image

        depth_value = depth_image[_pixel_y, _pixel_x]
        # print(depth_value)
        # print( depth_value * 0.001)
        # print(depth_image.shape)

        # Intrinsic camera parameters
        fx = self.camera_info.k[0]  # Focal length in x direction
        fy = self.camera_info.k[4]  # Focal length in y direction
        cx = self.camera_info.k[2]  # Principal point x
        cy = self.camera_info.k[5]  # Principal point y
        
        # Convert pixel (u, v) to 3D point (x, y, z)
        z = depth_value / 1000.0  # Depth value in meters (assuming depth image is in millimeters)
        x = (_pixel_x - cx) * z / fx
        y = (_pixel_y - cy) * z / fy


        camera_position = np.array([x, y, z])

        point_msg = PointStamped()
        point_msg.header.frame_id = 'odom'  # The frame of odometry data
        point_msg.header.stamp = self.node.get_clock().now().to_msg()
        point_msg.point.x = self.robot_position.position.x
        point_msg.point.y = self.robot_position.position.y
        point_msg.point.z = self.robot_position.position.z

        try:
            # Transform the position from 'odom' to 'map' frame
            transform = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())  # Transform between map and odom
            # print(transform)
            print(transform.transform.translation)
            print("-=-=-=")

            # Convert the robot's orientation (quaternion) to a rotation matrix
            quaternion = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
            rotation_matrix = tf_transformations.quaternion_matrix(quaternion)[:3, :3]  # 3x3 rotation matrix

            # Apply the rotation to the camera position to transform it to the robot's frame
            transformed_position = np.dot(rotation_matrix, camera_position)
            # print("-++++++++-")
            # print(transformed_position)

            # Translate to the robot's global position
            transformed_position[0] += transform.transform.translation.x
            transformed_position[1] += transform.transform.translation.y
            transformed_position[2] += transform.transform.translation.z

            # print("----------")
            # print(transformed_position)


            point_msg.point.x = transformed_position[0]
            point_msg.point.y = transformed_position[1]
            point_msg.point.z = transformed_position[2]
            # self.object_position_publisher.publish(point_msg)
            print(point_msg)
            print("-=-=-=-=-=-=-=-=-0000000000000")
            
            transformed_point = do_transform_point(point_msg, transform)

            # Publish the transformed object position
            print(transformed_point)
            self.object_position_publisher.publish(transformed_point)
            # self.node.get_logger().info(f"Publishing Object Position in map frame: x={transformed_point.point.x:.2f}, y={transformed_point.point.y:.2f}, z={transformed_point.point.z:.2f}")

        except Exception as e:
            self.node.get_logger().error(f"Failed to transform object position: {str(e)}")




        # # The 3D position in the camera frame is (x, y, z)
        # camera_position = np.array([x, y, z])
        
        # # Transform the 3D position from the camera frame to the global frame (robot's pose)
        # transformed_position = self.transform_to_global(camera_position)

        # # Publish the transformed global position of the object
        # object_position_msg = PointStamped()
        # object_position_msg.header.stamp = self.node.get_clock().now().to_msg()
        # object_position_msg.header.frame_id = 'map'  # Global frame (or use the appropriate global frame)
        # object_position_msg.point.x = transformed_position[0]
        # object_position_msg.point.y = transformed_position[1]
        # object_position_msg.point.z = transformed_position[2]

        # self.object_position_publisher.publish(object_position_msg)
        # # self.node.get_logger().info(f"Publishing Object Position: x={transformed_position[0]:.2f}, y={transformed_position[1]:.2f}, z={transformed_position[2]:.2f}")


        return [x,y,z]

    def transform_to_global(self, camera_position):
        print(self.robot_position)
        print(camera_position)

        """
        Transforms the 3D position from the camera frame to the global frame using the robot's pose.
        """
        # Extract robot's position and orientation from the odometry message
        robot_x = self.robot_position.position.x
        robot_y = self.robot_position.position.y
        robot_z = self.robot_position.position.z
        robot_orientation = self.robot_position.orientation

        # Convert the robot's orientation (quaternion) to a rotation matrix
        quaternion = [
            robot_orientation.x,
            robot_orientation.y,
            robot_orientation.z,
            robot_orientation.w
        ]
        rotation_matrix = tf_transformations.quaternion_matrix(quaternion)[:3, :3]  # 3x3 rotation matrix

        # Apply the rotation to the camera position to transform it to the robot's frame
        transformed_position = np.dot(rotation_matrix, camera_position)
        print("-++++++++-")
        print(transformed_position)

        # Translate to the robot's global position
        transformed_position[0] += robot_x
        transformed_position[1] += robot_z
        transformed_position[2] += robot_y

        print("----------")
        print(transformed_position)

        # return transformed_position
        return [robot_x, robot_y, robot_z]
    
    def rviz_point_label(self, _position, _label):
        # Publish the point
        point_msg = PointStamped()
        point_msg.header = Header()
        point_msg.header.stamp = self.node.get_clock().now().to_msg()
        point_msg.header.frame_id = "map"  # Frame ID should match Rviz2
        point_msg.point.x = _position[0]
        point_msg.point.y = _position[1]
        point_msg.point.z = _position[2]
        self.point_publisher.publish(point_msg)
        
        # Publish the label (marker)
        marker_msg = Marker()
        marker_msg.header.frame_id = "map"  # Same frame as the point
        marker_msg.header.stamp = self.node.get_clock().now().to_msg()
        marker_msg.ns = "labels"
        marker_msg.id = 0
        marker_msg.type = Marker.TEXT_VIEW_FACING
        marker_msg.action = Marker.ADD

        # Position the label close to the point
        marker_msg.pose.position.x = point_msg.point.x
        marker_msg.pose.position.y = point_msg.point.y
        marker_msg.pose.position.z = point_msg.point.z + 0.2  # Slight offset above the point

        # Set label text
        marker_msg.text = _label

        # Set marker properties
        marker_msg.scale.z = 0.2  # Height of the text
        marker_msg.color.r = 1.0  # Red color
        marker_msg.color.g = 1.0  # Green color
        marker_msg.color.b = 1.0  # Blue color
        marker_msg.color.a = 1.0  # Fully opaque

        self.marker_publisher.publish(marker_msg)
  
    def spin(self):
        rclpy.spin(self.node)

    def destroy_node(self):
        self.node.destroy_node()
    ########## ROS ##########

    ########## Live View
    def run(self):
        while True:
            ret, frame = self.cam.read()

            if not ret:
                self.debug_core.log_error("Error: Could not read frame.")
                break
            
            self.video_saver(frame)

            thread = threading.Thread(target=self.run_with_lock, args=(frame,))
            thread.start()

            cv2.imshow('Live Camera', frame)

            if cv2.waitKey(1) == ord('q'):
                break
    ########## Live View

    def run_with_lock(self, _frame):
        with self.lock:
            self.frame_analyzer(_frame)

    def frame_analyzer(self, _frame):
        self.elapsed_time = int(time.time() - self.init_time)
        self.debug_core.log_info(f"Current frame timestamp: {self.elapsed_time} s")
            
        json_object, _tag = self.image_context_analyzer(_frame)

        try:
            # Save Video Summary
            self.frame_context_list.append(json_object["description"])

            self.debug_core.log_key("------ video check-----")
            self.debug_core.log_info(self.video_filename)
            self.debug_core.log_info(_tag["filename"])
            self.debug_core.log_key("------ video check-----")

            if(_tag["filename"] != self.video_filename):
                question = "The following content is a list of summary of continues frame from live view, return the summary of what happen in a short paragraph : \n\n" + '\n'.join(self.frame_context_list)
                _result, tag = self.mechllm_core.chat_text(question, None, _tag)

                self.debug_core.log_key("------ video summary-----")
                self.debug_core.log_info(_result)
                self.postgres_core.post_video_summary_db(tag["filename"], 
                                            _result
                                            )
                self.frame_context_list = []


            # Object DB
            for object in json_object["objects"]:
                final_features = None
                final_reference_videos = []
                final_summary = ""
                _db_record = self.postgres_core.get_objects_map_record_by_name_db(object["name"])
                self.debug_core.log_key("------ start processing frame------")
                self.debug_core.log_key(object["name"])

                self.debug_core.log_info("------ record from database ------")
                self.debug_core.log_info(_db_record)

                self.debug_core.log_info("------ load features anaylzer ------")
                final_features = self.features_analyzer(_db_record["features"] if _db_record else None,
                                                        object["features"])

                self.debug_core.log_info("------ load video anaylzer  ------")
                final_reference_videos = self.video_analyzer(_db_record["reference_videos"] if _db_record else None,
                                                                            self.elapsed_time)
                
                self.debug_core.log_key("------ before write to db ------")
                self.debug_core.log_key(final_features)
                self.debug_core.log_key(final_reference_videos)
                self.debug_core.log_key(final_summary)
                self.postgres_core.post_objects_map_db(object["name"], final_features, final_reference_videos, final_summary)

        except Exception as e:
                    self.debug_core.log_warning("------ Error on Video Processing  ------")
                    self.debug_core.log_warning(e)


    def image_context_analyzer(self, _frame):
        base64_image = utilities_core.opencv_frame_to_base64(_frame)
        image_url = f"data:image/jpeg;base64,{base64_image}"

        # tag = {"filename": "test"}
        tag = {"filename": self.video_filename}

        question = "analysis this image, and give me a detail break down of list of objects in the image"

        json_schema = {
            "title": "image_analysis",
            "description": "give a detail analysis of what happen in the image",
            "type": "object",
            "properties": {
                "objects": {
                    "type": "object",
                    "description": "the list of objects",
                    "properties": {
                        "name": {
                            "type": "string",
                            "description": "the name of the object detected",
                        },
                        "position": {
                            "type": "array",
                            "items": {
                                "type": "number"
                            },
                            "description": "the bounding box coordinate of the object detected, such as ['top_left_x', 'top_left_y', 'bottom_right_x', 'bottom_right_y']",
                        },
                        "features": {
                            "type": "array",
                            "items": {
                                "type": "string"
                            },
                            "description": "the key features of the object detected",
                        },
                    }
                },
                "description": {
                    "type": "string",
                    "description": "Overall description of what is seen in the image"
                }
            },
            "required": ["objects", "description"]
        }

        _result = self.mechllm_core.chat_img(question, image_url, json_schema, tag)

        return _result
    
    def video_analyzer(self, _db_reference_videos, _elapsed_time):
        _current_time = int(time.time())

        if(_db_reference_videos != None):
            self.debug_core.log_info("------ record of references video from database ------")
            self.debug_core.log_info(_db_reference_videos)

            if(_db_reference_videos == []):
                _db_reference_videos.append([_current_time, _current_time])
            else:
                if(_current_time - _db_reference_videos[-1][0] < self.video_context_switch_durtion):
                    _db_reference_videos[-1][1] = _current_time
                else:
                    _db_reference_videos.append([_current_time, _current_time])
        else:
            _db_reference_videos = []
            _db_reference_videos.append([_current_time, _current_time])

        self.debug_core.log_info("------ record of references video from database ------")
        self.debug_core.log_info(_db_reference_videos)
        
        return _db_reference_videos
    
    def features_analyzer(self, _db_features, _current_features):
        if(_db_features != None):
            self.debug_core.log_info("------ features for merge ------")
            self.debug_core.log_info(_db_features)
            self.debug_core.log_info(_current_features)

            features_summary, _ = self.mechllm_core.chat_text(f"""
                                        Merge the items with similar meanings from the provided lists below. 
                                        Format the final output as one single list as [feature1, feature2, feature3]. 
                                        Only return the JSON array of features, no need for the reasoning or any additional content.

                                        {_db_features + _current_features}
                                        """)
            
            self.debug_core.log_key("------ features mege output from llm ------")
            self.debug_core.log_key(features_summary)
            
            output_features = utilities_core.llm_output_list_cleaner(features_summary)
            self.debug_core.log_info("------ features after clean up ------")
            self.debug_core.log_key(output_features)

            return output_features
        
        else:
            return _current_features
        

    def clear_old_videos(self):
        video_files = [f for f in os.listdir(utilities_core.VIDEOS_OUTPUT_PATH) if f.startswith('output_video_') and f.endswith('.mp4')]
        for file in video_files:
            os.remove(os.path.join(utilities_core.VIDEOS_OUTPUT_PATH, file))
            self.debug_core.log_info(f"Deleted old video file: {file}")

    def video_saver(self, frame):
        current_time = int(time.time())
        if current_time - self.start_time >= self.saved_video_duration or self.reference_video is None:
            if self.reference_video is not None:
                self.reference_video.release()
                self.postgres_core.post_video_record_db(self.video_filename, 
                                                   self.start_time, 
                                                   current_time
                                                   )
            
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.video_filename = f'output_video_{int(time.time())}.mp4'
            
            self.reference_video = cv2.VideoWriter(os.path.join(utilities_core.VIDEOS_OUTPUT_PATH, self.video_filename), self.fourcc, self.fps, (self.frame_width, self.frame_height))
            self.start_time = current_time
            self.debug_core.log_info(f"Started recording: {os.path.join(utilities_core.VIDEOS_OUTPUT_PATH, self.video_filename)}")
        
        if self.reference_video is not None:
            self.reference_video.write(frame)

def ros_main(args=None):
    rclpy.init(args=args)

    long_running_function_node = VisionCore(True)
    long_running_function_node.spin()
    long_running_function_node.destroy_node()

    rclpy.shutdown()
  
if __name__ == '__main__':
    ros_main()

    # vision_core = VisionCore(False)
    # vision_core.run()