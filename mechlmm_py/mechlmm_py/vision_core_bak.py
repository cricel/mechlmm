IS_ROS_ENABLE = False

if(IS_ROS_ENABLE):
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
    def __init__(self, ros_enable=False, data_path = "../output"):
        self.mechllm_core = MechLLMCore()
        self.postgres_core = PostgresCore()
        self.debug_core = DebugCore()
        self.debug_core.verbose = 3

        self.init_data_path(data_path)

        self.clear_old_videos()
        self.video_context_switch_durtion = 20

        self.frame_width = None
        self.frame_height = None

        self.camera_info = None
        self.robot_position = None
        
        if(ros_enable):
            self.node = Node('long_running_function_node')

            self.bridge = CvBridge()
            
            self.image_sub = self.node.create_subscription(
                Image, '/intel_realsense_r200_depth/image_raw', self.raw_image_callback, 10)
            self.depth_sub = self.node.create_subscription(
                Image, '/intel_realsense_r200_depth/depth/image_raw', self.depth_callback, 10)
            self.camera_info_sub = self.node.create_subscription(
                CameraInfo, '/intel_realsense_r200_depth/camera_info', self.camera_info_callback, 10)


            self.marker_pub = self.node.create_publisher(Marker, 'visualization_marker', 10)

            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
            
            self.camera_intrinsics = None
            self.depth_image = None

        else:
            self.cam = cv2.VideoCapture(1)

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

        self.temp_bounding_box = [0,0,0,0]

        
    def init_data_path(self, _data_path):
        self.VIDEOS_OUTPUT_PATH = os.path.join(_data_path, "videos")
        self.IMAGES_OUTPUT_PATH = os.path.join(_data_path, "images")

        os.makedirs(self.VIDEOS_OUTPUT_PATH, exist_ok=True)
        os.makedirs(self.IMAGES_OUTPUT_PATH, exist_ok=True)

    ########## ROS ##########

    def raw_image_callback(self, msg):
        current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if(self.frame_width == None):
            self.frame_height, self.frame_width, channels = current_frame.shape

        self.video_saver(current_frame)

        thread = threading.Thread(target=self.run_with_lock, args=(current_frame,))
        thread.start()

        cv2.rectangle(current_frame, (self.temp_bounding_box[0], self.temp_bounding_box[1]), (self.temp_bounding_box[2], self.temp_bounding_box[3]), (0, 255, 0), 2)

        cv2.imshow("Robot Camera", current_frame)
        
        cv2.waitKey(1)


    def camera_info_callback(self, msg):
        self.camera_intrinsics = np.array(msg.k).reshape((3, 3))

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
        self.debug_core.log_key("=======================================")
        self.debug_core.log_key(json_object["objects"][0]["name"])
        self.debug_core.log_key(json_object["objects"][0]["position"])
        self.temp_bounding_box = json_object["objects"][0]["position"]
        self.debug_core.log_key("=======================================")
        # try:
        #     # Save Video Summary
        #     self.frame_context_list.append(json_object["description"])

        #     self.debug_core.log_key("------ video check-----")
        #     self.debug_core.log_info(self.video_filename)
        #     self.debug_core.log_info(_tag["filename"])
        #     self.debug_core.log_key("------ video check-----")

        #     if(_tag["filename"] != self.video_filename):
        #         question = "The following content is a list of summary of continues frame from live view, return the summary of what happen in a short paragraph : \n\n" + '\n'.join(self.frame_context_list)
        #         _result, tag = self.mechllm_core.chat_text(question, None, _tag)

        #         self.debug_core.log_key("------ video summary-----")
        #         self.debug_core.log_info(_result)
        #         self.postgres_core.post_video_summary_db(tag["filename"], 
        #                                     _result
        #                                     )
        #         self.frame_context_list = []


        #     # Object DB
        #     for object in json_object["objects"]:
        #         final_features = None
        #         final_reference_videos = []
        #         final_summary = ""
        #         _db_record = self.postgres_core.get_objects_map_record_by_name_db(object["name"])
        #         self.debug_core.log_key("------ start processing frame------")
        #         self.debug_core.log_key(object["name"])

        #         self.debug_core.log_info("------ record from database ------")
        #         self.debug_core.log_info(_db_record)

        #         self.debug_core.log_info("------ load features anaylzer ------")
        #         final_features = self.features_analyzer(_db_record["features"] if _db_record else None,
        #                                                 object["features"])

        #         self.debug_core.log_info("------ load video anaylzer  ------")
        #         final_reference_videos = self.video_analyzer(_db_record["reference_videos"] if _db_record else None,
        #                                                                     self.elapsed_time)
                
        #         self.debug_core.log_key("------ before write to db ------")
        #         self.debug_core.log_key(final_features)
        #         self.debug_core.log_key(final_reference_videos)
        #         self.debug_core.log_key(final_summary)
        #         self.postgres_core.post_objects_map_db(object["name"], final_features, final_reference_videos, final_summary)

        # except Exception as e:
        #             self.debug_core.log_warning("------ Error on Video Processing  ------")
        #             self.debug_core.log_warning(e)


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
        video_files = [f for f in os.listdir(self.VIDEOS_OUTPUT_PATH) if f.startswith('output_video_') and f.endswith('.mp4')]
        for file in video_files:
            os.remove(os.path.join(self.VIDEOS_OUTPUT_PATH, file))
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
            
            self.reference_video = cv2.VideoWriter(os.path.join(self.VIDEOS_OUTPUT_PATH, self.video_filename), self.fourcc, self.fps, (self.frame_width, self.frame_height))
            self.start_time = current_time
            self.debug_core.log_info(f"Started recording: {os.path.join(self.VIDEOS_OUTPUT_PATH, self.video_filename)}")
        
        if self.reference_video is not None:
            self.reference_video.write(frame)

def ros_main(args=None):
    rclpy.init(args=args)

    long_running_function_node = VisionCore(True)
    long_running_function_node.spin()
    long_running_function_node.destroy_node()

    rclpy.shutdown()
  
if __name__ == '__main__':
    if(IS_ROS_ENABLE):
        # Uncomment this to use it for ROS
        ros_main()
    else:
        # Uncomment this to use it WITHOUT ROS, and use camera view
        vision_core = VisionCore(False)
        vision_core.run()