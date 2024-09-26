import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from mechllm_core import MechLLMCore
from postgres_core import PostgresCore
from debug_core import DebugCore
import utilities_core

import os
import time
from datetime import datetime
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
        
        if(ros_enable):
            self.node = Node('long_running_function_node')
            
            self.subscription = self.node.create_subscription(
                Image, 
                '/camera/image_raw', 
                self.camera_callback, 
                10
            )
            self.subscription
            
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
   

    ########## ROS
    def camera_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data)
        if(self.frame_width == None):
            self.frame_height, self.frame_width, channels = current_frame.shape

        self.video_saver(current_frame)

        thread = threading.Thread(target=self.run_with_lock, args=(current_frame,))
        thread.start()

        cv2.imshow("Robot Camera", current_frame)
        
        cv2.waitKey(1)
  
    def spin(self):
        rclpy.spin(self.node)

    def destroy_node(self):
        self.node.destroy_node()
    ########## ROS

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
            if(_tag["filename"] != self.video_filename):
                question = "The following content is a list of summary of continues frame from live view, can you summary them into a storyline of what happen in a short paragraph : \n\n" + '\n'.join(self.frame_context_list)
                _result, tag = self.mechllm_core.chat_text(question, None, _tag)

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

        tag = {"filename": "test"}

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

            # json_schema = {
            #     "title": "list_merge",
            #     "description": "combine the item with similar meaning together",
            #     "type": "object",
            #     "properties": {
            #         "item_list": {
            #             "type": "array",
            #             "items": {
            #                 "type": "string"
            #             },
            #             "description": "list of items",
            #         },
            #     },
            #     "required": ["item_list"]
            # }

            # question = f"""
            #     combine the items with similar meanings from the provided lists below into one dimension array as [item1, item2, item3]. 

            #     {_db_features + _current_features}
            # """

            # features_summary = self.mechllm_core.chat_text(question, json_schema)

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