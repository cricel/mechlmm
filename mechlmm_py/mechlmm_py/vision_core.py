import cv2

from .mechlmm_core import MechLMMCore
from .postgres_core import PostgresCore
from .debug_core import DebugCore
from . import utilities_core

import os
import time
from datetime import datetime

class VisionCore:
    def __init__(self, _data_path = "../output"):
        self.mechlmm_core = MechLMMCore()
        self.postgres_core = PostgresCore()
        self.debug_core = DebugCore()
        self.debug_core.verbose = 3

        self.video_storage_path, self.image_storage_path = utilities_core.init_media_data_path(_data_path)
        utilities_core.clear_media_storage(_data_path)

        self.video_context_switch_durtion = 20
        self.frame_context_list = []

        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.fps = 20.0
        self.saved_video_duration = 10
        self.current_video_filename = ""
        self.current_video_writer = None

        self.start_time = int(time.time())
        self.init_time = int(time.time())
        self.elapsed_time = 0

        self.frame_width = 0
        self.frame_height = 0
        

    def ext_init(self, _frame_width, _frame_height):
        self.frame_width = _frame_width
        self.frame_height = _frame_height

    def frame_analyzer(self, _frame):
        self.elapsed_time = int(time.time() - self.init_time)
        self.debug_core.log_info(f"Current frame timestamp: {self.elapsed_time} s")
            
        json_object, _tag = self.image_context_analyzer(_frame)
        self.debug_core.log_key("=======================================")
        self.debug_core.log_key(json_object["objects"][0]["name"])
        self.debug_core.log_key(json_object["objects"][0]["position"])
        self.temp_bounding_box = json_object["objects"][0]["position"]
        self.debug_core.log_key("=======================================")
        
        try:
            self.frame_context_list.append(json_object["description"])

            self.debug_core.log_key("------ video check-----")
            self.debug_core.log_info(self.current_video_filename)
            self.debug_core.log_info(_tag["filename"])
            self.debug_core.log_key("------ video check-----")

            if(_tag["filename"] != self.current_video_filename):
                question = "The following content is a list of summary of continues frame from live view, return the summary of what happen in a short paragraph : \n\n" + '\n'.join(self.frame_context_list)
                _result, tag = self.mechlmm_core.chat_text(question, None, _tag)

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
        tag = {"filename": self.current_video_filename}

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

        _result = self.mechlmm_core.chat_img(question, image_url, json_schema, tag)

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

            features_summary, _ = self.mechlmm_core.chat_text(f"""
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
        

    def video_saver(self, frame):
        current_time = int(time.time())
        if current_time - self.start_time >= self.saved_video_duration or self.current_video_writer is None:
            if self.current_video_writer is not None:
                self.current_video_writer.release()
                self.postgres_core.post_video_record_db(self.current_video_filename, 
                                                   self.start_time, 
                                                   current_time
                                                   )
            
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.current_video_filename = f'output_video_{int(time.time())}.mp4'
            
            self.current_video_writer = cv2.VideoWriter(os.path.join(self.video_storage_path, self.current_video_filename), self.fourcc, self.fps, (self.frame_width, self.frame_height))
            self.start_time = current_time
            self.debug_core.log_info(f"Started recording: {os.path.join(self.video_storage_path, self.current_video_filename)}")
        
        if self.current_video_writer is not None:
            self.current_video_writer.write(frame)
  
if __name__ == '__main__':
    pass