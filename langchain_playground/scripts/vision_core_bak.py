import cv2
import time
import json
import base64
import shutil
import os

from ollama_core import OllamaCore
from postgres_core import PostgresCore
from debug_core import DebugCore

import concurrent.futures

from datetime import datetime

import utilities_core

class VisionCore:
    def __init__(self):
        self.video_context_switch_durtion = 30
        
        self.ollama_core = OllamaCore()
        self.postgres_core = PostgresCore()
        self.debug_core = DebugCore()
        self.debug_core.verbose = 3

        self.clear_old_videos()

        self.cam = cv2.VideoCapture(1)

        self.frame_width = int(self.cam.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.fps = 20.0

        self.saved_video_duration = 10
        self.video_filename = ""

        self.reference_video = None

        self.start_time = time.time()
        self.init_time = time.time()
        self.elapsed_time = 0

        self.executor = concurrent.futures.ThreadPoolExecutor(max_workers=5)
        self.image_analyse_thread = None

    def run(self):
        while True:
            ret, frame = self.cam.read()

            if not ret:
                self.debug_core.log_error("Error: Could not read frame.")
                break
            
            self.video_saver(frame)

            self.frame_analyzer(frame)

            cv2.imshow('Camera', frame)

            if cv2.waitKey(1) == ord('q'):
                break
    
    def clear_old_videos(self):
        video_files = [f for f in os.listdir(utilities_core.VIDEOS_OUTPUT_PATH) if f.startswith('output_video_') and f.endswith('.mp4')]
        for file in video_files:
            os.remove(os.path.join(utilities_core.VIDEOS_OUTPUT_PATH, file))
            self.debug_core.log_info(f"Deleted old video file: {file}")

    def video_saver(self, frame):
        current_time = time.time()
        if current_time - self.start_time >= self.saved_video_duration or self.reference_video is None:
            if self.reference_video is not None:
                self.reference_video.release()
                # self.video_summary_analyzer(self.video_filename, self.start_time, current_time)
            
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.video_filename = f'output_video_{timestamp}.mp4'
            
            self.reference_video = cv2.VideoWriter(os.path.join(utilities_core.VIDEOS_OUTPUT_PATH, self.video_filename), self.fourcc, self.fps, (self.frame_width, self.frame_height))
            self.start_time = current_time
            self.debug_core.log_info(f"Started recording: {os.path.join(utilities_core.VIDEOS_OUTPUT_PATH, self.video_filename)}")
        
        if self.reference_video is not None:
            self.reference_video.write(frame)

    def frame_analyzer(self, frame):
        if self.image_analyse_thread is None or self.image_analyse_thread.done():
            if(self.image_analyse_thread is not None):
                try:
                    json_object = json.loads(self.image_analyse_thread.result())
                    
                    if(json_object["objects"]):
                        self.debug_core.log_key("------ JSON Object ------")
                        self.debug_core.log_key(json_object)
                        for key, value in json_object["objects"].items():
                            final_features = None
                            final_reference_videos = []
                            final_summary = ""

                            _db_record = self.postgres_core.get_objects_map_record_by_name_db(key)
                            self.debug_core.log_key("------ start processing frame------")
                            self.debug_core.log_key(key)

                            self.debug_core.log_info("------ record from database ------")
                            self.debug_core.log_info(_db_record)

                            self.debug_core.log_info("------ load features anaylzer ------")
                            final_features = self.features_analyzer(_db_record["features"] if _db_record else None,
                                                                    value["features"])

                            self.debug_core.log_info("------ load video anaylzer  ------")
                            final_reference_videos, final_summary = self.video_analyzer(_db_record["reference_videos"] if _db_record else None,
                                                                                        self.elapsed_time)
                            
                            self.debug_core.log_key("------ before write to db ------")
                            self.debug_core.log_key(final_features)
                            self.debug_core.log_key(final_reference_videos)
                            self.debug_core.log_key(final_summary)
                            self.postgres_core.post_objects_map_db(key, final_features, final_reference_videos, final_summary)

                except Exception as e:
                    self.debug_core.log_warning("------ Error on Video Processing  ------")
                    self.debug_core.log_warning(e)
            
            self.elapsed_time = int(time.time() - self.init_time)
            self.debug_core.log_info(f"Current frame timestamp: {self.elapsed_time} s")
            
            self.image_analyse_thread = self.executor.submit(self.ollama_core.chat_img, utilities_core.opencv_frame_to_base64(frame))
        
    def features_analyzer(self, _db_features, _current_features):
        if(_db_features != None):
            self.debug_core.log_info("------ features for merge ------")
            self.debug_core.log_info(_db_features)
            self.debug_core.log_info(_current_features)
            features_summary = self.ollama_core.chat_text(f"""
                                        Merge the items with similar meanings from the two provided lists below, then combine the rest into a single list. 
                                        Format the final output as one single list as [feature1, feature2, feature3]. 
                                        Only return the JSON array of features, no need for the reasoning or any additional content.

                                        {_db_features}
                                        {_current_features}
                                        """)
            
            self.debug_core.log_info("------ features mege output from llm ------")
            self.debug_core.log_info(features_summary)
            
            output_features = utilities_core.llm_output_list_cleaner(features_summary)
            self.debug_core.log_info("------ features after clean up ------")
            self.debug_core.log_key(output_features)

            return output_features
        
        else:
            return _current_features
        
    def video_analyzer(self, _db_reference_videos, _elapsed_time):
        video_summary = ""

        if(_db_reference_videos != None):
            self.debug_core.log_info("------ record of references video from database ------")
            self.debug_core.log_info(_db_reference_videos)

            if(_db_reference_videos == []):
                _db_reference_videos.append([self.video_filename, str(_elapsed_time), str(_elapsed_time)])
            else:
                if(_elapsed_time - int(_db_reference_videos[-1][1]) < self.video_context_switch_durtion):
                    _db_reference_videos[-1][2] = str(_elapsed_time)
                else:
                    # self.debug_core.log_key("------ video summary start ------")
                    # temp_video = shutil.copy(self.video_file_path, self.video_folder_path + "temp.mp4")
                    # video_summary = self.ollama_core.video_summary(temp_video, int(_db_reference_videos[-1][1]), int(_db_reference_videos[-1][2]))
                    # os.remove(self.video_folder_path + "temp.mp4")

                    # video_summary = self.ollama_core.video_summary_video_feed_storage(int(_db_reference_videos[-1][1]), int(_db_reference_videos[-1][2]))
                    # self.debug_core.log_key(video_summary)

                    _db_reference_videos.append([self.video_filename, str(_elapsed_time), str(_elapsed_time)])
        else:
            _db_reference_videos = []
            _db_reference_videos.append([self.video_filename, str(_elapsed_time), str(_elapsed_time)])

        self.debug_core.log_info("------ record of references video from database ------")
        self.debug_core.log_info(_db_reference_videos)
        
        return _db_reference_videos, video_summary
    
    def video_summary_analyzer(self, _filename,_start_time, _end_time):
        
        summary = self.ollama_core.video_summary_file(os.path.join(utilities_core.VIDEOS_OUTPUT_PATH, _filename))
        self.postgres_core.post_video_summaries_db(_filename, 
                                                   utilities_core.time_to_string(_start_time), 
                                                   utilities_core.time_to_string(_end_time), 
                                                   summary)
    
if __name__ == '__main__':
    vision_core = VisionCore()
    vision_core.run()
