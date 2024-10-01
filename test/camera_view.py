import cv2

from mechlmm_py import VisionCore, DebugCore

import threading
import time

from dotenv import load_dotenv
load_dotenv()

class CameraView:
    def __init__(self, _data_path = "../output"):
        self.vision_core = VisionCore(_data_path)
        self.debug_core = DebugCore()
        self.debug_core.verbose = 3

        self.cam = cv2.VideoCapture(1)

        self.lmm_result = None

        self.vision_core.frame_width = int(self.cam.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.vision_core.frame_height = int(self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.lock = threading.Lock()
        self.processing_thread = threading.Thread(target=self.process_frames)
        self.processing_thread.daemon = True
        self.processing_thread.start()

    def run(self):
        while True:
            ret, frame = self.cam.read()

            if not ret:
                self.debug_core.log_error("Error: Could not read frame.")
                break
            
            self.vision_core.video_saver(frame)

            with self.lock:
                self.latest_frame = frame.copy()

            if(self.lmm_result):
                temp_bounding_box = self.lmm_result["objects"][0]["position"]
                cv2.putText(frame, self.lmm_result["objects"][0]["name"], (temp_bounding_box[0] + 10, temp_bounding_box[1] + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
                cv2.rectangle(frame, (temp_bounding_box[0], temp_bounding_box[1]), (temp_bounding_box[2], temp_bounding_box[3]), (255, 0, 0), 2)
            cv2.imshow('Live Camera', frame)

            if cv2.waitKey(1) == ord('q'):
                break
    
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

if __name__ == '__main__':
    camera_view = CameraView()
    camera_view.run()