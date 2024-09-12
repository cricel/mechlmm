import cv2
import time
import json
import base64

from ollama_core import OllamaCore
from postgres_core import PostgresCore

import concurrent.futures

from datetime import datetime

## TODO
# Base on if current object is in the frame or not, set it to active, 
# and check if current time is within 2 sec of last time it was active
# if it is, keep updating the last time ending time variable
# if not, append a new 2 size array to indicate the new clip session, and record current time as start time
# then keep increase the ending time of the new added array

ollama_core = OllamaCore()
postgres_core = PostgresCore()

cam = cv2.VideoCapture(1)

frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

current_datetime = datetime.now()
datetime_string = current_datetime.strftime('%Y-%m-%d_%H:%M:%S')

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
fps = 20.0
out = cv2.VideoWriter('../output/videos/' + datetime_string + '.mp4', fourcc, fps, (frame_width, frame_height))

start_time = time.time()

executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)

last_future = None

detected_objects_dict = {}

while True:
    ret, frame = cam.read()

    if not ret:
        break
    
    _, buffer = cv2.imencode('.jpg', frame)
    frame_base64 = base64.b64encode(buffer).decode('utf-8')

    if last_future is None or last_future.done():
        if(last_future is not None and last_future.done()):
            try:
                json_object = json.loads(last_future.result())
                # object_names = list(json_object["objects"].keys())
                
                for key, value in json_object.items():
                    if key in detected_objects_dict:
                        if not isinstance(detected_objects_dict[key], list):
                            detected_objects_dict[key] = [detected_objects_dict[key]]
                        detected_objects_dict[key].append(value)
                    else:
                        detected_objects_dict[key] = value
                        print("-=-=-=-=-= value")
                        print(value)
                        print("-=-=-=-=-= value")
                
                print(detected_objects_dict)
            except:
                print("Something wrong with the llm generated json, ignore current detection")

        elapsed_time = time.time() - start_time
        timestamp = elapsed_time * 1000
        print(f"Current frame timestamp: {timestamp:.2f} ms")
        
        last_future = executor.submit(ollama_core.chat_img, frame_base64)
    
    out.write(frame)

    cv2.imshow('Camera', frame)

    if cv2.waitKey(1) == ord('q'):
        break

cam.release()
out.release()
cv2.destroyAllWindows()