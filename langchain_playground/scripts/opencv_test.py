import cv2
import time
import json
import base64

from ollama_core import OllamaCore
from postgres_core import PostgresCore

import concurrent.futures

from datetime import datetime

import utilities_core

## TODO
# Search related info and video sequence when ask

ollama_core = OllamaCore()
postgres_core = PostgresCore()

cam = cv2.VideoCapture(1)

frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

current_datetime = datetime.now()
video_file_name = current_datetime.strftime('%Y-%m-%d_%H-%M-%S')
video_file_path = '../output/videos/' + video_file_name + '.mp4'

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
fps = 20.0
out = cv2.VideoWriter(video_file_path, fourcc, fps, (frame_width, frame_height))

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
        if(last_future is not None):
            try:
                json_object = json.loads(last_future.result())
                
                if(json_object["objects"]):
                    for key, value in json_object["objects"].items():
                        current_features = value["features"]
                        current_reference_videos = []
                        key_record = postgres_core.get_objects_map_record_by_name_db(key)
                        
                        print("-=-=-=-=-=-=-")
                        print(key_record)
                        # if(key_record != None):
                        #     print("-=-=-reference_videos-=-=-=-")
                        #     print(key_record["reference_videos"])

                        if(key_record != None and key_record["features"] != None):
                            print("-----")
                            print(key_record["features"])
                            print(current_features)
                            print("=========")
                            features_future = executor.submit(ollama_core.chat_text, f"""
                                                        Merge the items with similar meanings from the two provided lists below, then combine the rest into a single list. 
                                                        Format the final output as one single list as [feature1, feature2, feature3]. 
                                                        Only return the JSON array of features, no need for the reasoning or any additional content.

                                                        {key_record["features"]}
                                                        {current_features}
                                                        """)
                            
                            new_features = features_future.result()
                            print(new_features)
                            print("))---((")
                            current_features = utilities_core.llm_output_list_cleaner(new_features)
                            print(current_features)

                        if(key_record != None and key_record["reference_videos"] != None):
                            current_reference_videos = key_record["reference_videos"]
                            print("-=-=-reference_videos-=-=-=-")
                            print(current_reference_videos)

                            if(current_reference_videos == []):
                                print("1=====")
                                current_reference_videos.append([video_file_name, str(elapsed_time), str(elapsed_time)])
                                print(key)
                                print(current_reference_videos)
                            else:
                                if(elapsed_time - int(current_reference_videos[-1][1]) < 30):
                                    print("2=====")
                                    current_reference_videos[-1][2] = str(elapsed_time)
                                    print(key)
                                    print(current_reference_videos)
                                else:
                                    ## TODO
                                    # Add video summary for previous one when jump
                                    # video_summary = ollama_core.video_summary(video_file_path, int(current_reference_videos[-1][1]), int(current_reference_videos[-1][2]))
                                    print("3=====")
                                    current_reference_videos.append([video_file_name, str(elapsed_time), str(elapsed_time)])
                                    print(key)
                                    print(current_reference_videos)
                        else:
                            print("4=====")
                            current_reference_videos.append([video_file_name, str(elapsed_time), str(elapsed_time)])
                            print(key)
                            print(current_reference_videos)
                            
                        postgres_core.post_objects_map_db(key, current_features, current_reference_videos)

            except Exception as e:
                print(e)
        
        elapsed_time = int(time.time() - start_time)
        print(f"Current frame timestamp: {elapsed_time} s")
        
        last_future = executor.submit(ollama_core.chat_img, frame_base64)
    
    out.write(frame)

    cv2.imshow('Camera', frame)

    if cv2.waitKey(1) == ord('q'):
        break

cam.release()
out.release()
cv2.destroyAllWindows()