import cv2
import base64
import ast

import os
from datetime import datetime, timedelta

import requests

from .debug_core import DebugCore

from langchain_core.utils.function_calling import convert_to_openai_function

debug_core = DebugCore()
debug_core.verbose = 3

def opencv_frame_to_base64(_frame):
    _, buffer = cv2.imencode('.jpg', _frame)
    frame_base64 = base64.b64encode(buffer).decode('utf-8')
    image_url = f"data:image/jpeg;base64,{frame_base64}"

    return image_url

def jpg_to_base64(image_path):
        with open(image_path, "rb") as image_file:
            frame_base64 = base64.b64encode(image_file.read()).decode("utf-8")
            image_url = f"data:image/jpeg;base64,{frame_base64}"

            return image_url
        
def llm_output_json_cleaner(json_text):
    start = json_text.find("{")
    end = json_text.rfind("}")
    if start != -1 and end != -1 and start < end:
        return json_text[start:end+1]
    return ""

def llm_output_list_cleaner(string_list):
    clean_string = ""
    start = string_list.find("[")
    end = string_list.find("]")
    if start != -1 and end != -1 and start < end:
        clean_string = string_list[start:end+1]

    return ast.literal_eval(clean_string)

def query_video_frame(requested_time):
    video_files = sorted([f for f in os.listdir(VIDEOS_OUTPUT_PATH) if f.startswith('output_video_') and f.endswith('.mp4')])
    
    timestamps = []
    for filename in video_files:
        try:
            timestamp_str = filename.replace('output_video_', '').replace('.mp4', '')
            timestamp = datetime.strptime(timestamp_str, '%Y%m%d_%H%M%S')
            timestamps.append((timestamp, filename))
        except ValueError:
            debug_core.log_error(f"Error parsing timestamp from filename: {filename}")
            continue

    timestamps.sort(key=lambda x: x[0])
    
    for i in range(len(timestamps)):
        video_start_time = timestamps[i][0]
        
        video_relative_time = requested_time - (video_start_time - timestamps[0][0]).total_seconds()
        if 0 <= video_relative_time < 10:
            video_filename = timestamps[i][1]
            frame_time_in_video = video_relative_time
            break
    else:
        debug_core.log_error(f"No video file found for the requested time: {requested_time} seconds")
        return None
    
    cap = cv2.VideoCapture(os.path.join(VIDEOS_OUTPUT_PATH, video_filename))
    
    if not cap.isOpened():
        debug_core.log_error(f"Error: Could not open video file {video_filename}")
        return None
    
    fps = cap.get(cv2.CAP_PROP_FPS)
    frame_number = int(fps * frame_time_in_video)
    
    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_number)
    
    ret, frame = cap.read()
    
    if not ret:
        debug_core.log_error(f"Error: Could not read frame at {requested_time} seconds from video {video_filename}")
        cap.release()
        return None

    cap.release()
    
    return frame

def frame_to_jpg(frame, filename):
    cv2.imwrite(os.path.join(IMAGES_OUTPUT_PATH, filename), frame)
    debug_core.log_info(f"Frame saved as {os.path.join(IMAGES_OUTPUT_PATH, filename)}")

def time_to_string(_time):
    dt_object = datetime.fromtimestamp(_time)
    timestamp = dt_object.strftime('%Y%m%d_%H%M%S')

    return timestamp

def init_media_data_path(_data_path):
    print(_data_path)
    video_path = os.path.join(_data_path, "videos")
    image_path = os.path.join(_data_path, "images")

    os.makedirs(video_path, exist_ok=True)
    os.makedirs(image_path, exist_ok=True)

    return video_path, image_path

def clear_media_storage(_data_path):
    video_files = [f for f in os.listdir(os.path.join(_data_path, "videos")) if f.startswith('output_video_') and f.endswith('.mp4')]
    for file in video_files:
        os.remove(os.path.join(_data_path, "videos", file))
        debug_core.log_info(f"Deleted old video file: {file}")

def rest_post_request(_data, _server_url = 'http://192.168.1.134:5001/mechlmm/chat'):
    """
    data = {
        'question': 'question',
        'schema': schema,
        'tag': 'tag',
        'base_img': [base_img_1, base_img_2],
        'tools': [tools_1, tools_2],
        'model': "claude"
    }
    """

    response = requests.post(_server_url, json = _data)

    if response.status_code == 200:
        
        result = response.json()
        return result
    else:
        print('Failed:', response.status_code, response.text)
        return None
    
def basemodel_to_json(_basemodel):
    return convert_to_openai_function(_basemodel)

def ros_message_to_dict(_msg):
    if hasattr(_msg, '__slots__'):
        output = {}
        for field_name in _msg.__slots__:
            field_value = getattr(_msg, field_name)
            output[field_name] = ros_message_to_dict(field_value)
        return output
    
    elif isinstance(_msg, list):
        return [ros_message_to_dict(item) for item in _msg]
    
    else:
        return _msg
    
if __name__ == '__main__':
    frame_to_jpg(query_video_frame(7), "test.jpg")