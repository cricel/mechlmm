import cv2
import base64
import ast

def opencv_frame_to_base64(_frame):
    _, buffer = cv2.imencode('.jpg', _frame)
    frame_base64 = base64.b64encode(buffer).decode('utf-8')

    return frame_base64

def jpg_to_base64(image_path):
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode("utf-8")
        
def llm_output_json_cleaner(json_text):
    cleaned_json_text = json_text.replace("```json", "")
    cleaned_json_text = cleaned_json_text.replace("```", "")

    return cleaned_json_text

def llm_output_list_cleaner(string_list):
     return ast.literal_eval(string_list)
