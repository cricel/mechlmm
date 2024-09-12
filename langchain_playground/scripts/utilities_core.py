import cv2
import base64


def opencv_frame_to_base64(_frame):
    _, buffer = cv2.imencode('.jpg', _frame)
    frame_base64 = base64.b64encode(buffer).decode('utf-8')

    return frame_base64

def jpg_to_base64(image_path):
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode("utf-8")
        
