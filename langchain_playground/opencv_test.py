import cv2
import base64

from ollama_image import OllamaCore

import concurrent.futures


ollama_core = OllamaCore()

cam = cv2.VideoCapture(0)

frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)

last_future = None

while True:
    ret, frame = cam.read()

    if not ret:
        break
    
    _, buffer = cv2.imencode('.jpg', frame)
    frame_base64 = base64.b64encode(buffer).decode('utf-8')
    
    if last_future is None or last_future.done():
        last_future = executor.submit(ollama_core.chat_img, frame_base64)
    
    cv2.imshow('Camera', frame)

    if cv2.waitKey(1) == ord('q'):
        break

cam.release()
cv2.destroyAllWindows()