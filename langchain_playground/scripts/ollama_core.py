import base64
import cv2

from langchain_core.messages import HumanMessage
from langchain_community.chat_models import ChatOllama

class OllamaCore:
    def __init__(self):
        self.model = "llava"
        self.ollama_server_ip = "http://192.168.1.182:11434"

    def chat(self):
        pass

    def chat_text(self):
        pass

    def chat_img(self, _base_img):
        summary_prompt_text = """What's in this image?"""
        feature_prompt_text = """
        What objects are in the image, and give me the bounding box coordinate position of each object, and unique features of each object, and return it in json format.
        and use this format {objects: {object_name: {position: [top_left_x, top_left_y, bottom_right_x, bottom_right_y]}, features: [features_1, feature_2]}
        only return the json itself, no any other additional content
        """

        target_prompt = feature_prompt_text
        summary_result = self.image_summarize(_base_img, target_prompt)

        cleaned_summary_result = summary_result.replace("```json", "")
        cleaned_summary_result = cleaned_summary_result.replace("```", "")

        print(cleaned_summary_result)

        return cleaned_summary_result

    def encode_image(self, image_path):
        """Getting the base64 string"""
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode("utf-8")


    def image_summarize(self, img_base64, prompt):
        """Make image summary"""
        chat = ChatOllama(base_url=self.ollama_server_ip, model=self.model)

        image_url = f"data:image/jpeg;base64,{img_base64}"

        msg = chat.invoke(
            [
                HumanMessage(
                    content=[
                        {"type": "text", "text": prompt},
                        {"type": "image_url", "image_url": image_url},
                    ]
                )
            ]
        )

        return msg.content
    
    def video_summary(self, video_path, start_time, end_time, interval_time = 5):
        # Load video
        cap = cv2.VideoCapture(video_path)
        
        # Get the FPS (frames per second) of the video
        fps = cap.get(cv2.CAP_PROP_FPS)
        
        # Calculate the total number of frames
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        
        # Calculate the start and end frame numbers
        start_frame = int(start_time * fps)
        end_frame = int(end_time * fps)
        
        # Ensure that the end frame doesn't exceed the total frame count
        end_frame = min(end_frame, total_frames - 1)
        
        # Move to the start frame
        cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)
        
        frames = []
        current_frame = start_frame
        
        # Loop through the frames
        while current_frame <= end_frame:
            ret, frame = cap.read()
            if not ret:
                break
            
            # Capture the first frame
            if current_frame == start_frame:
                frames.append(('first_frame', frame))
            
            # Capture every 5 seconds frame
            if (current_frame - start_frame) % (int(5 * fps)) == 0:
                frames.append((f'frame_at_{current_frame // fps}_sec', frame))
            
            # Capture the last frame
            if current_frame == end_frame:
                frames.append(('last_frame', frame))
            
            current_frame += 1
        
        cap.release()
        return frames

    
if __name__ == '__main__':
    ollama_core = OllamaCore()

    img_path = "../data/images/art_1.jpg"
    base64_image = ollama_core.encode_image(img_path)
    
    ollama_core.chat_img(base64_image)