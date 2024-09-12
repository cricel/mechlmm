import base64
import cv2

from langchain_core.messages import HumanMessage
from langchain_community.chat_models import ChatOllama

import utilities_core

class OllamaCore:
    def __init__(self):
        self.img_model = "llava"
        self.txt_model = "llama3.1"
        self.ollama_server_ip = "http://192.168.1.182:11434"

    def chat(self):
        pass

    def chat_text(self, _prompt):
        chat = ChatOllama(base_url=self.ollama_server_ip, model=self.txt_model)

        msg = chat.invoke(
            [
                HumanMessage(content=_prompt)
            ]
        )

        return msg.content

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

    def image_summarize(self, img_base64, prompt):
        chat = ChatOllama(base_url=self.ollama_server_ip, model=self.img_model)

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
        summary_prompt_text = """This is a live view, describe what you see in the scene"""
        conversion_list = []

        cap = cv2.VideoCapture(video_path)
        fps = cap.get(cv2.CAP_PROP_FPS)
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        
        start_frame = int(start_time * fps)
        end_frame = int(end_time * fps)
        
        end_frame = min(end_frame, total_frames - 1)
        
        cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)
        
        current_frame = start_frame
        
        while current_frame <= end_frame:
            ret, frame = cap.read()
            if not ret:
                break
            
            if (
                current_frame == start_frame or 
                (current_frame - start_frame) % (int(interval_time * fps)) == 0 or 
                (current_frame == end_frame)
            ):

                cv2.imshow('Video Frame', frame)

                _summary = self.image_summarize(utilities_core.opencv_frame_to_base64(frame), summary_prompt_text)
                
                print("--- Video Frame Summary ---")
                print(_summary)
                
                conversion_list.append(_summary)
            
            current_frame += 1
        
        cap.release()

        print("========== Complete Video Summary ==========")
        summary = self.chat_text("The following content is coming from a series of live view, can you summary them into a storyline of what happen in a short paragraph : \n\n" + '\n'.join(conversion_list))
        print(summary)

        return summary

    
if __name__ == '__main__':
    ollama_core = OllamaCore()

    # ollama_core.chat_text("how are you")

    # base64_image = utilities_core.jpg_to_base64("../data/images/art_1.jpg")
    # ollama_core.chat_img(base64_image)

    ollama_core.video_summary("../data/videos/fast_and_furious.mp4", 0, 30, 10)