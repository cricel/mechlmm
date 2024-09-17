import base64
import cv2

from langchain_core.messages import HumanMessage
from langchain_community.chat_models import ChatOllama

from debug_core import DebugCore
import utilities_core

class OllamaCore:
    def __init__(self):
        self.img_model = "llava"
        self.txt_model = "llama3.1"
        self.ollama_server_ip = "http://192.168.1.182:11434"
        self.debug_core = DebugCore()
        self.debug_core.verbose = 3

    def chat(self):
        pass

    def chat_text(self, _prompt):
        chat = ChatOllama(base_url=self.ollama_server_ip, model=self.img_model)

        msg = chat.invoke(
            [
                HumanMessage(content=_prompt)
            ]
        )

        return msg.content

    def chat_img(self, _base_img):
        # summary_prompt_text = """What's in this image?"""
        # feature_prompt_text = f"""
        # What objects are in the image, and give me the bounding box coordinate position of each object, and unique features of each object, and return it in json format.
        # and use this format {{objects: {{object_name: {{position: [top_left_x, top_left_y, bottom_right_x, bottom_right_y], features: [features_1, feature_2]}}}}}}
        # only return the json itself, no any other additional content
        # """

        feature_prompt_text = """
            "Analyze the following image and identify all objects present in the image. For each object, provide the following information:

            The bounding box position, specifying the coordinates for the top-left corner (top_left_x, top_left_y) and the bottom-right corner (bottom_right_x, bottom_right_y).
            A list of key features that were used to identify this object (e.g., color, texture, shape, etc.).
            The context of what this object is doing in the whole environment

            Please return the output strictly in the following JSON format:

            {
                "objects": {
                    "object_name_1": {
                        "position": ["top_left_x", "top_left_y", "bottom_right_x", "bottom_right_y"],
                        "features": ["features_1", "features_2", "feature_3"],
                        "context": "xxx"
                    },
                    "object_name_2": {
                        "position": ["top_left_x", "top_left_y", "bottom_right_x", "bottom_right_y"],
                        "features": ["features_1", "features_2", "feature_3"],
                        "context": "xxx"
                    }
                    // Add more objects as necessary
                }
            }

            Make sure to fill in all required information for each object detected in the image."
        """

        target_prompt = feature_prompt_text
        summary_result = self.image_summarize(_base_img, target_prompt)

        cleaned_summary_result = utilities_core.llm_output_json_cleaner(summary_result)

        self.debug_core.log_info("------ llm image analyzer output ------")
        self.debug_core.log_info(cleaned_summary_result)

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
    

    def video_summary_video_feed_storage(self, start_time, end_time, interval_time = 5):
        summary_prompt_text = """This is a live view, describe what you see in the scene"""
        conversion_list = []

        time_passed = end_time - start_time

        _frame_time_list = [i for i in range(0, time_passed, interval_time)] + [time_passed]
        
        for _frame_time in _frame_time_list:
            _frame = utilities_core.query_video_frame(_frame_time)

            frame_summary = self.image_summarize(utilities_core.opencv_frame_to_base64(_frame), summary_prompt_text)

            self.debug_core.log_info("------ llm video single frame analyzer output ------")
            self.debug_core.log_info(frame_summary)
            
            conversion_list.append(frame_summary)

        video_summary = self.chat_text("The following content is coming from a series of live view, can you summary them into a storyline of what happen in a short paragraph : \n\n" + '\n'.join(conversion_list))
        self.debug_core.log_info("------ llm video analyzer output ------")
        self.debug_core.log_info(video_summary)

        return video_summary

    def video_summary_old(self, video_path, start_time, end_time, interval_time = 5):
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

                frame_summary = self.image_summarize(utilities_core.opencv_frame_to_base64(frame), summary_prompt_text)

                self.debug_core.log_info("------ llm video single frame analyzer output ------")
                self.debug_core.log_info(frame_summary)
                
                conversion_list.append(frame_summary)
            
            current_frame += 1
        
        cap.release()

        video_summary = self.chat_text("The following content is coming from a series of live view, can you summary them into a storyline of what happen in a short paragraph : \n\n" + '\n'.join(conversion_list))
        self.debug_core.log_info("------ llm video analyzer output ------")
        self.debug_core.log_info(video_summary)

        return video_summary

    
if __name__ == '__main__':
    ollama_core = OllamaCore()

    # print(ollama_core.chat_text("I want to write a promot"))

    # base64_image = utilities_core.jpg_to_base64("../data/images/art_1.jpg")
    # ollama_core.chat_img(base64_image)

    # ollama_core.video_summary("../data/videos/fast_and_furious.mp4", 0, 30, 10)
    ollama_core.video_summary_video_feed_storage(7,15)