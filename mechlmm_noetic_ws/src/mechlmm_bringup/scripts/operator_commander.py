#!/usr/bin/env python

import speech_recognition as sr
from mechlmm_py import TTS_Core, DebugCore, utilities_core, PostgresCore, lmm_function_pool
import sys

import cv2

class MechLMMChatBot:
    def __init__(self):
        self.tts_core = TTS_Core()
        self.debug_core = DebugCore()
        self.postgres_core = PostgresCore(False)

        self.recognizer = sr.Recognizer()

    def get_speech_input(self):
        with sr.Microphone() as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=0.2)
            audio = self.recognizer.listen(source)
            try:
                speech_text = self.recognizer.recognize_google(audio)
                return speech_text.lower()
            except sr.RequestError as e:
                self.debug_core.log_error(f"Could not request results; {e}")
                return None
            except sr.UnknownValueError:
                self.debug_core.log_error("Unknown error occurred")
                return None

    def send_request(self, question):
        chat_type = "qa"

        if len(sys.argv) > 1:
            chat_type = sys.argv[1]
        
        data = {
            'question': question,
        }
        try:
            response = utilities_core.rest_post_request(data, 'http://192.168.1.182:5001/mechlmm/chat')

            if(chat_type == "qa"):
                # response = utilities_core.rest_post_request(data, 'http://192.168.1.182:5001/mechlmm/chat/qa')
                response = self.chat_knowledge(question)
            if(chat_type == "data"):
                response = utilities_core.rest_post_request(data, 'http://192.168.1.182:5001/mechlmm/chat/data')

            return response
        except Exception as e:
            self.debug_core.log_error(f"Error sending request: {e}")
            return None

    def play_response(self, response_text):
        self.tts_core.tts_play(response_text)

    def run(self):
        while True:
            self.debug_core.log_info("How can I help you?")
            user_input = self.get_speech_input()
            if user_input:
                self.debug_core.log_info(f"You Said: {user_input}")
                result = self.send_request(user_input)
                if result:
                    self.debug_core.log_info(f"MechLMM: {result}")
                    self.play_response(result["result"])

    def run_terminal(self):
        while True:
            self.debug_core.log_flash("======= User Input =======")

            user_input = input("How can I help you: ")
            
            if user_input.lower() == "exit":
                self.debug_core.log_flash("Exiting...")
                break
            else:
                self.debug_core.log_info(self.send_request(user_input))

    def chat_knowledge(self, _question):
        self.debug_core.log_info("------ chat_text_knowledge ------")
        db_item_list = self.postgres_core.get_objects_map_name_list_db()
        db_video_list = self.postgres_core.get_video_summary_list_db()

        query = f"""
                Dont answer the question, just parser the following question to find the list of similar items in the list provided: 
                {_question}
            
                {db_item_list}

                return the exact name of the matching item in provided list as array, if none found, return None as item
                """
        
        data = {
            'question': query,
            'schema': utilities_core.basemodel_to_json(lmm_function_pool.ListItems),
            # 'tag': 'tag',
            # 'base_img': [base_img_1, base_img_2],
            # 'tools': [tools_1, tools_2],
            # 'model': "claude"
        }
        results = utilities_core.rest_post_request(data, "http://192.168.1.182:5001/mechlmm/chat")
        

        
        # results, _, return_type = self.chat(_question = query, _schema = utilities_core.basemodel_to_json(lmm_function_pool.ListItems))
        
        self.debug_core.log_info("------ find target object in db ------")
        self.debug_core.log_info(results["result"])

            
        video_list = []
        object_db_list = []

        # if(results["items"][0] == "robot"):
        #     test_db = self.postgres_core.get_table("data_log")
        #     print(test_db)

        for result in results["result"]["items"]:
            _record_result = self.postgres_core.get_objects_map_record_by_name_db(result)
            object_db_list.append(_record_result)
            self.debug_core.log_key("---++++++----")
            self.debug_core.log_key(result)
            self.debug_core.log_key(_record_result)
            for video_time in _record_result["reference_videos"]:
                matching_video = utilities_core.find_video_in_range(db_video_list, video_time)
                video_list = list(set(video_list + matching_video))
                self.debug_core.log_key("----------------------")
                self.debug_core.log_key(video_list)
        
        self.debug_core.log_info("------ list of video used ------")
        self.debug_core.log_info(video_list)

        video_summary_list = []

        for video in video_list:
            _record_result = self.postgres_core.get_video_summary_record_by_name_db(video)
            video_summary_list.append(_record_result["summary"])

        self.debug_core.log_key("------ list of knowledge used ------")
        self.debug_core.log_info(video_summary_list)
        self.debug_core.log_info(object_db_list)

        # query = f"""
        #         {_question}

        #         make the answer concise
        #         the information below serve as additional context information, no neccary have to use it
        #         List of object Info:
        #         {object_db_list}
        #         List of context info:
        #         {video_summary_list}
        #         """
        
        query = f"""
                You are the robot, Given the information below, answer the question if it can be answered, otherwise, simply return "None": "{_question}"

                List of object Info:
                {object_db_list}
                List of context info:
                {video_summary_list}
                """
        
        data = {
            'question': query,
            # 'schema': utilities_core.basemodel_to_json(lmm_function_pool.ListItems),
            # 'tag': 'tag',
            # 'base_img': [base_img_1, base_img_2],
            # 'tools': [tools_1, tools_2],
            # 'model': "claude"
        }
        results = utilities_core.rest_post_request(data, "http://192.168.1.182:5001/mechlmm/chat")
        

        self.debug_core.log_key("------ chat_text_knowledge result ------")
        self.debug_core.log_info(results["result"])

        # mechllm_core.chat_video("../output/videos/output_video_1727316267.mp4", "what color is the desk")
        video_detail_summary_list = []
        # print(type(results))
        if ("None" in results["result"]):
            for video in video_list:
                video_detail_summary_list.append(self.chat_video("../output/videos/" + video, _question))

        
            # results, _ = self.chat_text(f"""
            #                             given the information below, answer the question in summary: "{_question}"

            #                             Detail context analyze:
            #                             {video_detail_summary_list}
            #                             List of context info:
            #                             {object_db_list}
            #                             {video_summary_list}
            #                             """
            #                             )
            
            query = f"""
                    given the information below, answer the question in summary: "{_question}"

                    Detail context analyze:
                    {video_detail_summary_list}
                    List of context info:
                    {object_db_list}
                    {video_summary_list}
                    """
            
            data = {
                'question': query,
                # 'schema': utilities_core.basemodel_to_json(lmm_function_pool.ListItems),
                # 'tag': 'tag',
                # 'base_img': [base_img_1, base_img_2],
                # 'tools': [tools_1, tools_2],
                # 'model': "claude"
            }
            results = utilities_core.rest_post_request(data, "http://192.168.1.182:5001/mechlmm/chat")
            
            self.debug_core.log_key("------ 2222 chat_text_knowledge result ------")
            self.debug_core.log_info(results)
        

        return results
    
    def chat_video(self, _video_path, _question, _start_time = 0, _end_time = 0, _interval_time = 5):
        conversion_list = []

        cap = cv2.VideoCapture(_video_path)
        fps = cap.get(cv2.CAP_PROP_FPS)
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        
        start_frame = int(_start_time * fps)
        end_frame = 0

        if(end_frame == 0):
            end_frame = total_frames
        else:
            end_frame = int(_end_time * fps)
        
        
        end_frame = min(end_frame, total_frames - 1)
        
        cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)
        
        current_frame = start_frame
        
        while current_frame <= end_frame:
            ret, frame = cap.read()
            if not ret:
                break
            
            if (
                current_frame == start_frame or 
                (current_frame - start_frame) % (int(_interval_time * fps)) == 0 or 
                (current_frame == end_frame)
            ):

                image_url = utilities_core.opencv_frame_to_base64(frame)
                # base64_image = utilities_core.opencv_frame_to_base64(frame)
                # image_url = f"data:image/jpeg;base64,{base64_image}"
                
                # frame_summary, _ = self.chat_img(f"""
                #                                     answer the question if the question can be answered, otherwise, simply return "None"
                                                    
                #                                     Question:
                #                                     {_question}
                #                                 """,
                #                               image_url)
                

                query = f"""
                            answer the question if the question can be answered, otherwise, simply return "None"
                            
                            Question:
                            {_question}
                        """
                
                data = {
                    'question': query,
                    # 'schema': utilities_core.basemodel_to_json(lmm_function_pool.ListItems),
                    # 'tag': 'tag',
                    'base_img': [image_url],
                    # 'tools': [tools_1, tools_2],
                    # 'model': "claude"
                }
                results = utilities_core.rest_post_request(data, "http://192.168.1.182:5001/mechlmm/chat")

                self.debug_core.log_key("------ llm video single frame analyzer output ------")
                self.debug_core.log_info(int(current_frame/fps))
                self.debug_core.log_info(results["result"])
                self.debug_core.log_key("^^^^^^^^^^^^^^ llm video single frame analyzer output ^^^^^^^^^^^^^^")
                
                conversion_list.append(results["result"])
            
            current_frame += 1
        
        cap.release()

        self.debug_core.log_key("^^^^^^^^^^^^^^ ^^^^^^^^^^^^^^")
        print(conversion_list)
        # video_summary, _ = self.chat_text(
        #     f"""
        #         given the context below, answer the question in summary:
                
        #         Question:
        #         "{_question}"
                
        #         Context:
        #         {conversion_list}
        #     """
        # )

        query = f"""
                    given the context below, answer the question in summary:
                    
                    Question:
                    "{_question}"
                    
                    Context:
                    {conversion_list}
                """
        
        data = {
            'question': query,
            # 'schema': utilities_core.basemodel_to_json(lmm_function_pool.ListItems),
            # 'tag': 'tag',
            # 'base_img': [base_img_1, base_img_2],
            # 'tools': [tools_1, tools_2],
            # 'model': "claude"
        }
        results = utilities_core.rest_post_request(data, "http://192.168.1.182:5001/mechlmm/chat")

        self.debug_core.log_info("------ ------------------------- ------")
        self.debug_core.log_info("------ llm video analyzer output ------")
        self.debug_core.log_info(results["result"])

        return results["result"]
if __name__ == "__main__":
    bot = MechLMMChatBot()
    # bot.run()
    bot.run_terminal()
