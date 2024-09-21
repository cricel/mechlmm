from langchain_community.callbacks import get_openai_callback
from langchain_core.messages import HumanMessage

from langchain_openai import ChatOpenAI
from langchain_ollama import ChatOllama
from langchain_google_genai import ChatGoogleGenerativeAI

from dotenv import load_dotenv
load_dotenv()

import time

from debug_core import DebugCore
from postgres_core import PostgresCore
import utilities_core

class MechLLMCore:
    def __init__(self):
        self.ollama_model = ChatOllama(
            base_url="http://192.168.1.182:11434",
            model="llama3.1",
            temperature=0,
        )

        self.open_ai_model = ChatOpenAI(
            model="gpt-4o-mini",
            temperature=0,
        )
        
        self.debug_core = DebugCore()
        self.debug_core.verbose = 3

        self.postgres_core = PostgresCore(False)

    def chat(self):
        pass
    
    def chat_text_knowledge(self, _question):
        self.debug_core.log_info("------ chat_text_knowledge ------")
        db_item_list = self.postgres_core.get_objects_map_name_list_db()
        db_video_list = self.postgres_core.get_video_summary_list_db()

        # print(db_video_list)
        # print(db_item_list)
        # time_data = [1726895700, 1726895703]
        # matching_video = self.find_video_in_range(db_video_list, time_data)
        # print(matching_video)


        results, _ = mechllm_core.chat_text(f"""
                                    Dont answer the question, just parser the following question and find the similar item in the list provided: 
                                    {_question}
                                
                                    {db_item_list}

                                    return the excat name of the matching item in provided list as array,
                                    Only return the JSON array, no need for the reasoning or any additional content.
                                    
                                    """
                                    )
        
        self.debug_core.log_info("------ find target object in db ------")
        self.debug_core.log_info(results)
        output_results = utilities_core.llm_output_list_cleaner(results)

        video_list = []
        for result in output_results:
            _record_result = self.postgres_core.get_objects_map_record_by_name_db(result)
            for video_time in _record_result["reference_videos"]:
                matching_video = self.find_video_in_range(db_video_list, video_time)
                video_list = list(set(video_list + matching_video))
        
        self.debug_core.log_info("------ list of video used ------")
        self.debug_core.log_info(video_list)

        video_summary_list = []

        for video in video_list:
            _record_result = self.postgres_core.get_video_summary_record_by_name_db(video)
            video_summary_list.append(_record_result["summary"])

        self.debug_core.log_info("------ list of knowledge used ------")
        self.debug_core.log_info(video_summary_list)

        results, _ = mechllm_core.chat_text(f"""
                                    given the context below, answer the question in summary: "{_question}"

                                    {video_summary_list}
                                    """
                                    )
        
        self.debug_core.log_key("------ chat_text_knowledge result ------")
        self.debug_core.log_info(results)

        return results


    
    def find_video_in_range(self, video_data, time_data):
        matching_video_list = []
        start_time, end_time = time_data
        
        for video in video_data:
            video_start_time = video[2]
            video_end_time = video[3]
            
            if video_start_time <= end_time and video_end_time >= start_time:
                matching_video_list.append(video[1])
        
        return matching_video_list
        

    def chat_text(self, _question, _json_schema = None, _tag = None):
        text_llm = None

        if(_json_schema):
            text_llm = self.ollama_model.with_structured_output(_json_schema)
        else:
            text_llm = self.ollama_model

        result = text_llm.invoke(
            [
                HumanMessage(
                    content=[
                        {
                            "type": "text", 
                            "text": _question
                        },
                    ]
                )
            ]
        )

        self.debug_core.log_info("------ chat_text output ------")
        self.debug_core.log_info(result)

        if(_json_schema):
            return result, _tag
        else:
            return result.content, _tag

    def chat_img(self, _question, _base_img, _json_schema = None, _tag = None):
        text_llm = None

        if(_json_schema):
            text_llm = self.open_ai_model.with_structured_output(_json_schema)
        else:
            text_llm = self.open_ai_model

        result = text_llm.invoke(
            [
                HumanMessage(
                    content=[
                        {
                            "type": "text", 
                            "text": _question
                        },
                        {
                            "type": "image_url",
                            "image_url": {"url": _base_img}
                        },
                    ]
                )
            ]
        )

        self.debug_core.log_info("------ chat_img output ------")
        self.debug_core.log_info(result)

        return result, _tag

if __name__ == '__main__':
    mechllm_core = MechLLMCore()

    mechllm_core.chat_text_knowledge("what is the guy doing")

    # print(mechllm_core.chat_text("""
    #                             how are you"
    #                             """))
    
    # json_schema = {
    #     "title": "story",
    #     "description": "give me a break down of story",
    #     "type": "object",
    #     "properties": {
    #         "names": {
    #             "type": "array",
    #             "items": {
    #                 "type": "string"
    #             },
    #             "description": "list of name that shows up in the story",
    #         },
    #         "locations": {
    #             "type": "string",
    #             "description": "list of locations that shows up in the story"
    #         }
    #     },
    #     "required": ["names", "locations"]
    # }
    
    # print(mechllm_core.chat_text("""
    #                             write me a short story with names and location"
    #                             """, json_schema))
    
    # db_objects = ["wall","person", "background", "phone", "cloth"]

    # json_schema = {
    #     "title": "find_item_parser",
    #     "description": "return the similar items that mentioned in the question",
    #     "type": "object",
    #     "properties": {
    #         "objects": {
    #             "type": "array",
    #             "items": {
    #                 "type": "string"
    #             },
    #             "description": "the objects that mentioned",
    #         },
    #     },
    #     "required": ["objects"]
    # }

    # print(mechllm_core.chat_text(f"""
    #                             Dont answer the question, just parser the following question and find the similar item in the list provided: 
    #                             'I am looking for my phone '
                            
    #                             {db_objects}

    #                             return the excat name of the matching item in provided list as array,
    #                             Only return the JSON array, no need for the reasoning or any additional content.
                                
    #                             """
    #                             ))

    # base64_image = utilities_core.jpg_to_base64("../data/images/art_1.jpg")
    # ollama_core.chat_img(base64_image)

    # ollama_core.video_summary("../data/videos/fast_and_furious.mp4", 0, 30, 10)
    # ollama_core.video_summary_video_feed_storage(7,15)

    


    # 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