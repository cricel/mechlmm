from langchain_core.messages import HumanMessage
from langchain_core.utils.function_calling import convert_to_openai_function

from langchain_openai import ChatOpenAI
from langchain_ollama import ChatOllama
from langchain_google_genai import ChatGoogleGenerativeAI
from langchain_anthropic import ChatAnthropic

import cv2

from .debug_core import DebugCore
from .postgres_core import PostgresCore
from . import utilities_core

class MechLMMCore:
    def __init__(self, data_path = "../output"):
        self.ollama_model = ChatOllama(
            base_url="http://192.168.1.182:11434",
            model="llama3.2",
            temperature=0,
        )

        self.openai_model = ChatOpenAI(
            model="gpt-4o-mini",
            temperature=0,
        )

        self.gemini_model = ChatGoogleGenerativeAI(
            model="gemini-1.5-pro",
            temperature=0,
            max_tokens=None,
            timeout=None,
            max_retries=2,
        )
        
        self.claude_model = ChatAnthropic(
            model='claude-3-opus-20240229'
        )

        self.debug_core = DebugCore()
        self.debug_core.verbose = 3

        self.postgres_core = PostgresCore(False)

        self.mechlmm_model = self.gemini_model

    def chat(self, _question, _tools = None, _base_imgs = None, _schema = None, _tag = None, _model = None):
        self.debug_core.log_info("------ llm chat calling ------")

        return_type = "json"
        lmm_model = None
        content_list = [
            {"type": "text", "text": _question}
        ]

        ## check model
        if(_model == "claude"):
            if (_base_imgs != None):
                return "Claude did not support image"
            self.mechlmm_model = self.claude_model
        else:
            self.mechlmm_model = self.gemini_model

        ## check schema
        if(_schema):
            if (_tools != None):
                return "schema did not work with tool output"
            lmm_model = self.mechlmm_model.with_structured_output(_schema)
        else:
            lmm_model = self.mechlmm_model
        
        ## check tools
        if(_tools): 
            # lmm_model = self.mechlmm_model.bind_tools(_tools)
            lmm_model = self.mechlmm_model.bind_tools(_tools, tool_choice="any")

        ## check imgs
        if(_base_imgs):
            for img_url in _base_imgs:
                content_list.append(
                    {
                        "type": "image_url",
                        "image_url": img_url
                    }
                )

        query = [
                HumanMessage(
                    content= content_list
                )
            ]
        
        
        _result = lmm_model.invoke(query)
        
        self.debug_core.log_info(_result)

        try:
            # schema
            return_type = "json"
            return _result[0]["args"], return_type
        except:
            pass
        
        try:
            # tools
            return_type = "tools"
            if(_result.tool_calls != []):
                return _result.tool_calls, return_type
        except:
            pass

        try:
            # text
            return_type = "content"
            return _result.content, return_type
        except:
            pass
            
        try:
            # claude structure output
            return_type = "json"
            return _result, return_type
        except:
            pass

    def chat_data(self, _question, _schema = None, _tag = None):
        self.debug_core.log_info("------ chat_data output ------")

        text_llm = None

        if(_schema):
            text_llm = self.claude_model.with_structured_output(_schema)
        else:
            text_llm = self.claude_model

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

        self.debug_core.log_info(result)

        if(_schema):
            return result[0]["args"], _tag
        else:
            return result.content, _tag

    def chat_text_knowledge(self, _question):
        self.debug_core.log_info("------ chat_text_knowledge ------")
        db_item_list = self.postgres_core.get_objects_map_name_list_db()
        db_video_list = self.postgres_core.get_video_summary_list_db()

        # print(db_video_list)
        # print(db_item_list)
        # time_data = [1726895700, 1726895703]
        # matching_video = self.find_video_in_range(db_video_list, time_data)
        # print(matching_video)


        results, _ = self.chat_text(f"""
                                    Dont answer the question, just parser the following question and find the list of similar items in the list provided: 
                                    {_question}
                                
                                    {db_item_list}

                                    return the exact name of the matching item in provided list as array, if none found, return None
                                    Only return the JSON array, no need for the reasoning or any additional content.
                                    
                                    """
                                    )
        
        self.debug_core.log_info("------ find target object in db ------")
        self.debug_core.log_info(results)
        output_results = utilities_core.llm_output_list_cleaner(results)

        video_list = []
        object_db_list = []
        for result in output_results:
            _record_result = self.postgres_core.get_objects_map_record_by_name_db(result)
            object_db_list.append(_record_result)
            self.debug_core.log_key("---++++++----")
            self.debug_core.log_key(result)
            self.debug_core.log_key(_record_result)
            for video_time in _record_result["reference_videos"]:
                matching_video = self.find_video_in_range(db_video_list, video_time)
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

        results, _ = self.chat_text(f"""
                                    given the information below, answer the question in summary if answer found, otherwise, simply return "None": "{_question}"

                                    List of object Info:
                                    {object_db_list}
                                    List of context info:
                                    {video_summary_list}
                                    """
                                    )
        
        self.debug_core.log_key("------ chat_text_knowledge result ------")
        self.debug_core.log_info(results)

        # mechllm_core.chat_video("../output/videos/output_video_1727316267.mp4", "what color is the desk")
        video_detail_summary_list = []
        # print(type(results))
        if(results == "None"):
            for video in video_list:
                video_detail_summary_list.append(self.chat_video("../output/videos/" + video, _question))

        
            results, _ = self.chat_text(f"""
                                        given the information below, answer the question in summary: "{_question}"

                                        Detail context analyze:
                                        {video_detail_summary_list}
                                        List of context info:
                                        {object_db_list}
                                        {video_summary_list}
                                        """
                                        )
            
            self.debug_core.log_key("------ 2222 chat_text_knowledge result ------")
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
        
    def chat_text(self, _question, _schema = None, _tag = None):
        self.debug_core.log_info("------ chat_text output ------")

        img_llm = None

        if(_schema):
            img_llm = self.mechlmm_model.with_structured_output(_schema)
        else:
            img_llm = self.mechlmm_model

        result = img_llm.invoke(
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

        self.debug_core.log_info(result)

        if(_schema):
            return result[0]["args"], _tag
        else:
            return result.content, _tag

    def chat_imgs(self, _question, _base_img_list, _schema = None, _tag = None):
        self.debug_core.log_info("------ chat_img output ------")

        img_llm = None

        if(_schema):
            img_llm = self.mechlmm_model.with_structured_output(_schema)
        else:
            img_llm = self.mechlmm_model

        content_list = [
            {"type": "text", "text": _question}
        ]
        for img_url in _base_img_list:
            content_list.append(
                {
                    "type": "image_url",
                    "image_url": img_url
                }
            )
            
        result = img_llm.invoke(
            [
                HumanMessage(
                    content = content_list
                )
            ]
        )

        self.debug_core.log_info(result)

        if(_schema):
            return result[0]["args"], _tag
        else:
            return result.content, _tag


    def chat_img(self, _question, _base_img, _schema = None, _tag = None):
        self.debug_core.log_info("------ chat_img output ------")

        img_llm = None

        if(_schema):
            img_llm = self.mechlmm_model.with_structured_output(_schema)
        else:
            img_llm = self.mechlmm_model

        result = img_llm.invoke(
            [
                HumanMessage(
                    content=[
                        {
                            "type": "text", 
                            "text": _question
                        },
                        {
                            "type": "image_url",
                            # "image_url": {"url": _base_img}
                            "image_url": _base_img
                        },
                    ]
                )
            ]
        )

        self.debug_core.log_info(result)

        if(_schema):
            return result[0]["args"], _tag
        else:
            return result.content, _tag

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

                base64_image = utilities_core.opencv_frame_to_base64(frame)
                image_url = f"data:image/jpeg;base64,{base64_image}"
                
                frame_summary, _ = self.chat_img(f"""
                                                    answer the question if the question can be answered, otherwise, simply return "None"
                                                    
                                                    Question:
                                                    {_question}
                                                """,
                                              image_url)

                self.debug_core.log_key("------ llm video single frame analyzer output ------")
                self.debug_core.log_info(int(current_frame/fps))
                self.debug_core.log_info(frame_summary.content)
                self.debug_core.log_key("^^^^^^^^^^^^^^ llm video single frame analyzer output ^^^^^^^^^^^^^^")
                
                conversion_list.append(frame_summary.content)
            
            current_frame += 1
        
        cap.release()

        self.debug_core.log_key("^^^^^^^^^^^^^^ ^^^^^^^^^^^^^^")
        print(conversion_list)
        video_summary, _ = self.chat_text(
            f"""
                given the context below, answer the question in summary:
                
                Question:
                "{_question}"
                
                Context:
                {conversion_list}
            """
        )
        self.debug_core.log_info("------ ------------------------- ------")
        self.debug_core.log_info("------ llm video analyzer output ------")
        self.debug_core.log_info(video_summary)

        return video_summary
    
    def chat_tool(self, _tools, _question, _base_img = None, _schema = None, _tag = None):
        self.debug_core.log_info("------ llm tool calling ------")
        
        query = None
        if(_base_img):
            query = [
                HumanMessage(
                    content=[
                        {
                            "type": "text", 
                            "text": _question
                        },
                        {
                            "type": "image_url",
                            # "image_url": {"url": _base_img}
                            "image_url": _base_img
                        },
                    ]
                )
            ]
        else:
            query = [
                HumanMessage(
                    content=[
                        {
                            "type": "text", 
                            "text": _question
                        }
                    ]
                )
            ]

        self.llm_with_tools = self.mechlmm_model.bind_tools(_tools)
        _result = self.llm_with_tools.invoke(query)
        
        self.debug_core.log_info(_result)

        if(_schema):
            return _result.tool_calls, _tag
        else:
            return _result.content, _tag

    def basemodel_to_json(self, _basemodel):
        return convert_to_openai_function(_basemodel)

if __name__ == '__main__':
    pass