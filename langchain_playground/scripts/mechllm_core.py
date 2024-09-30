from langchain_community.callbacks import get_openai_callback
from langchain_core.messages import HumanMessage, ToolMessage
from langchain_core.tools import tool

from pydantic import BaseModel
from langchain.tools import StructuredTool

from langchain_openai import ChatOpenAI
from langchain_ollama import ChatOllama
from langchain_google_genai import ChatGoogleGenerativeAI

from dotenv import load_dotenv
load_dotenv()

import time

import cv2

from debug_core import DebugCore
from postgres_core import PostgresCore
import utilities_core
import os

import json

# from geometry_msgs.msg import Pose

# Define the Position class
class Position(BaseModel):
    x: float
    y: float
    z: float

# Define the Orientation class
class Orientation(BaseModel):
    x: float
    y: float
    z: float
    w: float

class PoseData(BaseModel):
    position: Position
    orientation: Orientation

class MechLLMCore:
    def __init__(self, data_path = "../output"):
        self.ollama_model = ChatOllama(
            # base_url="http://192.168.1.182:11434",
            base_url="http://192.168.1.182:11434",
            model="llama3.2",
            temperature=0,
        )

        self.open_ai_model = ChatOpenAI(
            model="gpt-4o-mini",
            temperature=0,
        )
        
        self.debug_core = DebugCore()
        self.debug_core.verbose = 3

        self.postgres_core = PostgresCore(False)
        # self.postgres_core = None
        self.init_data_path(data_path)

        tools = [self.add, self.multiply, self.move_base, self.move_arm_end_effector]
        # self.llm_with_tools = self.open_ai_model.bind_tools(tools)
        self.llm_with_tools = self.open_ai_model.bind_tools([self.move_base], tool_choice="any")
        # always_multiply_llm = llm.bind_tools([multiply], tool_choice="multiply")
    
    def init_data_path(self, _data_path):
        VIDEOS_OUTPUT_PATH = os.path.join(_data_path, "videos")
        IMAGES_OUTPUT_PATH = os.path.join(_data_path, "images")

        os.makedirs(VIDEOS_OUTPUT_PATH, exist_ok=True)
        os.makedirs(IMAGES_OUTPUT_PATH, exist_ok=True)

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
    

    def chat_tool(self, _question):
        self.debug_core.log_info("------ llm tool calling ------")
        _result = self.llm_with_tools.invoke(_question)
        print(_result.tool_calls)
        self.debug_core.log_info("------ ------")

        return _result


    @tool
    def add(a: int, b: int) -> int:
        """Adds a and b.

        Args:
            a: first int
            b: second int
        """
        return a + b


    @tool
    def multiply(a: int, b: int) -> int:
        """Multiplies a and b.

        Args:
            a: first int
            b: second int
        """
        return a * b
    
    # @tool
    # def move_base(target_location: Pose) -> bool:
    #     """move the robot to the target location

    #     Args:
    #         target_location: the target location robot is gonna move to
    #     """

    #     return True
    
    @tool
    def move_arm_end_effector(target_location: str) -> bool:
        """move the arm of robot to the target location

        Args:
            target_location: the target location arm is gonna reach to
        """

        return True
    
    @tool
    def move_base(target_direction: str) -> bool:
        """move the robot to the target direction for 1cm

        Args:
            target_location: the target direction robot is gonna move to, it only have four type of value "forward", "left", "right", "back", "stop"
        """
        
        print("move_base....")
        return True

    @tool
    def arrived_stop() -> bool:
        """ stop the robot as it arrived its target position

        """
        
        print("arrived_stop....")
        return True
    

def move_base(target_direction: str):
    print(target_direction)

if __name__ == '__main__':
    mechllm_core = MechLLMCore()

    # current_pose = PoseData()
    # target_pose = PoseData()

    # current_pose.position.x = 0.0
    # current_pose.position.y = 0.0

    # current_pose.orientation.w = 1.0

    # target_pose.position.x = 3.1
    # target_pose.position.y = -3.0
    # target_pose.orientation.z = -0.15
    # target_pose.orientation.w = 0.98

    # pose_dict = {
    #     "position": {
    #         "x": current_pose.position.x,
    #         "y": current_pose.position.y,
    #         "z": current_pose.position.z
    #     },
    #     "orientation": {
    #         "x": current_pose.orientation.x,
    #         "y": current_pose.orientation.y,
    #         "z": current_pose.orientation.z,
    #         "w": current_pose.orientation.w
    #     }
    # }

    # _result = mechllm_core.chat_tool(
    #     f"""
    #         given the data below, the "arm_end_effector" is attached to the "robot_base" and the position is relative position to the "robot_base". 

    #         if I want to grab the "target_obj", what would the sequence of function call, you can call single function multiple time

    #         current robot pose: {current_pose}

    #         target robot pose: {target_pose}
    #     """
    # )
    # print(_result)

    # tool_call = _result.tool_calls[0]
    # print(tool_call)
    # selected_tool = {
    #         "add": mechllm_core.add, "multiply": mechllm_core.multiply, "move_base": mechllm_core.move_base, "move_arm_end_effector": mechllm_core.move_arm_end_effector, "arrived_stop": mechllm_core.arrived_stop
    #     }[tool_call["name"].lower()]

    # print(selected_tool)
    # selected_tool.invoke(tool_call["args"])

    # print("--")

    # tt = move_base

    # tt(tool_call["args"]["target_direction"])


    # current_pose = {
    #     "position": {
    #         "x": 0.0,
    #         "y": 0.0,
    #         "z": 0.0,
    #     },
    #     "orientation": {
    #         "x": 0.0,
    #         "y": 0.0,
    #         "z": 0.0,
    #         "w": 1.0,
    #     }
    # }

    # target_pose = {
    #     "position": {
    #         "x": 2.1,
    #         "y": -1.0,
    #         "z": 0.0,
    #     },
    #     "orientation": {
    #         "x": 0.0,
    #         "y": 0.0,
    #         "z": -0.15,
    #         "w": 0.98,
    #     }
    # }

    # # print(mechllm_core.chat_tool("What is 3 * 12? Also, what is 11 + 49?"))

    # _result =(mechllm_core.chat_tool(
    #     """
    #         given the data below, the "arm_end_effector" is attached to the "robot_base" and the position is relative position to the "robot_base". 

    #         if I want to grab the "target_obj", what would the sequence of function call, you can call single function multiple time

    #         {
    #             "robot_base": {
    #                 "position": {
    #                     "x": 3,
    #                     "y": 2,
    #                     "z": 2
    #                 }
    #             },
    #             "arm_end_effector": {
    #                 "position": {
    #                     "x": 0,
    #                     "y": 1,
    #                     "z": 1
    #                 }
    #             }
    #         }

    #         {
    #             "target_obj":{
    #                 "position": {
    #                     "x": 5,
    #                     "y": 2,
    #                     "z": 3
    #                 }
    #             }
    #         }
    #     """
    # ))

    # _functions[0].invoke(_args[0])

    # function_list = []
    # function_arg_list = []
    # for tool_call in _result.tool_calls:
    #     selected_tool = {"add": self.add, "multiply": self.multiply, "move_base": self.move_base, "move_arm_end_effector": self.move_arm_end_effector, "arrived_stop": self.arrived_stop}[tool_call["name"].lower()]
    #     function_list.append(selected_tool)
    #     function_arg_list.append(tool_call["args"])
    #     # tool_output = selected_tool.invoke(tool_call["args"])
    #     # print(ToolMessage(tool_output, tool_call_id=tool_call["id"]))
    #     # print("---")

    # print(function_list)
    # print(function_arg_list)

    # return function_list, function_arg_list
    
    # mechllm_core.chat_text_knowledge("what is the guy doing")
    # mechllm_core.chat_text_knowledge("what is in front of the wall")
    # mechllm_core.chat_video("../output/videos/output_video_1727316267.mp4", "what color is the desk")

    # print(mechllm_core.chat_text("""
    #                             how are you"
    #                             """))
    
    json_schema = {
        "title": "story",
        "description": "give me a break down of story",
        "type": "object",
        "properties": {
            "names": {
                "type": "array",
                "items": {
                    "type": "string"
                },
                "description": "list of name that shows up in the story",
            },
            "locations": {
                "type": "string",
                "description": "list of locations that shows up in the story"
            }
        },
        "required": ["names", "locations"]
    }
    
    print(mechllm_core.chat_text("""
                                write me a short story with names and location"
                                """, json_schema))
    
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
    # mechllm_core.chat_img(base64_image)

    # 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