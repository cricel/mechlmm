from langchain_core.messages import HumanMessage

from langchain_openai import ChatOpenAI
from langchain_ollama import ChatOllama
from langchain_google_genai import ChatGoogleGenerativeAI
from langchain_anthropic import ChatAnthropic

from mechlmm_py import DebugCore, PostgresCore, lmm_function_pool, utilities_core

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
        self.postgres_core = PostgresCore(False, "localhost")

        self.mechlmm_model = self.gemini_model

        self.rulebook = ""
        with open('/Users/cricel/Documents/GitHub/llm-smart-home/mechlmm_server/mechlmm_server/rulebook.txt', 'r') as file:
            self.rulebook = file.read()

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
                return "Claude did not support image", _tag, "Error"
            self.mechlmm_model = self.claude_model
        else:
            self.mechlmm_model = self.gemini_model

        ## check schema
        if(_schema):
            if (_tools != None):
                return "schema did not work with tool output", _tag, "Error"
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
            return _result[0]["args"], _tag, return_type
        except:
            pass
        
        try:
            # tools
            return_type = "tools"
            if(_result.tool_calls != []):
                return _result.tool_calls, _tag, return_type
        except:
            pass

        try:
            # text
            return_type = "content"
            return _result.content, _tag, return_type
        except:
            pass
            
        try:
            # claude structure output
            return_type = "json"
            return _result, _tag, return_type
        except:
            pass
    
    def chat_datalog(self, _msg):
        self.debug_core.log_info("------ chat_datalog ------")
        db_item_list = self.postgres_core.get_table("data_log")

        query = f"""
                    {_msg}

                    answer the question base on the data provided, keep the answer concise

                    {self.rulebook}
                    {db_item_list}
                    """
        result, _, _ = self.chat(
            _question = query,
            _model = "claude"
        )
        
        return result

    def chat_knowledge(self, _question):
        self.debug_core.log_info("------ chat_text_knowledge ------")
        db_item_list = self.postgres_core.get_objects_map_name_list_db()
        db_video_list = self.postgres_core.get_video_summary_list_db()

        print(db_video_list)
        print(db_item_list)
        # time_data = [1726895700, 1726895703]
        # matching_video = self.find_video_in_range(db_video_list, time_data)
        # print(matching_video)

        query = f"""
                Dont answer the question, 
                if the question is related to robot status, then simply return the word "robot"
                otherwise, return "items" 
                """
        results, _, return_type = self.chat(_question = query, _schema = utilities_core.basemodel_to_json(lmm_function_pool.ListItems))
        

        query = f"""
                Dont answer the question, just parser the following question,
                if the question is related to robot status, then simply return the word "robot"

                otherwise, find the list of similar items in the list provided: 
                {_question}
            
                {db_item_list}

                return the exact name of the matching item in provided list as array, if none found, return None as item
                """
        results, _, return_type = self.chat(_question = query, _schema = utilities_core.basemodel_to_json(lmm_function_pool.ListItems))
        
        self.debug_core.log_info("------ find target object in db ------")
        self.debug_core.log_info(results)

            
        video_list = []
        object_db_list = []

        # if(results["items"][0] == "robot"):
        #     test_db = self.postgres_core.get_table("data_log")
        #     print(test_db)

        for result in results["items"]:
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

        results, _, return_type = self.chat(f"""
                                    {_question}

                                    make the answer concise
                                    the information below serve as additional context information, no neccary have to use it
                                    List of object Info:
                                    {object_db_list}
                                    List of context info:
                                    {video_summary_list}
                                    """
                                    )
        
        self.debug_core.log_key("------ chat_text_knowledge result ------")
        self.debug_core.log_info(results)

        # # mechllm_core.chat_video("../output/videos/output_video_1727316267.mp4", "what color is the desk")
        # video_detail_summary_list = []
        # # print(type(results))
        # if(results == "None"):
        #     for video in video_list:
        #         video_detail_summary_list.append(self.chat_video("../output/videos/" + video, _question))

        
        #     results, _ = self.chat_text(f"""
        #                                 given the information below, answer the question in summary: "{_question}"

        #                                 Detail context analyze:
        #                                 {video_detail_summary_list}
        #                                 List of context info:
        #                                 {object_db_list}
        #                                 {video_summary_list}
        #                                 """
        #                                 )
            
        #     self.debug_core.log_key("------ 2222 chat_text_knowledge result ------")
        #     self.debug_core.log_info(results)
        

        return results
if __name__ == '__main__':
    pass