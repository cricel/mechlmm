from langchain_core.messages import HumanMessage

from langchain_openai import ChatOpenAI
from langchain_ollama import ChatOllama
from langchain_google_genai import ChatGoogleGenerativeAI
from langchain_anthropic import ChatAnthropic

from mechlmm_py import DebugCore

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

if __name__ == '__main__':
    pass