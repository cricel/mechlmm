from langchain_community.llms import Ollama

from langchain.chains import LLMChain
from langchain.memory import ConversationBufferMemory
from langchain.schema.messages import HumanMessage

from langchain_core.messages import SystemMessage
from langchain_core.prompts import (
    ChatPromptTemplate,
    HumanMessagePromptTemplate,
    MessagesPlaceholder,
)

from langchain_community.chat_message_histories import (
    UpstashRedisChatMessageHistory,
)

from langchain_openai import ChatOpenAI

import os
import base64

class LLMCore:
    def __init__(self):
        URL = "https://working-tadpole-41672.upstash.io"
        TOKEN = "AaLIAAIncDFjMzRiOGEwOWZkMzk0ZWY5YmQ4YjZlNTY0Y2FhMjM2NnAxNDE2NzI"

        self.history = UpstashRedisChatMessageHistory(
            url=URL,
            token=TOKEN,
            ttl=0, 
            session_id="my-test-session"
        )

        self.ollama_llm = Ollama(base_url="http://131.123.41.132:11434", model="llava:34b")

        # os.environ["OPENAI_API_KEY"] = "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"
        # self.ollama_llm = ChatOpenAI(model_name="gpt-4-vision-preview", max_tokens=1024)


        self.prompt = ChatPromptTemplate.from_messages(
            [
                SystemMessage(
                    content="You are a smart home assistant AI and Your Name is Adam, you are having a conversion with the human"
                ),
                MessagesPlaceholder(
                    variable_name="chat_history"
                ),
                HumanMessagePromptTemplate.from_template(
                    "{human_input}"
                ),
            ]
        )

        self.memory = ConversationBufferMemory(
            memory_key="chat_history", 
            return_messages=True,
            chat_memory=self.history
            )

        self.llm_chain = LLMChain(
            llm=self.ollama_llm,
            prompt=self.prompt,
            memory=self.memory,
        )
        

    def ask_txt(self, _msg):
        return self.llm_chain.invoke(_msg)["text"]
    
    def ask_img(self, _msg, _img_path):
        image = self.encode_image(_img_path)

        return self.llm_chain.invoke(
            [
                HumanMessage
                (
                    content =
                    [
                        {
                            "type": "text", 
                            "text": "what is happening in this image"
                        },
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpg;base64,{image}"
                            },
                        },
                    ]
                )
            ]
        )["text"]

    def encode_image(self, _image_path):
        with open(_image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode('utf-8')
    
if __name__ == "__main__":
    llm_core = LLMCore()
    print(llm_core.ask_txt("what is my name"))
    # print(llm_core.ask_img("what is in the image", "./messy_room.jpg"))