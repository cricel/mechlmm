from langchain_community.llms import Ollama
from langchain_openai import ChatOpenAI

from langchain.chains import LLMChain
from langchain.memory import ConversationBufferMemory
from langchain.schema.messages import HumanMessage

from langchain_core.messages import SystemMessage
from langchain_core.prompts import (
    ChatPromptTemplate,
    HumanMessagePromptTemplate,
    MessagesPlaceholder,
)
from langchain_core.runnables import RunnableSequence

import uuid
from langchain_postgres import PostgresChatMessageHistory
import psycopg


import os
import base64

class LLMCore:
    def __init__(self, llm_server):

        if(llm_server == "openai"):
            os.environ["OPENAI_API_KEY"] = "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"
            self.llm_base = ChatOpenAI(model_name="gpt-4-vision-preview", max_tokens=1024)
        elif(llm_server == "ollama"):
            self.llm_base = Ollama(base_url="http://131.123.41.132:11434", model="llava:34b")
        else:
            self.llm_base = Ollama(base_url="http://131.123.41.132:11434", model="llava:34b")


        conn_info = "postgresql://postgres:qwepoi123@localhost:5432/llm-smart-home"
        sync_connection = psycopg.connect(conn_info)
        table_name = "chat_history"
        PostgresChatMessageHistory.create_tables(sync_connection, table_name)

        # session_id = str(uuid.uuid4())
        session_id = "4eaab6aa-aefe-4ec7-a311-eb4acbd4b34a"
        print("Your Current Conversion Session ID: " + session_id)

        chat_history = PostgresChatMessageHistory(
            table_name,
            session_id,
            sync_connection=sync_connection
        )

        prompt_template = ChatPromptTemplate.from_messages(
            [
                SystemMessage(
                    content="You are a smart home assistant AI and Your Name is Adam"
                ),
                MessagesPlaceholder(
                    variable_name="chat_history"
                ),
                HumanMessagePromptTemplate.from_template(
                    "{human_input}"
                ),
            ]
        )

        memory = ConversationBufferMemory(
            memory_key="chat_history", 
            return_messages=True,
            chat_memory=chat_history
            )

        self.llm_chain = LLMChain(
            llm=self.llm_base,
            prompt=prompt_template,
            memory=memory,
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
    llm_core = LLMCore("ollama")
    print(llm_core.ask_txt("what is my name, and what do I like"))
    # print("------")
    # print(llm_core.ask_txt("my name is shawn"))
    # print("------")
    # print(llm_core.ask_txt("I like dog and cat"))
    # print("------")
    # print(llm_core.ask_txt("what is my name"))
    # print("------")
    # print(llm_core.ask_txt("what do I like"))
    # print("------")
    # print(llm_core.ask_img("what is in the image", "./messy_room.jpg"))