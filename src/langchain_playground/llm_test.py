from langchain_community.llms import Ollama
from langchain.chains import LLMChain


import uuid

from langchain_community.llms import Ollama

from langchain_core.prompts import (
    ChatPromptTemplate,
    HumanMessagePromptTemplate,
    MessagesPlaceholder,
)

from langchain_core.messages import SystemMessage, AIMessage, HumanMessage
from langchain_postgres import PostgresChatMessageHistory
import psycopg

conn_info = "postgresql://postgres:qwepoi123@localhost:5432/llm-smart-home"
sync_connection = psycopg.connect(conn_info)

table_name = "chat_history"
PostgresChatMessageHistory.create_tables(sync_connection, table_name)

session_id = str(uuid.uuid4())

# Initialize the chat history manager
chat_history = PostgresChatMessageHistory(
    table_name,
    session_id,
    sync_connection=sync_connection
)

ollama_llm = Ollama(base_url="http://131.123.41.132:11434", model="llava:34b")

template_messages = [
    SystemMessage(content="You are a helpful assistant."),
    MessagesPlaceholder(variable_name="chat_history"),
    HumanMessagePromptTemplate.from_template("{text}"),
]
prompt_template = ChatPromptTemplate.from_messages(template_messages)

llm_chain = LLMChain(
    llm=ollama_llm,
    memory=chat_history,
)

llm_chain.invoke("hi, how are you")


# # Add messages to the chat history
# chat_history.add_messages([
#     SystemMessage(content="1111"),
#     AIMessage(content="222"),
#     HumanMessage(content="3333"),
# ])

# print(chat_history.messages)