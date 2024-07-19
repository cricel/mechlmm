from langchain_community.llms import Ollama

from langchain.chains import LLMChain
from langchain.memory import ConversationBufferMemory
from langchain_core.prompts import PromptTemplate

from langchain_core.messages import SystemMessage
from langchain_core.prompts import (
    ChatPromptTemplate,
    HumanMessagePromptTemplate,
    MessagesPlaceholder,
)

from langchain_community.chat_message_histories import (
    UpstashRedisChatMessageHistory,
)

URL = "https://working-tadpole-41672.upstash.io"
TOKEN = "AaLIAAIncDFjMzRiOGEwOWZkMzk0ZWY5YmQ4YjZlNTY0Y2FhMjM2NnAxNDE2NzI"

history = UpstashRedisChatMessageHistory(
    url=URL,
    token=TOKEN,
    ttl=0, 
    session_id="my-test-session"
)


ollama_llm = Ollama(base_url="http://131.123.41.132:11434", model="llama3")

print(ollama_llm.invoke("hi"))

print("__________---------_ ____________")

prompt = ChatPromptTemplate.from_messages(
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

memory = ConversationBufferMemory(
    memory_key="chat_history", 
    return_messages=True,
    chat_memory=history
    )

llm_chain = LLMChain(
    llm=ollama_llm,
    prompt=prompt,
    memory=memory,
)

resp_1 = llm_chain.invoke("hi")
print(type(resp_1))
print(resp_1["text"])
print(" 1 ========================================")

resp_0 = llm_chain.invoke("what is my name")
print(resp_0["text"])
print(" 2 ========================================")

resp_2 = llm_chain.invoke("My name is shawn")
print(resp_2["text"])
print(" 3 ========================================")

resp_3 = llm_chain.invoke("what is my name")
print(resp_3["text"])
print(" 4 ========================================")

resp_4 = llm_chain.invoke("what day is today")
print(resp_4["text"])
print(" 5 ========================================")
