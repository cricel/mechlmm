from dotenv import load_dotenv
load_dotenv()

# from mechlmm_py import MechLMMCore, utilities_core, lmm_function_pool

# mechlmm_core = MechLMMCore()

# base64_image = utilities_core.jpg_to_base64("./test.jpg")
# print(mechlmm_core.chat_img("give me a list of objects in the list, and its bounding box", base64_image))

# mechlmm_core.chat_tool([lmm_function_pool.manipulation,lmm_function_pool.navigation], "go back home")

# ### Chat Text

# print(mechlmm_core.chat_text("""
#                             How are you
#                             """))

# json_schema = {
#     "title": "joke",
#     "description": "Joke to tell user.",
#     "type": "object",
#     "properties": {
#         "setup": {
#             "type": "string",
#             "description": "The setup of the joke",
#         },
#         "punchline": {
#             "type": "string",
#             "description": "The punchline to the joke",
#         },
#         "rating": {
#             "type": "integer",
#             "description": "How funny the joke is, from 1 to 10",
#             "default": None,
#         },
#     },
#     "required": ["setup", "punchline"],
# }

# print(mechlmm_core.chat_text("""
#                             write me a short story with names and location"
#                             """, json_schema))


# #### Chat Image

# # base64_image = utilities_core.jpg_to_base64("../data/images/art_1.jpg")
# # mechllm_core.chat_img(base64_image)



# from langchain_google_vertexai import VertexAI

# # To use model
# model = VertexAI(model_name="gemini-pro")
# message = "how are you"
# print(model.invoke(message))
# message = "how are you"
# print(model.invoke(message))
# message = "how are you"
# print(model.invoke(message))



from langchain_google_genai import ChatGoogleGenerativeAI

llm = ChatGoogleGenerativeAI(model="gemini-1.5-pro")
# print(llm.invoke("Write me a ballad about LangChain"))


# from typing import Optional

# from pydantic import BaseModel, Field


# json_schema = {
#     "title": "joke",
#     "description": "Joke to tell user.",
#     "type": "object",
#     "properties": {
#         "setup": {
#             "type": "string",
#             "description": "The setup of the joke",
#         },
#         "punchline": {
#             "type": "string",
#             "description": "The punchline to the joke",
#         },
#         "rating": {
#             "type": "integer",
#             "description": "How funny the joke is, from 1 to 10",
#             "default": None,
#         },
#     },
#     "required": ["setup", "punchline"],
# }

# class Joke(BaseModel):
#     '''Joke to tell user.'''

#     setup: str = Field(description="The setup of the joke")
#     punchline: str = Field(description="The punchline to the joke")
#     rating: Optional[int] = Field(description="How funny the joke is, from 1 to 10")


# structured_llm = llm.with_structured_output(Joke)
# _result = structured_llm.invoke("Tell me a joke about cats")

# print(_result)
# print(_result.punchline)


import base64
import httpx
from langchain_core.messages import HumanMessage

image_url = "https://upload.wikimedia.org/wikipedia/commons/thumb/d/dd/Gfp-wisconsin-madison-the-nature-boardwalk.jpg/2560px-Gfp-wisconsin-madison-the-nature-boardwalk.jpg"
image_data = base64.b64encode(httpx.get(image_url).content).decode("utf-8")
message = HumanMessage(
    content=[
        {"type": "text", "text": "Return bounding boxes around each object, for each one return [ymin, xmin, ymax, xmax]"},
        {
            "type": "image_url",
            "image_url": {"url": f"data:image/jpeg;base64,{image_data}"},
        },
    ]
)
ai_msg = llm.invoke([message])
print(ai_msg.content)