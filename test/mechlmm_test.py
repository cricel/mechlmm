from dotenv import load_dotenv
load_dotenv()
import google.generativeai as genai

from mechlmm_py import MechLMMCore, DebugCore, utilities_core, lmm_function_pool

from typing import Optional, List
from pydantic import BaseModel, Field
from langchain_core.utils.function_calling import convert_to_openai_function

mechlmm_core = MechLMMCore()
debug_core = DebugCore()


#### Chat Text
# debug_core.log_key("--------- Chat Text ---------")
# # print(mechlmm_core.chat_text("""
# #                             How are you
# #                             """))

# class ListItems(BaseModel):
#     '''list of items'''

#     items: List[str] = Field(..., description="the list of items")

# test1 = ["black hair", "face", "cloth"]
# test2 = ["head", "white cloth", "eyes"]

# dict_schema = convert_to_openai_function(ListItems)

# _result, _ = mechlmm_core.chat_text(f"""
#                             tell me a joke
#                             """, dict_schema, "test")

# print(_result)
# print(_result[0])
# print(_result[0].items)
# print(_result[0].type)
# print(_result[0].items["items"])



#### Chat Image
debug_core.log_key("--------- Chat Image ---------")

class Item(BaseModel):
    '''detail break down of item'''

    name: str = Field(..., description="the name of the object detected")
    position: List[float] = Field(..., description="the bounding box coordinate of the object detected, such as ['top_left_x', 'top_left_y', 'bottom_right_x', 'bottom_right_y']")
    features: List[str] = Field(..., description="the key features of the object detected")

class ItemList(BaseModel):
    '''a list of items description'''

    objects: List[Item] = Field(..., description="the list of items")
    description: str = Field(..., description="overall description of what is seen in the image")

dict_schema = convert_to_openai_function(ItemList)

image_url = utilities_core.jpg_to_base64("./test.jpg")
# _result, _ = mechlmm_core.chat_img("Return bounding boxes around the person, for each one return [ymin, xmin, ymax, xmax]", 
#                       image_url)

_result, _ = mechlmm_core.chat_img("Give me a list of items, return bounding boxes around each object, for each one return [ymin, xmin, ymax, xmax]", 
                      image_url, dict_schema, "test")

print(_result)
print(_result[0])
print(type(_result[0]))
try:
    print("1")
    print(_result[0]["args"])
except:
    pass

try:
    print("2")
    print(_result[0]["args"]["description"])
except:
    pass

try:
    print("3")
    print(_result[0]["args"]["objects"])
except:
    pass

try:
    print("4")
    print(_result[0]["args"]["objects"][0])
except:
    pass


#### Chat Tools
# debug_core.log_key("--------- Chat Tools ---------")

# mechlmm_core.chat_tool([lmm_function_pool.manipulation,
#                         lmm_function_pool.navigation
#                         ], 
#                        "go back home")