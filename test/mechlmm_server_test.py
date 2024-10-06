import requests

from typing import Optional, List
from pydantic import BaseModel, Field
from langchain_core.utils.function_calling import convert_to_openai_function

from mechlmm_py import utilities_core, lmm_function_pool

# url = 'http://127.0.0.1:5000/mechlmm/chat/data'

# class ListItems(BaseModel):
#     '''list of items'''

#     items: List[str] = Field(..., description="the list of items")

# test1 = ["black hair", "face", "cloth"]
# test2 = ["head", "white cloth", "eyes"]

# dict_schema = convert_to_openai_function(ListItems)

# data = {
#     'question': 'how are you',
#     # 'schema': dict_schema,
#     'tag': 'value2',
# }


url = 'http://127.0.0.1:5001/mechlmm/chat/images'
image_url = utilities_core.jpg_to_base64("./test.jpg")
image_url_1 = utilities_core.jpg_to_base64("./test_1.jpg")

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

data = {
    'question': 'what is in the first image, what is in the second image',
    # 'schema': dict_schema,
    'tag': 'value2',
    'base_img': [image_url, image_url_1]
}


# mechlmm_core.chat_tool([lmm_function_pool.manipulation,
#                         lmm_function_pool.navigation
#                         ], 
#                        "go back home")

# url = 'http://127.0.0.1:5000/mechlmm/chat/tool'
# image_url = utilities_core.jpg_to_base64("./test.jpg")

# class Item(BaseModel):
#     '''detail break down of item'''

#     name: str = Field(..., description="the name of the object detected")
#     position: List[float] = Field(..., description="the bounding box coordinate of the object detected, such as ['top_left_x', 'top_left_y', 'bottom_right_x', 'bottom_right_y']")
#     features: List[str] = Field(..., description="the key features of the object detected")

# class ItemList(BaseModel):
#     '''a list of items description'''

#     objects: List[Item] = Field(..., description="the list of items")
#     description: str = Field(..., description="overall description of what is seen in the image")

# dict_schema = convert_to_openai_function(ItemList)

# dict_schema_1 = convert_to_openai_function(lmm_function_pool.manipulation)
# dict_schema_2 = convert_to_openai_function(lmm_function_pool.navigation)

# data = {
#     'question': 'pick up the laptop and go back home',
#     'schema': dict_schema,
#     'tag': 'value2',
#     "tools": [dict_schema_1, dict_schema_2],
#     'base_img': image_url
# }

response = requests.post(url, json=data)

if response.status_code == 200:
    
    _result = response.json()
    print('Success: \n', _result)
    print('Success: \n', _result[0][0])
else:
    print('Failed:', response.status_code, response.text)
