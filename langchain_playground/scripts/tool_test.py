from langchain_community.callbacks import get_openai_callback
from langchain_core.messages import HumanMessage

from langchain_openai import ChatOpenAI
from langchain_ollama import ChatOllama
from langchain_google_genai import ChatGoogleGenerativeAI

from dotenv import load_dotenv
load_dotenv()

import time

import utilities_core


# model = ChatOpenAI(
#     model="gpt-4o-mini",
#     temperature=0,
# )

model = ChatOllama(
    base_url="http://localhost:11434",
    model="llama3.2:1b",
    temperature=0,
)

# model = ChatGoogleGenerativeAI(
#     model="gemini-1.5-pro",
#     temperature=0,
# )


json_schema = {
    "title": "image_analysis",
    "description": "give a detail analysis of what happen in the image",
    "type": "object",
    "properties": {
        "objects": {
            "type": "object",
            "description": "the list of objects",
            "properties": {
                "name": {
                    "type": "string",
                    "description": "the name of the object detected",
                },
                "position": {
                    "type": "array",
                    "items": {
                        "type": "number"
                    },
                    "description": "the bounding box coordinate of the object detected, such as ['top_left_x', 'top_left_y', 'bottom_right_x', 'bottom_right_y']",
                },
                "features": {
                    "type": "array",
                    "items": {
                        "type": "string"
                    },
                    "description": "the key features of the object detected",
                },
            }
        },
        "description": {
            "type": "string",
            "description": "Overall description of what is seen in the image"
        }
    },
    "required": ["objects", "description"]
}

structured_llm = model.with_structured_output(json_schema)

base64_image = utilities_core.jpg_to_base64("../data/images/art_1.jpg")

image_url = f"data:image/jpeg;base64,{base64_image}"

# for i in range(20):
begin_time = time.time()

with get_openai_callback() as cb:
    # answer = structured_llm.invoke("analysis this image, and give me a detail break down of list of objects in the image: 'https://media.cnn.com/api/v1/images/stellar/prod/220825103859-02-body-world-coolest-streets-macdougal-street.jpg?q=w_1110,c_fill'")
    answer = structured_llm.invoke(
        [
            HumanMessage(
                content=[
                    {
                        "type": "text", 
                        "text": "analysis this image, and give me a detail break down of list of objects in the image"
                    },
                    {
                        "type": "image_url",
                        "image_url": {"url": image_url}
                    },
                ]
            )
        ]
    )
    
    print("=-=-=-=-")
    print(time.time() - begin_time)
    print(answer)
    print(answer["objects"])
    print(answer["description"])
    print(cb)