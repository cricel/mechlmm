import random
from flask import Flask, request, jsonify
from langchain_anthropic import ChatAnthropic
from langchain.chains import LLMChain
from langchain.prompts import PromptTemplate

app = Flask(__name__)

# Set a fixed seed for reproducibility
seed_value = 42
random.seed(seed_value)

# Claude API Key
api_key = ""

# Initialize the Claude chat model
llm = ChatAnthropic(model='claude-3-5-sonnet-20240620', anthropic_api_key=api_key)

# Define the prompt template
template = """You are an AI assistant that analyzes images based on the provided class and image.
Analyze the provided image and identify the object based on the given class name.
Based on the image analysis, provide the following details in one word each:
- Shape of the object
- Color of the object
- Orientation of the object in degrees. Angle Generated with the vertical like.
- Characteristics of the object
Provide the response in JSON format:
{{
  "shape": "xxx",
  "color": "xxx",
  "orientation": "xxx",
  "characteristics": "xxx"
}}
Image: {image}
Class: {class_name}
AI:"""

PROMPT = PromptTemplate(input_variables=["image", "class_name"], template=template)

# Initialize the LLMChain
llm_chain = LLMChain(prompt=PROMPT, llm=llm)

@app.route('/analyze', methods=['POST'])
def analyze_image():
    data = request.get_json()
    if 'image' not in data or 'class' not in data:
        return jsonify({'error': 'Missing required parameters'}), 400
    
    base64_image = data['image']
    class_name = data['class']

    # Pass the input variables to the LLMChain
    assistant_response = llm_chain.run(image=base64_image, class_name=class_name)

    return jsonify({'response': assistant_response})

if __name__ == '__main__':
    app.run(debug=True)