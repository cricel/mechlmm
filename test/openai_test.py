import argparse
import base64
import io
import json
import os
from PIL import Image
import requests
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import openai

def get_api_key():
    api_key = os.environ.get("OPENAI_API_KEY")
    if not api_key:
        api_key = input("Please enter your OpenAI API key: ")
        os.environ["OPENAI_API_KEY"] = api_key
    return api_key

def resize_and_compress_image(file_path, max_width=1000):
    with Image.open(file_path) as img:
        if img.width > max_width:
            ratio = max_width / img.width
            new_size = (max_width, int(img.height * ratio))
            img = img.resize(new_size, Image.LANCZOS)
        
        buffer = io.BytesIO()
        img.save(buffer, format="JPEG", quality=70)
        return buffer.getvalue()

def process_prompt(api_key, model_name, prompt):
    openai.api_key = api_key
    response = openai.ChatCompletion.create(
        model=model_name,
        messages=[
            {"role": "system", "content": "You are an assistant that helps with processing bounding boxes."},
            {"role": "user", "content": prompt}
        ]
    )
    return response['choices'][0]['message']['content']

def extract_coordinates(text):
    import re
    regex = r'\[\s*\d+\s*,\s*\d+\s*,\s*\d+\s*,\s*\d+\s*\]'
    matches = re.findall(regex, text)
    return [json.loads(match) for match in matches]

def display_image_with_bounding_boxes(image_path, coordinates):
    img = Image.open(image_path)
    fig, ax = plt.subplots(1)
    ax.imshow(img)

    colors = ['r', 'g', 'b', 'y', 'm', 'c']
    for i, box in enumerate(coordinates):
        ymin, xmin, ymax, xmax = [coord / 1000 for coord in box]
        width = xmax - xmin
        height = ymax - ymin
        rect = Rectangle((xmin * img.width, ymin * img.height), width * img.width, height * img.height,
                         linewidth=2, edgecolor=colors[i % len(colors)], facecolor='none')
        ax.add_patch(rect)

    plt.xticks(range(0, img.width + 1, 100))
    plt.yticks(range(0, img.height + 1, 100))
    plt.show()

def main():
    # api_key = get_api_key()
    
    print("Processing prompt...")
    response_text = process_prompt("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", "gpt-3.5-turbo", "Return bounding boxes around each object in the format [ymin, xmin, ymax, xmax]")
    print("API Response:")
    print(response_text)

    coordinates = extract_coordinates(response_text)
    if coordinates:
        print("Displaying image with bounding boxes...")
        display_image_with_bounding_boxes("./test.jpg", coordinates)
    else:
        print("No bounding box coordinates found in the response.")

if __name__ == "__main__":
    main()
