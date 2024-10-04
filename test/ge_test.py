import argparse
import base64
import io
import json
import os
from PIL import Image
import requests
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

def get_api_key():
    api_key = os.environ.get("GEMINI_API_KEY")
    if not api_key:
        api_key = input("Please enter your Gemini API key: ")
        os.environ["GEMINI_API_KEY"] = api_key
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

def file_to_generative_part(file_path):
    with open(file_path, "rb") as image_file:
        return {
            "inlineData": {
                "data": base64.b64encode(image_file.read()).decode("utf-8"),
                "mimeType": "image/jpeg"
            }
        }

def process_image_and_prompt(api_key, model_name, image_path, prompt):
    url = f"https://generativelanguage.googleapis.com/v1beta/models/{model_name}:generateContent"
    headers = {
        "Content-Type": "application/json",
        "x-goog-api-key": api_key
    }

    image_part = file_to_generative_part(image_path)
    data = {
        "contents": [
            {
                "parts": [
                    {"text": prompt},
                    image_part
                ]
            }
        ]
    }

    response = requests.post(url, headers=headers, json=data)
    response.raise_for_status()
    print("---")
    print(response.json()["candidates"][0]["content"]["parts"][0]["text"])
    print("---")
    return response.json()["candidates"][0]["content"]["parts"][0]["text"]

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

    # plt.grid(True, color='r', linestyle='--', linewidth=0.5, alpha=0.5)
    plt.xticks(range(0, img.width + 1, 100))
    plt.yticks(range(0, img.height + 1, 100))
    plt.show()

def main():
    # parser = argparse.ArgumentParser(description="Process image and prompt using Gemini API")
    # parser.add_argument("image_path", help="Path to the input image")
    # parser.add_argument("prompt", help="Prompt for the Gemini API")
    # parser.add_argument("--model", default="gemini-pro-vision", help="Model name (default: gemini-pro-vision)")
    # args = parser.parse_args()

    api_key = "AIzaSyCSjCUGUydxiSsWmlMLEU_nfqtu70HPGZo"
    
    print("Processing image and prompt...")
    response_text = process_image_and_prompt(api_key, "gemini-1.5-pro-latest", "./test.jpg", "Return bounding boxes around each object, for each one return [ymin, xmin, ymax, xmax]")
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