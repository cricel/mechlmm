# Assuming _base_img_list is your list of image URLs
_base_img_list = ["https://example.com/image1.jpg", "https://example.com/image2.jpg"]  # Example URLs
_question = "Your question here"

# Initialize an empty list to hold the content for HumanMessage
content_list = [
    {"type": "text", "text": _question}
]

# Loop through _base_img_list to create the image_url content dynamically
for img_url in _base_img_list:
    content_list.append({
        "type": "image_url",
        "image_url": img_url
    })

# Final HumanMessage with dynamically created content
human_message = {
    "content": content_list
}

print(human_message)

{'content': [
    {'type': 'text', 'text': 'Your question here'}, 
    {'type': 'image_url', 'image_url': 'https://example.com/image1.jpg'}, 
    {'type': 'image_url', 'image_url': 'https://example.com/image2.jpg'}
    ]
}