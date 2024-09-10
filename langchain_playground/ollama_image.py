import base64

from langchain_core.messages import HumanMessage
from langchain_community.chat_models import ChatOllama

class OllamaCore:
    def __init__(self):
        self.model = "llava"

    def chat(self):
        pass

    def chat_text(self):
        pass

    def chat_img(self, base_img):
        summary_prompt_text = """What's in this image?"""

        summary_result = self.image_summarize(base_img, summary_prompt_text)

        print(summary_result)

    def encode_image(self, image_path):
        """Getting the base64 string"""
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode("utf-8")


    def image_summarize(self, img_base64, prompt):
        """Make image summary"""
        chat = ChatOllama(model=self.model)

        image_url = f"data:image/jpeg;base64,{img_base64}"

        msg = chat.invoke(
            [
                HumanMessage(
                    content=[
                        {"type": "text", "text": prompt},
                        {"type": "image_url", "image_url": image_url},
                    ]
                )
            ]
        )

        return msg.content
    
if __name__ == '__main__':
    ollama_core = OllamaCore()

    img_path = "./art_1.jpg"
    base64_image = ollama_core.encode_image(img_path)
    
    ollama_core.chat_img(base64_image)