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

    def chat_img(self, _base_img):
        summary_prompt_text = """What's in this image?"""
        feature_prompt_text = """
        What objects are in the image, and give me the bounding box coordinate position of each object, and unique features of each object, and return it in json format.
        and use this format {objects: {object_name: {position: [top_left_x, top_left_y, bottom_right_x, bottom_right_y]}, features: [features_1, feature_2]}
        only return the json itself, no any other additional content
        """

        target_prompt = feature_prompt_text
        summary_result = self.image_summarize(_base_img, target_prompt)

        cleaned_summary_result = summary_result.replace("```json", "")
        cleaned_summary_result = cleaned_summary_result.replace("```", "")

        print(cleaned_summary_result)

        return cleaned_summary_result

    def encode_image(self, image_path):
        """Getting the base64 string"""
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode("utf-8")


    def image_summarize(self, img_base64, prompt):
        """Make image summary"""
        chat = ChatOllama(base_url="http://192.168.1.182:11434", model=self.model)

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