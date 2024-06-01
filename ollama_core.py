from ollama import Client

class OllamaCore:
    def __init__(self):
        self.client = Client(host='http://131.123.41.132:11434')
        
    def ask_img(self, _msg, _img_path):
        return self.client.chat(
            model="llava:34b",
            messages=[
                {
                    'role': 'user',
                    'content': 'give me a description of what happen in this image:',
                    'images': [_img_path]
                }
            ]
        )['message']['content']

if __name__ == "__main__":
    ollama_core = OllamaCore()
    print(ollama_core.ask_img("", "./messy_room.jpg"))