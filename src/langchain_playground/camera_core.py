import cv2 
from llm_core import LLMCore
from ollama_core import OllamaCore

vid = cv2.VideoCapture(1) 

llm_count = 0

llm_core = LLMCore()
ollama_core = OllamaCore()

while(True): 
    llm_count += 1
    ret, frame = vid.read() 
    cv2.imshow('frame', frame) 

    processed_image_path = './cam.jpg'
    cv2.imwrite(processed_image_path, frame)

    result_txt = ollama_core.ask_img("", processed_image_path)
    print(result_txt)
    print("--------")
    print(llm_core.ask_txt(f"summary the following text into one sentences: {result_txt}"))

    if(llm_count >= 4):
        print("===== End")
        print(llm_core.ask_txt("give me a summary of all our conversion"))
        break
    if cv2.waitKey(1) & 0xFF == ord('q'): 
        print("===== End")
        print(llm_core.ask_txt("give me a summary of what you know from what I told you"))
        break

vid.release() 
cv2.destroyAllWindows() 