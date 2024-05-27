import cv2 
from ollama import Client


client = Client(host='http://131.123.41.132:11434')

vid = cv2.VideoCapture(1) 

llm_count = 0

while(True): 
    ret, frame = vid.read() 
    cv2.imshow('frame', frame) 

    processed_image_path = './cam.jpg'
    cv2.imwrite(processed_image_path, frame)

    res = client.chat(
        model="llava:34b",
        messages=[
            {
                'role': 'user',
                'content': 'give me a one sentence description of what happen in this image:',
                'images': ['./cam.jpg']
            }
        ]
    )

    print("run llm: (" + str(llm_count) + ") times")
    print(res['message']['content'])

    if cv2.waitKey(1) & 0xFF == ord('q'): 
        break

vid.release() 
cv2.destroyAllWindows() 