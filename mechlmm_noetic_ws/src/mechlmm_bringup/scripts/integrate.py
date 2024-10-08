import requests
import speech_recognition as sr
import pyttsx3 
from mechlmm_py import GTTS_Core

tts_core = GTTS_Core()
url = 'http://192.168.1.134:5001/mechlmm/chat'

while(1):    
    try:
        print ("say something") 
        r = sr.Recognizer() 
        with sr.Microphone() as source2:
            r.adjust_for_ambient_noise(source2, duration=0.2)
            audio2 = r.listen(source2)
            _stt_result = r.recognize_google(audio2)
            _stt_result = _stt_result.lower()
 
            print("Did you say ", _stt_result)

            data = {
                'question': _stt_result,
                # 'schema': dict_schema,
                # 'tag': 'value2',
                # 'base_img': [image_url, image_url_1],
                # 'tools': [tool_schema_1, tool_schema_2],
                'model': "claude"
            }

            response = requests.post(url, json=data)

            if response.status_code == 200:
                _result = response.json()
                print('Success: \n', _result)
                
                tts_core.tts_play(_result)

            else:
                print('Failed:', response.status_code, response.text)

            
    except sr.RequestError as e:
        print("Could not request results; {0}".format(e))
        # pass
         
    except sr.UnknownValueError:
        print("unknown error occurred")
        # pass