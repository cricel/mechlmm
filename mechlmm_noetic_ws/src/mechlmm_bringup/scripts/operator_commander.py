#!/usr/bin/env python

import requests
import speech_recognition as sr
import pyttsx3 
from mechlmm_py import TTS_Core, DebugCore, utilities_core

tts_core = TTS_Core()
debug_core = DebugCore()

while(1):    
    try:
        debug_core.log_info("How can I help you?")
        r = sr.Recognizer() 
        with sr.Microphone() as source2:
            r.adjust_for_ambient_noise(source2, duration=0.2)
            audio2 = r.listen(source2)
            _stt_result = r.recognize_google(audio2)
            _stt_result = _stt_result.lower()
            debug_core.log_info(f"you said: {_stt_result}")
            
            data = {
                'question': _stt_result,
                # 'schema': dict_schema,
                # 'tag': 'value2',
                # 'base_img': [image_url, image_url_1],
                # 'tools': [tool_schema_1, tool_schema_2],
                # 'model': "claude"
            }

            result = utilities_core.rest_post_request(data, 'http://192.168.1.134:5001/mechlmm/chat/qa')

            tts_core.tts_play(result["result"])

            
    except sr.RequestError as e:
        print("Could not request results; {0}".format(e))
        # pass
         
    except sr.UnknownValueError:
        print("unknown error occurred")
        # pass