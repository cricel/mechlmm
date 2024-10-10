#!/usr/bin/env python

import speech_recognition as sr
from mechlmm_py import TTS_Core, DebugCore, utilities_core
import sys

class MechLMMChatBot:
    def __init__(self):
        self.tts_core = TTS_Core()
        self.debug_core = DebugCore()
        self.recognizer = sr.Recognizer()

    def get_speech_input(self):
        with sr.Microphone() as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=0.2)
            audio = self.recognizer.listen(source)
            try:
                speech_text = self.recognizer.recognize_google(audio)
                return speech_text.lower()
            except sr.RequestError as e:
                self.debug_core.log_error(f"Could not request results; {e}")
                return None
            except sr.UnknownValueError:
                self.debug_core.log_error("Unknown error occurred")
                return None

    def send_request(self, question):
        chat_type = "normal"

        if len(sys.argv) > 1:
            chat_type = sys.argv[1]
        
        data = {
            'question': question,
        }
        try:
            response = utilities_core.rest_post_request(data, 'http://192.168.1.134:5001/mechlmm/chat')

            if(chat_type == "qa"):
                response = utilities_core.rest_post_request(data, 'http://192.168.1.134:5001/mechlmm/chat/qa')
            if(chat_type == "data"):
                response = utilities_core.rest_post_request(data, 'http://192.168.1.134:5001/mechlmm/chat/data')

            return response
        except Exception as e:
            self.debug_core.log_error(f"Error sending request: {e}")
            return None

    def play_response(self, response_text):
        self.tts_core.tts_play(response_text)

    def run(self):
        while True:
            self.debug_core.log_info("How can I help you?")
            user_input = self.get_speech_input()
            if user_input:
                self.debug_core.log_info(f"You Said: {user_input}")
                result = self.send_request(user_input)
                if result:
                    self.debug_core.log_info(f"MechLMM: {result}")
                    self.play_response(result["result"])

if __name__ == "__main__":
    bot = MechLMMChatBot()
    bot.run()
