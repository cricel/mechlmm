# Python program to translate
# speech to text and text to speech
 
 
import speech_recognition as sr
import pyttsx3 
 
while(1):    
    try:
        print ("say something") 
        r = sr.Recognizer() 
        with sr.Microphone() as source2:
            r.adjust_for_ambient_noise(source2, duration=0.2)
            audio2 = r.listen(source2)
            MyText = r.recognize_google(audio2)
            MyText = MyText.lower()
 
            print("Did you say ",MyText)
             
    except sr.RequestError as e:
        # print("Could not request results; {0}".format(e))
        pass
         
    except sr.UnknownValueError:
        # print("unknown error occurred")
        pass