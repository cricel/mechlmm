import speech_recognition as sr

# Initialize recognizer
recognizer = sr.Recognizer()

# Capture audio from the microphone
with sr.Microphone() as source:
    print("Say something:")
    audio = recognizer.listen(source)

# Convert audio to text using Google's Speech Recognition
try:
    print("You said: " + recognizer.recognize_google(audio))
except sr.UnknownValueError:
    print("Sorry, I could not understand the audio.")
except sr.RequestError as e:
    print("Could not request results from Google Speech Recognition service; {0}".format(e))
