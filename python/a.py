import pyttsx3
engine = pyttsx3.init()
engine.setProperty('rate', 160)
engine.say("This is a test of the voice system.")
engine.runAndWait()