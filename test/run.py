#-----------------------------------------------------------------------
import subprocess
import  RPi.GPIO as GPIO
#from gpiozero import Button
import pyrebase
import time
import signal
#-----------------------------------------------------------------------
cmd1 = "sudo chown root.gpio /dev/gpiomem"
cmd2 = "sudo chmod g+rw /dev/gpiomem"
subprocess.run(cmd1, shell=True)
subprocess.run(cmd2, shell=True)
#-----------------------------------------------------------------------
firebaseConfig = {
    "apiKey": "AIzaSyC-4AaSkX_jrlJHIBp78OJ2GW4spxtt_iI",
    "authDomain": "smarthome-c5a2f.firebaseapp.com",
    "databaseURL": "https://smarthome-c5a2f-default-rtdb.asia-southeast1.firebasedatabase.app",
    "projectId": "smarthome-c5a2f",
    "storageBucket": "smarthome-c5a2f.appspot.com",
    "messagingSenderId": "1064276878746",
    "appId": "1:1064276878746:web:69c4500083e78397958623",
    "measurementId": "G-N7CWDXGKHN"
}
firebase = pyrebase.initialize_app(firebaseConfig)
db = firebase.database()
#-----------------------------------------------------------------------
GPIO.setmode(GPIO.BCM)
led_pin = 21
GPIO.setup(led_pin, GPIO.OUT)

#-----------------------------------------------------------------------
while True:
  subprocess.run("python3 detect.py", shell=True)
  

