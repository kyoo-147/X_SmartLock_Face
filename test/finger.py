#def finger():
import serial
import adafruit_fingerprint
import  RPi.GPIO as GPIO
import pyrebase
import time
import sys

GPIO.setmode(GPIO.BCM)
led_pin21 = 21
led_pin26 = 26
GPIO.setup(led_pin21, GPIO.OUT)
GPIO.setup(led_pin26, GPIO.OUT)

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
uart = serial.Serial("/dev/ttyS0", baudrate=57600, timeout=1)
finger = adafruit_fingerprint.Adafruit_Fingerprint(uart)

print("----------------")
if finger.read_templates() != adafruit_fingerprint.OK:
    raise RuntimeError("Failed to read templates")
print("Fingerprint templates: ", finger.templates)
if finger.count_templates() != adafruit_fingerprint.OK:
    raise RuntimeError("Failed to read templates")
print("Number of templates found: ", finger.template_count)
if finger.read_sysparam() != adafruit_fingerprint.OK:
    raise RuntimeError("Failed to get system parameters")
def get_fingerprint():
    """Get a finger print image, template it, and see if it matches!"""
    print("Waiting for image...")
    
    while finger.get_image() != adafruit_fingerprint.OK:
        pass
    print("Templating...")
    if finger.image_2_tz(1) != adafruit_fingerprint.OK:
        return False
    print("Searching...")
    if finger.finger_search() != adafruit_fingerprint.OK:
        return False
    return True
if get_fingerprint():
    print("Detected #", finger.finger_id, "with confidence", finger.confidence)
    db.child("categories").child("0").update({"switch": True})
    GPIO.output(led_pin21, GPIO.LOW)
    GPIO.output(led_pin26, GPIO.LOW)
    time.sleep(6)
    GPIO.output(led_pin21, GPIO.HIGH)
    GPIO.output(led_pin26, GPIO.LOW)
    db.child("categories").child("0").update({"switch": False})
    time.sleep(6)
    sys.exit()
else:
    print("Finger not found")
    sys.exit()
