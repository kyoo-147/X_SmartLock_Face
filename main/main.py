import os
from turtle import st
import pandas as pd
import time
import logging
import math
import  RPi.GPIO as GPIO
import pyrebase
import dlib
import numpy as np
import cv2
import sys
import subprocess
import adafruit_fingerprint
import serial
import smbus
import threading

cmd1 = "sudo chown root.gpio /dev/gpiomem"
cmd2 = "sudo chmod g+rw /dev/gpiomem"

subprocess.run(cmd1, shell=True)
subprocess.run(cmd2, shell=True)

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

detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor('data/data_dlib/shape_predictor_68_face_landmarks.dat')
face_reco_model = dlib.face_recognition_model_v1("data/data_dlib/dlib_face_recognition_resnet_model_v1.dat")

GPIO.setmode(GPIO.BCM)

led_pin1 = 13
led_pin2 = 26

button_1 = 5
# button_2 = 6

GPIO.setup(led_pin1, GPIO.OUT)
GPIO.setup(led_pin2, GPIO.OUT)
GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# GPIO.setup(6, GPIO.IN, pull_up_down=GPIO.PUD_UP)
def door_open():
    GPIO.output(led_pin1, GPIO.LOW)
    GPIO.output(led_pin2, GPIO.LOW)
    db.child("categories").child("0").update({"switch": True})        
def door_close():
    GPIO.output(led_pin1, GPIO.HIGH)
    GPIO.output(led_pin2, GPIO.HIGH)
    db.child("categories").child("0").update({"switch": False})

I2C_ADDR  = 0x27 # I2C device address
LCD_WIDTH = 16   # Maximum characters per line

# Define some device constants
LCD_CHR = 1 # Mode - Sending data
LCD_CMD = 0 # Mode - Sending command

LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line
LCD_LINE_3 = 0x94 # LCD RAM address for the 3rd line
LCD_LINE_4 = 0xD4 # LCD RAM address for the 4th line

LCD_BACKLIGHT  = 0x08  # On
ENABLE = 0b00000100 # Enable bit

# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005

#Open I2C interface
bus = smbus.SMBus(1)

def lcd_init():
    # Initialise display
    lcd_byte(0x33,LCD_CMD) # 110011 Initialise
    lcd_byte(0x32,LCD_CMD) # 110010 Initialise
    lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
    lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off 
    lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
    lcd_byte(0x01,LCD_CMD) # 000001 Clear display
    time.sleep(E_DELAY)

def lcd_byte(bits, mode):
    # Send byte to data pins
    # bits = the data
    # mode = 1 for data
    #        0 for command

    bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
    bits_low = mode | ((bits<<4) & 0xF0) | LCD_BACKLIGHT

    # High bits
    bus.write_byte(I2C_ADDR, bits_high)
    lcd_toggle_enable(bits_high)

    # Low bits
    bus.write_byte(I2C_ADDR, bits_low)
    lcd_toggle_enable(bits_low)

def lcd_toggle_enable(bits):
    # Toggle enable
    time.sleep(E_DELAY)
    bus.write_byte(I2C_ADDR, (bits | ENABLE))
    time.sleep(E_PULSE)
    bus.write_byte(I2C_ADDR,(bits & ~ENABLE))
    time.sleep(E_DELAY)

def lcd_string(message,line):
    # Send string to display

    message = message.ljust(LCD_WIDTH," ")

    lcd_byte(line, LCD_CMD)

    for i in range(LCD_WIDTH):
        lcd_byte(ord(message[i]),LCD_CHR)

class Face_Recognizer():
    def __init__(self):
        self.font = cv2.FONT_ITALIC
        self.frame_time = 0
        self.frame_start_time = 0
        self.fps = 0
        self.fps_show = 0
        self.start_time = time.time()
        self.frame_cnt = 0
        self.face_features_known_list = []
        self.face_name_known_list = []
        self.last_frame_face_centroid_list = []
        self.current_frame_face_centroid_list = []
        self.last_frame_face_name_list = []
        self.current_frame_face_name_list = []
        self.last_frame_face_cnt = 0
        self.current_frame_face_cnt = 0
        self.current_frame_face_X_e_distance_list = []
        self.current_frame_face_position_list = []
        self.current_frame_face_feature_list = []
        self.last_current_frame_centroid_e_distance = 0
        self.reclassify_interval_cnt = 0
        self.reclassify_interval = 10

        #self.e_distance_accuracy = 0

    def get_face_database(self):
        if os.path.exists("data/features_all.csv"):
            path_features_known_csv = "data/features_all.csv"
            csv_rd = pd.read_csv(path_features_known_csv, header=None)
            for i in range(csv_rd.shape[0]):
                features_someone_arr = []
                self.face_name_known_list.append(csv_rd.iloc[i][0])
                for j in range(1, 129):
                    if csv_rd.iloc[i][j] == '':
                        features_someone_arr.append('0')
                    else:
                        features_someone_arr.append(csv_rd.iloc[i][j])
                self.face_features_known_list.append(features_someone_arr)
            return 1
        else:
            return 0
    def update_fps(self):
        now = time.time()
        if str(self.start_time).split(".")[0] != str(now).split(".")[0]:
            self.fps_show = self.fps
        self.start_time = now
        self.frame_time = now - self.frame_start_time
        self.fps = 1.0 / self.frame_time
        self.frame_start_time = now
    @staticmethod
    def return_euclidean_distance(feature_1, feature_2):
        feature_1 = np.array(feature_1)
        feature_2 = np.array(feature_2)
        dist = np.sqrt(np.sum(np.square(feature_1-feature_2)))
        return dist
    def centroid_tracker(self):
        for i in range(len(self.current_frame_face_centroid_list)):
            e_distance_current_frame_person_x_list = []
            for j in range(len(self.last_frame_face_centroid_list)):
                self.last_current_frame_centroid_e_distance = self.return_euclidean_distance(
                    self.current_frame_face_centroid_list[i], self.last_frame_face_centroid_list[j])
                e_distance_current_frame_person_x_list.append(
                    self.last_current_frame_centroid_e_distance)
            last_frame_num = e_distance_current_frame_person_x_list.index(
                min(e_distance_current_frame_person_x_list))
            self.current_frame_face_name_list[i] = self.last_frame_face_name_list[last_frame_num]
    def draw_note(self, img_rd):
        cv2.putText(img_rd, "HE THONG KHOA CUA THONG MINH - AI-LAB", (20, 40), self.font, 1, (255, 255, 255), 1, cv2.LINE_AA)

        cv2.putText(img_rd, "HUNG HINH MOI GIAY:    " + str(self.fps.__round__(2)), (20, 130), self.font, 0.8, (0, 255, 0), 1,
                    cv2.LINE_AA)
        cv2.putText(img_rd, "SO KHUON MAT:  " + str(self.current_frame_face_cnt), (20, 160), self.font, 0.8, (0, 255, 0), 1,
                    cv2.LINE_AA)
        cv2.putText(img_rd, "Q: THOAT", (20, 450), self.font, 0.8, (255, 255, 255), 1, cv2.LINE_AA)

    def process(self, stream):
        
        print("open camera")
        global door

        if self.get_face_database():
            
            debounce_time = 0.2
            button_state_1_last = GPIO.input(button_1)
            # button_state_2_last = GPIO.input(button_2)
            last_button_press_time_1 = 0
            last_button_press_time_2 = 0
                        # Define some device parameters
                
            lcd_init()
            

            def lcd_mo_cua():
                lcd_string("       Mo       ",LCD_LINE_1)
                lcd_string("                ",LCD_LINE_2)
            def lcd_dong_cua():
                lcd_string("      Dong      ",LCD_LINE_1)
                lcd_string("                ",LCD_LINE_2)

            def lcd_thanh_cong():
                lcd_string("   Thanh cong   ",LCD_LINE_1)
                lcd_string("                ",LCD_LINE_2)
            def lcd_that_bai():
                lcd_string(" Khong tim thay ",LCD_LINE_1)
                lcd_string("                ",LCD_LINE_2)

            def door_open():
                GPIO.output(led_pin1, GPIO.LOW)
                GPIO.output(led_pin2, GPIO.LOW)
                db.child("categories").child("0").update({"switch": True})        
            def door_close():
                GPIO.output(led_pin1, GPIO.HIGH)
                GPIO.output(led_pin2, GPIO.HIGH)
                db.child("categories").child("0").update({"switch": False})
 
            uart = serial.Serial("/dev/ttyS0", baudrate=57600, timeout=1)
            finger = adafruit_fingerprint.Adafruit_Fingerprint(uart)
            
            global door
            
            def status_door_open():
                door = True
            
            def status_door_close():
                door = False

            def finger_print():
                while True:
                    def get_fingerprint():
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
                        if status_door_close:
                            print("Detected #", finger.finger_id, "with confidence", finger.confidence)
                            door_open()
                            status_door_open
                            time.sleep(2)
                            status_door_close
                            door_close()

                    else:
                        print("Finger not found")
                        time.sleep(1)

                    if KeyboardInterrupt:
                        #GPIO.cleanup()
                        lcd_byte(0x01, LCD_CMD)
            def lcd():
                lcd_string(" He Thong Khoa  ",LCD_LINE_1)
                lcd_string(" Cua Thong Minh ",LCD_LINE_2)
            fingerprint_thread = threading.Thread(target=finger_print)
            fingerprint_thread.start()

            while stream.isOpened():
                lcd()
                self.frame_cnt += 1
                flag, img_rd = stream.read()
                kk = cv2.waitKey(1) 
                faces = detector(img_rd, 0)
                self.last_frame_face_cnt = self.current_frame_face_cnt
                self.current_frame_face_cnt = len(faces)
                self.last_frame_face_name_list = self.current_frame_face_name_list[:]
                self.last_frame_face_centroid_list = self.current_frame_face_centroid_list
                self.current_frame_face_centroid_list = []
                if (self.current_frame_face_cnt == self.last_frame_face_cnt) and (
                        self.reclassify_interval_cnt != self.reclassify_interval):
                    #logging.debug("scene 1: No face cnt changes in this frame!")
                    self.current_frame_face_position_list = []
                    if "unknown" in self.current_frame_face_name_list:
                        #logging.debug("  There are unknown faces, let's start counting reclassify_interval_cnt")
                        self.reclassify_interval_cnt += 1
                    if self.current_frame_face_cnt != 0:
                        for k, d in enumerate(faces):
                            self.current_frame_face_position_list.append(tuple(
                                [faces[k].left(), int(faces[k].bottom() + (faces[k].bottom() - faces[k].top()) / 4)]))
                            self.current_frame_face_centroid_list.append(
                                [int(faces[k].left() + faces[k].right()) / 2,
                                    int(faces[k].top() + faces[k].bottom()) / 2])
                            img_rd = cv2.rectangle(img_rd,
                                                    tuple([d.left(), d.top()]),
                                                    tuple([d.right(), d.bottom()]),
                                                    (255, 255, 255), 2)
                    if self.current_frame_face_cnt != 1:
                        self.centroid_tracker()
                    if self.current_frame_face_cnt == 0:
                        status_door_close
                        door_close()
                        #time.sleep(6)
                        
                    for i in range(self.current_frame_face_cnt):
                        img_rd = cv2.putText(img_rd, self.current_frame_face_name_list[i],
                                                self.current_frame_face_position_list[i], self.font, 0.8, (0, 255, 255), 1,
                                                cv2.LINE_AA)
                    self.draw_note(img_rd)
                else:
                    # logging.debug("scene 2: Faces cnt changes in this frame")
                    self.current_frame_face_position_list = []
                    self.current_frame_face_X_e_distance_list = []
                    self.current_frame_face_feature_list = []
                    self.reclassify_interval_cnt = 0
                    if self.current_frame_face_cnt == 0:
                        # logging.debug("  scene 2.1 No faces in this frame!!!")
                        self.current_frame_face_name_list = []
                    else:
                        # logging.debug("  scene 2.2 Get faces in this frame and do face recognition")
                        self.current_frame_face_name_list = []
                        for i in range(len(faces)):
                            shape = predictor(img_rd, faces[i])
                            self.current_frame_face_feature_list.append(
                                face_reco_model.compute_face_descriptor(img_rd, shape))
                            self.current_frame_face_name_list.append("unknown")
                        for k in range(len(faces)):
                            # logging.debug("  For face %d in current frame:", k + 1)
                            self.current_frame_face_centroid_list.append(
                                [int(faces[k].left() + faces[k].right()) / 2,
                                    int(faces[k].top() + faces[k].bottom()) / 2])
                            self.current_frame_face_X_e_distance_list = []
                            self.current_frame_face_position_list.append(tuple(
                                [faces[k].left(), int(faces[k].bottom() + (faces[k].bottom() - faces[k].top()) / 4)]))
                            for i in range(len(self.face_features_known_list)):
                                if str(self.face_features_known_list[i][0]) != '0.0':
                                    e_distance_tmp = self.return_euclidean_distance(
                                        self.current_frame_face_feature_list[k],
                                        self.face_features_known_list[i])
                                    linear_val =(1.0 - e_distance_tmp)/ (0.4*2.0)

                                    self.current_frame_face_X_e_distance_list.append(e_distance_tmp)

                                    self.draw_note(img_rd)
                                    
                                else:
                                    self.current_frame_face_X_e_distance_list.append(999999999)
                            similar_person_num = self.current_frame_face_X_e_distance_list.index(
                                min(self.current_frame_face_X_e_distance_list))
                            if min(self.current_frame_face_X_e_distance_list) < 0.4   :
                                self.current_frame_face_name_list[k] = self.face_name_known_list[similar_person_num]
                                print("Face recognition result:",
                                                self.face_name_known_list[similar_person_num])
                                if status_door_close:
                                    status_door_open()
                                    door_open()
                                    # time.sleep(0.5)
                                    status_door_close()
                            else:
                                print("Face recognition result: Unknown person")
                button_state_1 = GPIO.input(button_1)
                # button_state_2 = GPIO.input(button_2)
                if button_state_1 != button_state_1_last:
                    current_time = time.time()
                    if current_time - last_button_press_time_1 >= debounce_time:
                        if button_state_1 == GPIO.LOW:
                            print("Nút nhấn 1 được nhấn")
                            # lcd_van_tay()
                            # def get_fingerprint():
                            #     print("Waiting for image...")
                            #     while finger.get_image() != adafruit_fingerprint.OK:
                            #         pass
                            #     print("Templating...")
                            #     if finger.image_2_tz(1) != adafruit_fingerprint.OK:
                            #         return False
                            #     print("Searching...")
                            #     if finger.finger_search() != adafruit_fingerprint.OK:
                            #         return False
                            #     return True
                            # if get_fingerprint():
                            #     print("Detected #", finger.finger_id, "with confidence", finger.confidence)
                            #     lcd_thanh_cong()
                            #     door_open()
                            #     time.sleep(6)
                            #     door_close()
                            #     time.sleep(6)  
                            # else:
                            #     print("Finger not found")
                            #     lcd_that_bai()
                            #     time.sleep(1)

                            print("OPEN")
                            lcd_mo_cua()
                            door_open()
                            status_door_open()
                            time.sleep(5)
                            
                            print("CLOSE")
                            status_door_close()
                            door_close()

                        button_state_1_last = button_state_1
                        last_button_press_time_1 = current_time
                                            
                
                # if button_state_2 != button_state_2_last:
                #     current_time = time.time()
                #     if current_time - last_button_press_time_2 >= debounce_time:
                #         if button_state_2 == GPIO.LOW:

                #             print("Nút nhấn 2 được nhấn")

                #             print("OPEN")
                #             lcd_mo_cua()
                #             door_open()
                #             time.sleep(5)
                            
                #             print("CLOSE")
                #             lcd_dong_cua()
                #             door_close()
                #             time.sleep(5)
                            
                        # button_state_2_last = button_state_2
                        # last_button_press_time_2 = current_time

                if KeyboardInterrupt:
                    #GPIO.cleanup()
                    lcd_byte(0x01, LCD_CMD)
                    #print("Tạm dừng chương trình")
                
                if kk == ord('q'):
                    #GPIO.cleanup()
                    lcd_byte(0x01, LCD_CMD)
                    break
                    
                self.update_fps()

                cv2.namedWindow("camera", 1)
                cv2.imshow("camera", img_rd)
    

    def run(self):
        cap = cv2.VideoCapture(-1)
        #cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.process(cap)
        cap.release()
        cv2.destroyAllWindows()

def main():
    Face_Recognizer_con = Face_Recognizer()
    Face_Recognizer_con.run()

if __name__ == '__main__':
    main()



