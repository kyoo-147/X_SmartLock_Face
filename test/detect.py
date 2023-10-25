import os
import pandas as pd
import time
import logging
import math
import  RPi.GPIO as GPIO
from gpiozero import Button
import pyrebase
import dlib
import numpy as np
import cv2
import sys
import subprocesss

GPIO.setmode(GPIO.BCM)
led_pin21 = 21
led_pin26 = 26

GPIO.setup(led_pin21, GPIO.OUT)
GPIO.setup(led_pin26, GPIO.OUT)
button_1 = Button(5)
button_2 = Button(6)

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
class Face_Recognizer:
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
        if self.get_face_database():
            while stream.isOpened():
                    
                def door_open():
                    db.child("categories").child("0").update({"switch": True})        
                def door_close():
                    GPIO.output(led_pin21, GPIO.HIGH)
                    GPIO.output(led_pin26, GPIO.HIGH)
                    db.child("categories").child("0").update({"switch": False})
                

                self.frame_cnt += 1

                flag, img_rd = stream.read()
                # kk = cv2.waitKey(1) 
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
                        GPIO.output(led_pin21, GPIO.HIGH)
                        GPIO.output(led_pin26, GPIO.HIGH)  
                        db.child("categories").child("0").update({"switch": False})


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
                            if min(self.current_frame_face_X_e_distance_list) < 0.25:
                                self.current_frame_face_name_list[k] = self.face_name_known_list[similar_person_num]
                                print("Face recognition result:",
                                                self.face_name_known_list[similar_person_num])
                                GPIO.output(led_pin21, GPIO.LOW)
                                GPIO.output(led_pin26, GPIO.LOW)
                                door_open()
                                #time.sleep(6)
                            else:
                                print("Face recognition result: Unknown person")
                if button_2.is_pressed:
                    subprocess.run("python3 finger.py", shell=True)
                if button_1.is_pressed:

                    print("OPEN")
                    GPIO.output(led_pin21, GPIO.LOW)
                    GPIO.output(led_pin26, GPIO.LOW)
                    door_open()
                    time.sleep(5)
                    
                    GPIO.output(led_pin21, GPIO.HIGH)
                    GPIO.output(led_pin26, GPIO.HIGH)
                    door_close()
                    time.sleep(5)
                self.update_fps()
                cv2.namedWindow("camera", 1)
                cv2.imshow("camera", img_rd)

                #logging.debug("Frame ends\n\n")

    def run(self):
        cap = cv2.VideoCapture(0)              # Get video stream from camera
        #cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  
        self.process(cap)
        cap.release()
        cv2.destroyAllWindows()
    
def main():
    Face_Recognizer().run()

if __name__ == '__main__':
    main()



