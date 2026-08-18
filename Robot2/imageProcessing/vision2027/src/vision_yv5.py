#!/home/robotis/yolov5-env/bin/python3.8
# -*- coding: utf-8 -*-
from __future__ import annotations
import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'

if ros_path in sys.path:

    sys.path.remove(ros_path)
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

import os
import time
import subprocess
import math
import threading
import configparser
from dataclasses import dataclass
from typing import List, Tuple, Optional 
from threading import Thread

import numpy as np
import cv2

# ---------------- ROS ----------------
import rospy
from std_msgs.msg import String, Int16
from v2_detection.msg import _BallCoordinate, _BallState, _Ballarea, BallState, Ballarea, BallCoordinate,OpponentState, OpponentCoordinate,GoalState, GoalCoordinate


# -------------- PyTorch --------------
_YOLO_AVAILABLE = True
try:
    import torch  # type: ignore
except Exception as e:  # pragma: no cover
    _YOLO_AVAILABLE = False
    print("[WARN] PyTorch tidak tersedia:", e)


# ------------------------------------------------------------
# Inisialisasi YOLOv5 Nano
# ------------------------------------------------------------
# Pastikan sudah install ultralytics (pip install ultralytics==8.0.20)
# dan weight hasil training custom tersimpan misalnya di "best.pt"

WEIGHTS_PATH = os.path.expanduser('/home/robotis/catkin_ws/src/DEWO-OP3/KRI2025/ImageProcessing/v2_detection/src/runs/train/bola_yolov5n19/weights/best.pt')

device = 'cuda' if torch.cuda.is_available() else 'cpu'
model = torch.hub.load('ultralytics/yolov5', 'custom', path=WEIGHTS_PATH)
model.to(device)
model.conf = 0.5   # confidence threshold
model.iou = 0.45   # IOU threshold
model.classes = [0]  # hanya kelas bola (jika dataset 1 kelas)

# ------------------------------------------------------------
# Variabel Global
# ------------------------------------------------------------
min_h = 0
min_s = 0
min_v = 0
max_h = 0
max_s = 0
max_v = 0

x_ball = 0
y_ball = 0
w_ball = 0
h_ball = 0

pt1 = (0,0)
pt2 = (0,0)
fps = 0

# ---------------------------------------------------------------------------------
# jika meenggunakan fisheye maka gunakan 0.34
fisheye = 1
# jika tidak menggunakan fisheye maka gunakan 1
# ---------------------------------------------------------------------------------

detect_status = 'NOTFOUND'
waktu_sebelum = time.time()
ball_area = 0
center_ball = (0,0)
x_center_ball = 0
y_center_ball = 0
scan_area = [0,0]
framesize = [320,240]
status = 'TRACKBALL' # TRACKBALL , LAWAN_MAGENTA, KEEPER, TRACKGOAL
blobsize = 416
keeper_action = False

batas_keeper_kiri = np.array([[[0, 70], [145, 70], [70, 240]]], np.int32)
batas_keeper_kanan = np.array([[[250, 240], [175, 70], [320, 70]]], np.int32)
batas_keeper_tengah = np.array([[[70, 240], [160, 40], [160, 40], [250, 240]]], np.int32)

# ------------------------------------------------------------
# ROS Node & Publisher
# ------------------------------------------------------------
pub_state = rospy.Publisher("/vision/ball_state", BallState, queue_size=1)
pub_area = rospy.Publisher("/vision/ball_area", Ballarea, queue_size=1)
pub_coord = rospy.Publisher("/vision/ball_coordinate", BallCoordinate, queue_size=1)

# rospy.init_node("vision_yv5", anonymous=False)

# ------------------------------------------------------------
# Kamera Capture (default /dev/video0)
# ------------------------------------------------------------
class WebcamVideoStream:
    def __init__(self, src=0):
        self.stream = cv2.VideoCapture(src, cv2.CAP_V4L2)
        self.stream.set(cv2.CAP_PROP_FPS, 60)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, framesize[0])
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, framesize[1])
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False
    def start(self):
        Thread(target=self.update, args=()).start()
        return self
    def update(self):
        while True:
            if self.stopped:
                return
            (self.grabbed, self.frame) = self.stream.read()
    def read(self):
        return self.frame
    def stop(self):
        self.stopped = True

class ball_detection:

    def __init__(self):
        self.framecounter = 0
        self.startfpstime = time.time()
        self.displayfpstime = 0.2

    def calculate_fps(self):
        global fps
        current_time = time.time()
        elapsed_time = current_time - self.startfpstime
        if elapsed_time >= self.displayfpstime:
            fps = self.framecounter / elapsed_time
            self.framecounter = 0
            self.startfpstime = current_time 

    def send_ball_position(self, x_pos, y_pos, w_frame, h_frame, size_obj):
        ballposition = BallCoordinate()
        fix_x = (float) (x_pos / w_frame * 2 - 1)
        fix_y = (float) (y_pos / h_frame * 2 - 1)
        ballposition.pos_x = fix_x
        ballposition.pos_y = fix_y
        ballposition.obj_size = size_obj
        ballcoor.publish(ballposition)

    def send_ball_area(self, areaball):
        area = Ballarea()
        area.ballarea = areaball
        ballarea.publish(area)

    def send_ball_status(self, msg):
        ballstatus = BallState()
        ballstatus.ball_status = msg
        ballstate.publish(ballstatus)

    robot_heading = 0
    def get_robot_heading(msg):
        global robot_heading
        robot_heading = msg.data

    def min_value(self, a, b):
        if a <= b:
            return a
        else:
            return b

    def max_value(self, a, b):
        if a >= b:
            return a
        else:
            return b

    def map_value(self, source_val, source_min, source_max, target_min, target_max):
        source_val = (float) (self.min_value(source_val, self.max_value(source_min, source_max)))
        source_val = (float) (self.max_value(source_val, self.min_value(source_min, source_max)))
        return (float) (target_min + ((source_val - source_min) * ((target_max - target_min) / (source_max - source_min))))

    def determine_position(self, center_object, pt1, pt2):
        global keeper_action
        # Persamaan garis: y = mx + c
        x_obj, y_obj = center_object
        x1_line, y1_line = pt1
        x2_line, y2_line = pt2
        
        # Penanganan kasus pembagian oleh nol
        if x2_line - x1_line == 0:
            if x_obj < x1_line:  # Jika objek berada di sebelah kiri titik 1 garis
                print(0)
            else:  # Jika objek berada di sebelah kanan titik 1 garis
                print(1)
            return
        
        m = (y2_line - y1_line) / (x2_line - x1_line)
        c = y1_line - m * x1_line
        
        # Hitung nilai y pada garis untuk posisi x_obj
        y_line = m * x_obj + c
        
        # Bandingkan posisi objek dengan posisi garis
        if y_obj < y_line:
            print(0)  # Objek berada di atas garis
            keeper_action=False
        else:
            print(1)  # Objek berada di bawah garis
            keeper_action=True

    def get_hsv_val(self, img):
        global x_ball,y_ball,w_ball,h_ball
        global min_h, min_s, min_v, max_h, max_s, max_v, ball_area
        min_h = 0
        min_s = 0
        min_v = 0
        max_h = 0
        max_s = 0
        max_v = 0
        dot = ([[0][0],[0][0],[0][0],[0][0],[0][0],[0][0]])
        hsv_val = ([0,0,0])
        x = x_ball
        y = y_ball
        w = w_ball
        h = h_ball

        if x == None or y == None or w == None or h == None:
            None
            # print('') # print('NOT FOUND ALL')
        
        x1 = x
        y1 = y
        x2 = (x + w)
        y2 = (y + h)

        # menghitung area bola 
        ball_area = w * h
        print('LUAS BOLA :' ,ball_area)

        # menghitung nilai hsv bola 

        # KONVENSI :
        # HARUS MENGGUNAKAN x1,y1,x2,y2
        # acuan semua perhitungan adalah titik pusat

        # print(x1,y1,x2,y2)
        
        dot[0] = (x1 + x2) //2,(y1 + y2)//2 # dot_tengah
        dot[1] = (x1 + dot[0][0]) // 2 ,(y1 + dot[0][1]) // 2 #dot_sepertiga_kanan
        dot[2] = (x2 +  dot[0][0]) // 2 ,(y2 + dot[0][1]) // 2 #dot_duapertiga_kanan
        dot[3] = ( dot[0][0] + (x1 + (x2 - x1))) // 2 ,(dot[0][1] + y1) // 2 #dot_sepertiga_kiri
        dot[4] = (x1 +  dot[0][0]) // 2, (y1 + (y2 - y1) + dot[0][1]) // 2 # dot_duapertiga_kiri
        dot[5] = (dot[0][0]), (dot[0][1] + y1 + h//5)//2 # dot atas tengah
        # dot[6] = (dot[0][0]), (dot[0][1] + y2 - h//5)//2 # dot bawah tengah

        # print(dot)

        #dot tengah
        # koordinat_tengah = (iy + y)//2, (ix + x) //2
        tengah = [max(int((y1 + y2)//2), framesize[1]), max(int((x1 + x2) //2), framesize[0])]
        tengah = [min(int((y1 + y2)//2), 0), min(int((x1 + x2) //2), 0)]
        colorsB_tengah = img[tengah[0], tengah[1],0]
        colorsG_tengah = img[tengah[0], tengah[1],1]
        colorsR_tengah = img[tengah[0], tengah[1],2]
        colors_tengah = img[tengah[0], tengah[1]]
        hsv_value_tengah = np.uint8([[[colorsB_tengah ,colorsG_tengah,colorsR_tengah ]]])
        # hsv_tengah = cv2.cvtColor(hsv_value_tengah,cv2.COLOR_BGR2HSV)
        hsv_tengah = cv2.cvtColor(hsv_value_tengah, cv2.COLOR_BGR2HSV)

# sudah bisa yang atas, tinggal ngubah yang bawah

        # print ("HSV dot tengah : " ,hsv_tengah, "BRG Format: ",colors_tengah)

        #dot sepertiga kanan
        # koordinat_tengah = (iy + y)//2, (ix + x) //2
        sepertiga_kanan = [max(int((y1 + dot[0][1]) // 2), framesize[1]-1), max(int((x1 + dot[0][0]) // 2), framesize[0]-1)]
        
        colorsB_sepertiga_kanan = img[sepertiga_kanan[0], sepertiga_kanan[1],0]
        colorsG_sepertiga_kanan = img[sepertiga_kanan[0], sepertiga_kanan[1],1]
        colorsR_sepertiga_kanan = img[sepertiga_kanan[0], sepertiga_kanan[1],2]
        colors_sepertiga_kanan = img[sepertiga_kanan[0], sepertiga_kanan[1]]
        hsv_value_sepertiga_kanan = np.uint8([[[colorsB_sepertiga_kanan ,colorsG_sepertiga_kanan,colorsR_sepertiga_kanan ]]])
        hsv_sepertiga_kanan = cv2.cvtColor(hsv_value_sepertiga_kanan,cv2.COLOR_BGR2HSV)

        # print ("HSV dot sepertiga kanan : " ,hsv_sepertiga_kanan, "BRG Format: ",colors_sepertiga_kanan)

        #dot duapertiga kanan
        # koordinat_tengah = (iy + y)//2, (ix + x) //2
        duapertiga_kanan = [max(int((y2 + dot[0][1]) // 2), framesize[1]-1), max(int((x2 + dot[0][0]) // 2), framesize[0]-1)]
        colorsB_duapertiga_kanan = img[duapertiga_kanan[0], duapertiga_kanan[1],0]
        colorsG_duapertiga_kanan = img[duapertiga_kanan[0], duapertiga_kanan[1],1]
        colorsR_duapertiga_kanan = img[duapertiga_kanan[0], duapertiga_kanan[1],2]
        colors_duapertiga_kanan = img[duapertiga_kanan[0], duapertiga_kanan[1]]
        hsv_value_duapertiga_kanan = np.uint8([[[colorsB_duapertiga_kanan ,colorsG_duapertiga_kanan,colorsR_duapertiga_kanan ]]])
        hsv_duapertiga_kanan = cv2.cvtColor(hsv_value_duapertiga_kanan,cv2.COLOR_BGR2HSV)

        # print ("HSV dot duapertiga kanan : " ,hsv_duapertiga_kanan, "BRG Format: ",colors_duapertiga_kanan)

        #dot sepertiga kiri
        # koordinat_tengah = (iy + y)//2, (ix + x) //2
        sepertiga_kiri = [max(int((dot[0][1] + y1) // 2), framesize[1]-1),max(int((dot[0][0] + (x1 + (x2 - x1))) // 2),framesize[0]-1)]
        colorsB_sepertiga_kiri = img[sepertiga_kiri[0], sepertiga_kiri[1],0]
        colorsG_sepertiga_kiri = img[sepertiga_kiri[0], sepertiga_kiri[1], 1]
        colorsR_sepertiga_kiri = img[sepertiga_kiri[0], sepertiga_kiri[1],2]
        colors_sepertiga_kiri = img[sepertiga_kiri[0], sepertiga_kiri[1]]
        hsv_value_sepertiga_kiri = np.uint8([[[colorsB_sepertiga_kiri ,colorsG_sepertiga_kiri,colorsR_sepertiga_kiri ]]])
        hsv_sepertiga_kiri = cv2.cvtColor(hsv_value_sepertiga_kiri,cv2.COLOR_BGR2HSV)

        # print ("HSV dot sepertiga kiri : " ,hsv_sepertiga_kiri, "BRG Format: ",colors_sepertiga_kiri)

        #dot duapertiga kiri
        # koordinat_tengah = (iy + y)//2, (ix + x) //2
        duapertiga_kiri = [max(int((y1 + (y2 - y1) + dot[0][1]) // 2),framesize[1]-1), max(int((x1 + dot[0][0]) // 2),framesize[0]-1)]
        colorsB_duapertiga_kiri = img[duapertiga_kiri[0], duapertiga_kiri[1], 0]
        colorsG_duapertiga_kiri = img[duapertiga_kiri[0], duapertiga_kiri[1], 1]
        colorsR_duapertiga_kiri = img[duapertiga_kiri[0], duapertiga_kiri[1] ,2]
        colors_duapertiga_kiri = img[duapertiga_kiri[0], duapertiga_kiri[1]]
        hsv_value_duapertiga_kiri = np.uint8([[[colorsB_duapertiga_kiri ,colorsG_duapertiga_kiri,colorsR_duapertiga_kiri ]]])
        hsv_duapertiga_kiri = cv2.cvtColor(hsv_value_duapertiga_kiri,cv2.COLOR_BGR2HSV)

        # dot atas tengah
        # (dot[0][1]), (dot[0][0] + y1 + h//5)//2

        colorsB_atas_tengah = img[int((dot[0][1] + y1 + h//4)//3), int(dot[0][0]), 0]
        colorsG_atas_tengah = img[int((dot[0][1] + y1 + h//4)//3), int(dot[0][0]), 1]
        colorsR_atas_tengah = img[int((dot[0][1] + y1 + h//4)//3), int(dot[0][0]),2]
        # colors_atas_tengah = img[[int((dot[0][1] + y1 + h//4)//3), int(dot[0][0])]]
        hsv_value_atas_tengah = np.uint8([[[colorsB_atas_tengah ,colorsG_atas_tengah,colorsR_atas_tengah]]])
        hsv_atas_tengah = cv2.cvtColor(hsv_value_atas_tengah,cv2.COLOR_BGR2HSV)

        # print ("HSV dot duapertiga kiri : " ,hsv_duapertiga_kiri, "BRG Format: ",colors_duapertiga_kiri)
        
        # dot bawah tengah
        # (dot[0][0]), (dot[0][1] + y2 - h//5)//2

        colorsB_bawah_tengah = img[int((dot[0][1] + y2)//2), int(dot[0][0]), 0]
        colorsG_bawah_tengah = img[int((dot[0][1] + y2)//2), int(dot[0][0]), 1]
        colorsR_bawah_tengah = img[int((dot[0][1] + y2)//2), int(dot[0][0]),2]
        # colors_bawah_tengah = img[[int(dot[0][1]), int((dot[0][0] + y1 + h//4)//6)]]
        hsv_value_bawah_tengah = np.uint8([[[colorsB_bawah_tengah ,colorsG_bawah_tengah,colorsR_bawah_tengah]]])
        hsv_bawah_tengah = cv2.cvtColor(hsv_value_bawah_tengah,cv2.COLOR_BGR2HSV)

        nilai_hsv_tengah = hsv_tengah[0, 0, 0]
        nilai_hsv_sepertiga_kanan = hsv_sepertiga_kanan[0,0,0]
        nilai_hsv_duapertiga_kanan = hsv_duapertiga_kanan[0,0,0]
        nilai_hsv_sepertiga_kiri = hsv_sepertiga_kiri[0,0,0]
        nilai_hsv_duapertiga_kiri = hsv_duapertiga_kiri[0,0,0]
        nilai_hsv_atas_tengah = hsv_atas_tengah[0,0,0]
        nilai_hsv_bawah_tengah = hsv_bawah_tengah[0,0,0]

        satu_nilai_hsv_tengah = hsv_tengah[0, 0, 1]
        satu_nilai_hsv_sepertiga_kanan = hsv_sepertiga_kanan[0,0,1]
        satu_nilai_hsv_duapertiga_kanan = hsv_duapertiga_kanan[0,0,1]
        satu_nilai_hsv_sepertiga_kiri = hsv_sepertiga_kiri[0,0,1]
        satu_nilai_hsv_duapertiga_kiri = hsv_duapertiga_kiri[0,0,1]
        satu_nilai_hsv_atas_tengah = hsv_atas_tengah[0,0,1]
        satu_nilai_hsv_bawah_tengah = hsv_bawah_tengah[0,0,1]

        dua_nilai_hsv_tengah = hsv_tengah[0, 0, 2]
        dua_nilai_hsv_sepertiga_kanan = hsv_sepertiga_kanan[0,0,2]
        dua_nilai_hsv_duapertiga_kanan = hsv_duapertiga_kanan[0,0,2]
        dua_nilai_hsv_sepertiga_kiri = hsv_sepertiga_kiri[0,0,2]
        dua_nilai_hsv_duapertiga_kiri = hsv_duapertiga_kiri[0,0,2]
        dua_nilai_hsv_atas_tengah = hsv_atas_tengah[0,0,2]
        dua_nilai_hsv_bawah_tengah = hsv_bawah_tengah[0,0,2]

    # def output_hsv_val(self):
        hsv_val[0] = [nilai_hsv_tengah, nilai_hsv_sepertiga_kanan,nilai_hsv_duapertiga_kanan,nilai_hsv_sepertiga_kiri,nilai_hsv_duapertiga_kiri,nilai_hsv_bawah_tengah] # nilai_hsv_atas_tengah
        hsv_val[1] = [satu_nilai_hsv_tengah, satu_nilai_hsv_sepertiga_kanan,satu_nilai_hsv_duapertiga_kanan,satu_nilai_hsv_sepertiga_kiri,satu_nilai_hsv_duapertiga_kiri,satu_nilai_hsv_bawah_tengah] # satu_nilai_hsv_atas_tengah
        hsv_val[2] = [dua_nilai_hsv_tengah, dua_nilai_hsv_sepertiga_kanan,dua_nilai_hsv_duapertiga_kanan,dua_nilai_hsv_sepertiga_kiri,dua_nilai_hsv_duapertiga_kiri,satu_nilai_hsv_bawah_tengah] # dua_nilai_hsv_atas_tengah

        min_h = np.min(hsv_val[0])
        min_s = np.min(hsv_val[1])
        min_v = np.min(hsv_val[2])
        max_h = np.max(hsv_val[0])
        max_s = np.max(hsv_val[1])
        max_v = np.max(hsv_val[2]) 

        nilai_maksimum_h = 33
        if max_h>= nilai_maksimum_h:
            max_h = np.uint8(nilai_maksimum_h)

        nilai_minimum_s = 160
        if min_s <= nilai_minimum_s:
            min_s = np.uint8(nilai_minimum_s)
        # nilai_minimum_s = 40
        # if min_s<= nilai_minimum_s:
        #     max_s = np.uint8(nilai_minimum_s)

        # print('min hue',self.min_h,'min sat', self.min_s,'min val', self.min_v, 'max hue',self.max_h,'max sat', self.max_s,'max val', self.max_v)
        print('min hue',min_h,'min sat', min_s,'min val', min_v, 'max hue',max_h,'max sat', max_s,'max val', max_v)
 

    def ball_detect(self):
        global x_ball, y_ball, w_ball, h_ball, detect_status, min_h, min_s, min_v, max_h, max_s, max_v, ball_area, waktu_sebelum, scan_area, framesize, status, pt1, pt2, center_ball,  x_center_ball, y_center_ball, status, fps, blobsize
        global x_center_goal_0, y_center_goal_0, x_center_goal_1, y_center_goal_1, x_goal_0, y_goal_0, w_goal_0, h_goal_0, x_goal_1, y_goal_1, w_goal_1, h_goal_1
        img = capture.read()
        self.framecounter += 1
        img_result = img.copy()
        
        if detect_status == 'FOUND':
            hsv_lower = np.array([min_h, min_s, min_v])
            hsv_upper = np.array([max_h, max_s, max_v])

            config_lapangan = configparser.ConfigParser()
            config_lapangan.read('/home/robotis/catkin_ws/src/DEWO-OP3/KRI2024/Imageprocessing/v1_detection/cfg/hsv_lapangan.ini')
            lower_h = int(config_lapangan['HSV_LAPANGAN']['lower_h'])
            upper_h = int(config_lapangan['HSV_LAPANGAN']['upper_h'])
            lower_s = int(config_lapangan['HSV_LAPANGAN']['lower_s'])
            upper_s = int(config_lapangan['HSV_LAPANGAN']['upper_s'])
            lower_v = int(config_lapangan['HSV_LAPANGAN']['lower_v'])
            upper_v = int(config_lapangan['HSV_LAPANGAN']['upper_v'])

            lower_hsv_field = np.array([lower_h,lower_s,lower_v])
            upper_hsv_field = np.array([upper_h,upper_s,upper_v])

            field_kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
            hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            field_binary = cv2.inRange(hsv_image, lower_hsv_field, upper_hsv_field)
            field_binary = cv2.erode(field_binary, field_kernel, iterations = 2)
            field_binary = cv2.dilate(field_binary, field_kernel, iterations = 5)
            field_mask = np.zeros(img.shape[:2], np.uint8)
            field_contours, _ = cv2.findContours(field_binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            field_img = img.copy()
            if len(field_contours) > 0:
                field_cntr = max(field_contours, key=cv2.contourArea)
                hull = cv2.convexHull(field_cntr)
                cv2.drawContours(field_mask, [hull], 0, 255, cv2.FILLED, offset=(0,0))
                field_img = cv2.bitwise_and(img, img, mask=field_mask)

            # cv2.imshow("INDONESIA", field_img)


            # deteksi bola
            # cv2.imshow('lapangan',field_img)
            frame_copy = field_img.copy()
            # kernel = np.ones((5,5), np.float32) /25
            # frame_copy = cv2.filter2D(frame_copy, -1, kernel)

            x_center_frame = (int) (framesize[0]/2)
            y_center_frame = (int) (framesize[1]/2)

            hsv = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2HSV)

            binary_ball = cv2.inRange(hsv, hsv_lower, hsv_upper)
            kernel_bball = np.ones((5,5), np.uint8)
            binary_ball = cv2.morphologyEx(binary_ball, cv2.MORPH_CLOSE, kernel_bball)
            binary_ball = cv2.morphologyEx(binary_ball, cv2.MORPH_OPEN, kernel_bball)
            frame_copy = cv2.bitwise_and(frame_copy, frame_copy, mask=binary_ball)
            contours, hierarchy = cv2.findContours(binary_ball, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            result = img.copy()
            contourLength = len(contours)
            objek_ditemukan = False

            for contour in contours:
                area = cv2.contourArea(contour)
                if area > ball_area//5 and area < ball_area*1.1 and contourLength > 0:
                    x, y, w, h = cv2.boundingRect(contour)
                    x_center = x + w/2
                    if scan_area[0] <= x_center <= scan_area[1] and (w*h) > (3000 * fisheye):
                        # cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 255), 2) 
                        # if pt1 and pt2 is not None:
                        
                        cv2.rectangle(img_result, (x, y), (x + w, y + h), (0, 255, 255), 2) 
                        bounding_box_found = True
                        area_bola = (w*h)
                        x_center_rect = (x+w/2)
                        y_center_rect = (y+h/2)
                        center_ball = (x_center_rect, y_center_rect)
                        self.send_ball_position((float)(x_center_rect), (float)(y_center_rect), framesize[0], framesize[1], area_bola)
                        self.send_ball_area(area_bola)
                        self.send_ball_status("FOUND")
                        objek_ditemukan = True
                        # cv2.imshow('HSV', result)
                        # cv2.waitKey(1)
                        waktu_detect = self.map_value(area_bola, 0, 76800, 0.5, 80)
                        waktu_sesudah = time.time()
                        delta = waktu_sesudah - waktu_sebelum
                        
                        # Keluar dari loop jika waktu deteksi melebihi 1 detik
                        print('Ball Area Result :', area_bola)
                        if delta >= waktu_detect:
                            waktu_sebelum = time.time()
                            detect_status = 'NOTFOUND'
                            break
                    break

            if not objek_ditemukan:
                detect_status = 'NOTFOUND'

        if detect_status == 'NOTFOUND':

            layer_names = net.getLayerNames()
            output_layers = net.getUnconnectedOutLayersNames()
            height, width, channels = img.shape
            blob = cv2.dnn.blobFromImage(img, 0.00392, (blobsize, blobsize), (0, 0, 0), True, crop=False) # 416,416
            net.setInput(blob)
            outs = net.forward(output_layers)
            class_ids = []
            confidences = []
            boxes = []
            for out in outs:
                for detection in out:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]
                    if confidence > 0.5:
                        # print(class_id)
                        center_x = int(detection[0] * width)
                        center_y = int(detection[1] * height)
                        w = int(detection[2] * width)
                        h = int(detection[3] * height)
                        x = int(center_x - w / 2)
                        y = int(center_y - h / 2)
                        boxes.append([x, y, w, h])
                        confidences.append(float(confidence))
                        class_ids.append(class_id)

            indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
            daftar=[]
            if len(boxes) == 0:
                print('not found all')
                self.send_ball_status("NOTFOUND")
                blobsize = 416
                print("Did not detect ball, setting blobsize to ",blobsize)

            for i in range(len(boxes)):
                if i in indexes:
                    x, y, w, h =  boxes[i]
                    try:
                        x_ball, y_ball, w_ball, h_ball = boxes[class_ids.index(0)]     
                    except ValueError as e:
                        x_ball, y_ball, w_ball, h_ball = [0, 0, 0, 0]
                    x_center_ball = (x_ball+w_ball/2)
                    y_center_ball = (y_ball+h_ball/2)
                    center_ball = (x_center_ball, y_center_ball)
                    obj_size_ball = w_ball*h_ball
                    if ball_area <= (5000*fisheye) :
                        in_area_ball = w_ball * 3
                    elif ball_area > (5000*fisheye) :
                        in_area_ball = w_ball + (35*fisheye)
                    scan_area = [x_center_ball-in_area_ball, x_center_ball+in_area_ball]
                    label = str(classes[class_ids[i]])
                    daftar.append(label)
                    cv2.rectangle(img_result, (x, y), (x + w, y + h), (255,0,0), 1)
                    text = "{}: {:.2f}".format(label, confidences[i])
                    cv2.putText(img, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)
                    
                    if 0 in class_ids:
                        self.send_ball_position((float)(x_center_ball), (float)(y_center_ball), framesize[0], framesize[1], obj_size_ball)
                        self.send_ball_area(obj_size_ball)
                        self.send_ball_status("FOUND")
                        print('bola [{}%]'.format(np.round(confidences[class_ids.index(0)] * 100)))
                        self.get_hsv_val(img)
                        detect_status = 'FOUND'
                        if obj_size_ball <= (2800*fisheye):
                            blobsize = 320
                        elif obj_size_ball > (2800*fisheye):
                            blobsize = 224
                        break

                    else:
                        self.send_ball_position((float)(0), (float)(0), framesize[0], framesize[1], obj_size_ball)
                        self.send_ball_status("NOTFOUND")
                        print('bola not found')
                        break

            print(daftar)
        # if status == 'TRACKBALL':
        if pt1 and pt2 is not None:
            cv2.line(img_result, pt1, pt2, (0, 0, 255), 3, cv2.LINE_AA)
            self.determine_position(center_ball, pt1, pt2)

        if status == 'KEEPER':
            cv2.polylines(img_result, [batas_keeper_kiri], isClosed=False, color=(0, 255, 255), thickness=2)
            cv2.polylines(img_result, [batas_keeper_kanan], isClosed=False, color=(0, 255, 255), thickness=2)
            cv2.polylines(img_result, [batas_keeper_tengah], isClosed=False, color=(255, 0, 0), thickness=2)
        
        print(detect_status)
        self.calculate_fps()
        fps_str = "FPS: {:.2f}".format(fps)
        if pt1 and pt2 is not None:
            cv2.line(img_result, pt1, pt2, (0, 0, 255), 3, cv2.LINE_AA)
            self.determine_position(center_ball, pt1, pt2)
        if x_goal_0 and x_goal_1 is not None:
            cv2.rectangle(img_result, (x_goal_0, y_goal_0), (x_goal_0 + w_goal_0, y_goal_0 + h_goal_0), (255, 0, 255), 2)
            cv2.rectangle(img_result, (x_goal_1, y_goal_1), (x_goal_1 + w_goal_1, y_goal_1 + h_goal_1), (255, 0, 255), 2)
        cv2.putText(img_result, fps_str, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(img_result, str(blobsize), (280, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.imshow("GANDAMANA_VISION", img_result)
        cv2.waitKey(1)  # Menunggu sebentar agar jendela imshow dapat ditampilkan
        # else:
        #     cv2.destroyWindow("GANDAMANA_VISION")

class OpponentDetection:
    
    def __init__(self):
        self.lower_h, self.upper_h, self.lower_s, self.upper_s, self.lower_v, self.upper_v = self.read_hsv_config()

    def send_opponent_position(self, x_pos, y_pos, w_frame, h_frame, size_obj):
        ballposition = OpponentCoordinate()
        fix_x = (float) (x_pos / w_frame * 2 - 1)
        fix_y = (float) (y_pos / h_frame * 2 - 1)
        ballposition.pos_x = fix_x
        ballposition.pos_y = fix_y
        ballposition.obj_size = size_obj
        ballcoor.publish(ballposition)

    def send_opponent_status(self, msg):
        opponenetstatus = OpponentState()
        opponenetstatus.opponent_status = msg
        opponent_state.publish(opponenetstatus)

    def read_hsv_config(self):
        config = configparser.ConfigParser()
        config.read('/home/robotis/catkin_ws/src/DEWO-OP3/KRI2024/Imageprocessing/v1_detection/cfg/hsv_lawan_magenta.ini')

        # Membaca nilai parameter dari file
        lower_h = int(config['HSV_LAWAN_MAGENTA']['lower_h'])
        upper_h = int(config['HSV_LAWAN_MAGENTA']['upper_h'])
        lower_s = int(config['HSV_LAWAN_MAGENTA']['lower_s'])
        upper_s = int(config['HSV_LAWAN_MAGENTA']['upper_s'])
        lower_v = int(config['HSV_LAWAN_MAGENTA']['lower_v'])
        upper_v = int(config['HSV_LAWAN_MAGENTA']['upper_v'])

        return lower_h, upper_h, lower_s, upper_s, lower_v, upper_v

    def detect_opponent(self, image):
        config_lapangan = configparser.ConfigParser()
        config_lapangan.read('/home/robotis/catkin_ws/src/DEWO-OP3/KRI2024/Imageprocessing/v1_detection/cfg/hsv_lapangan.ini')
        lower_h = int(config_lapangan['HSV_LAPANGAN']['lower_h'])
        upper_h = int(config_lapangan['HSV_LAPANGAN']['upper_h'])
        lower_s = int(config_lapangan['HSV_LAPANGAN']['lower_s'])
        upper_s = int(config_lapangan['HSV_LAPANGAN']['upper_s'])
        lower_v = int(config_lapangan['HSV_LAPANGAN']['lower_v'])
        upper_v = int(config_lapangan['HSV_LAPANGAN']['upper_v'])

        lower_hsv_field = np.array([lower_h,lower_s,lower_v])
        upper_hsv_field = np.array([upper_h,upper_s,upper_v])

        field_kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(7,7))
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        field_binary = cv2.inRange(hsv_image, lower_hsv_field, upper_hsv_field)
        field_binary = cv2.erode(field_binary, field_kernel, iterations = 2)
        field_binary = cv2.dilate(field_binary, field_kernel, iterations = 7)
        field_mask = np.zeros(image.shape[:2], np.uint8)
        field_contours, _ = cv2.findContours(field_binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        field_img = image.copy()
        if len(field_contours) > 0:
            field_cntr = max(field_contours, key=cv2.contourArea)
            hull = cv2.convexHull(field_cntr)
            cv2.drawContours(field_mask, [hull], 0, 255, cv2.FILLED, offset=(0,0))
            field_img = cv2.bitwise_and(image, image, mask=field_mask)

        frame_copy = field_img.copy()
        frame_fix = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2HSV)

        lower_magenta = np.array([self.lower_h, self.lower_s, self.lower_v]) 
        upper_magenta = np.array([self.upper_h, self.upper_s, self.upper_v]) 
        mask = cv2.inRange(frame_fix, lower_magenta, upper_magenta)
        result = cv2.bitwise_and(image, image, mask=mask)
        return result, mask
        
    def calculate_overlap(self, box1, box2):
        x1_tl, y1_tl, x1_br, y1_br, _ = box1
        x2_tl, y2_tl, x2_br, y2_br, _ = box2
        area_box1 = (x1_br - x1_tl) * (y1_br - y1_tl)
        area_box2 = (x2_br - x2_tl) * (y2_br - y2_tl)
        x_tl = max(x1_tl, x2_tl)
        y_tl = max(y1_tl, y2_tl)
        x_br = min(x1_br, x2_br)
        y_br = min(y1_br, y2_br)
        overlap_area = max(0, x_br - x_tl) * max(0, y_br - y_tl)
        overlap_ratio = overlap_area / min(area_box1, area_box2)
        return overlap_ratio

    def non_max_suppression(self, boxes, threshold, min_region):
        sorted_boxes = sorted(boxes, key=lambda x: x[4], reverse=True)
        kept_boxes = []
        for box in sorted_boxes:
            keep_box = True
            for kept_box in kept_boxes:
                if self.calculate_overlap(kept_box, box) > threshold:
                    if (kept_box[2] - kept_box[0]) * (kept_box[3] - kept_box[1]) < (box[2] - box[0]) * (box[3] - box[1]):
                        kept_boxes.remove(kept_box)
                    else:
                        keep_box = False
                    break
            if keep_box and ((box[2] - box[0]) * (box[3] - box[1]) >= min_region):
                kept_boxes.append(box)
        return kept_boxes

    def run(self):
        global status
        frame = capture.read()
        frame_copy2 = frame.copy()
        opponent_detection_result, mask = self.detect_opponent(frame)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boxes = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            boxes.append((x, y, x + w, y + h, 1))
        final_boxes = self.non_max_suppression(boxes, threshold=0.5, min_region=50)
        for box in final_boxes:
            x1, y1, x2, y2, _ = box
            cv2.rectangle(frame_copy2, (x1, y1), (x2, y2), (0, 255, 255), 2)
            self.send_opponent_status("MAGENTA_FOUND")

        if status == 'LAWAN_MAGENTA':
            cv2.imshow("DETEKSI_LAWAN", frame_copy2)
            cv2.waitKey(1)
        else:
            cv2.destroyWindow("DETEKSI_LAWAN")

x_center_goal_0 = 0
y_center_goal_0 = 0
x_center_goal_1 = 0
y_center_goal_1 = 0
x_goal_0 = 0
y_goal_0 = 0
w_goal_0 = 0
h_goal_0 = 0
x_goal_1 = 0
y_goal_1 = 0
w_goal_1 = 0
h_goal_1 = 0

class goal_detection:

    def __init__(self):
        None

    def send_goal_position(self, x_pos, y_pos, w_frame, h_frame, size_obj):
        goalposition = GoalCoordinate()
        fix_x = (float) (x_pos / w_frame * 2 - 1)
        fix_y = (float) (y_pos / h_frame * 2 - 1)
        goalposition.pos_x = fix_x
        goalposition.pos_y = fix_y
        goalposition.obj_size = size_obj
        goal_pos.publish(goalposition)

    def send_goal_status(self, msg):
        goalstatus = GoalState()
        goalstatus.goal_status = msg
        goal_state.publish(goalstatus)

    def detect(self):
        global x_center_goal_0, y_center_goal_0, x_center_goal_1, y_center_goal_1, x_goal_0, y_goal_0, w_goal_0, h_goal_0, x_goal_1, y_goal_1, w_goal_1, h_goal_1
        img = capture.read()
        layer_names = net_goal.getLayerNames()
        output_layers = net_goal.getUnconnectedOutLayersNames()
        blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False) # 416,416
        net_goal.setInput(blob)
        outs = net_goal.forward(output_layers)
        class_ids = []
        confidences = []
        boxes = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:
                    # print(class_id)
                    center_x = int(detection[0] * framesize[0])
                    center_y = int(detection[1] * framesize[1])
                    w = int(detection[2] * framesize[0])
                    h = int(detection[3] * framesize[1])
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        daftar=[]
        if len(boxes) == 0:
            print('not found all')
            self.send_goal_status("NOTFOUND")

        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h =  boxes[i]
                try:
                    x_goal_0, y_goal_0, w_goal_0, h_goal_0 = boxes[range(class_ids.count(0))[0]]
                    x_goal_1, y_goal_1, w_goal_1, h_goal_1 = boxes[range(class_ids.count(0))[1]]
                    # print(range(class_ids.count(1))[1])
                except IndexError as e:
                    x_goal_0, y_goal_0, w_goal_0, h_goal_0 = [0, 0, 0, 0]
                    x_goal_1, y_goal_1, w_goal_1, h_goal_1 = [0, 0, 0, 0]

                label = str(classes_goal[class_ids[i]])
                daftar.append(label)
                cv2.rectangle(img, (x, y), (x + w, y + h), (255,0,0), 1)
                text = "{}: {:.2f}".format(label, confidences[i])
                cv2.putText(img, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)
                
                if 0 in class_ids:
                    if class_ids.count(0) > 1:
                        x_center_goal_0 = (x_goal_0+w_goal_0/2)
                        y_center_goal_0 = (y_goal_0+h_goal_0/2)
                        x_center_goal_1 = (x_goal_1+w_goal_1/2)
                        y_center_goal_1 = (y_goal_1+h_goal_1/2)
                        cv2.line(img, (int(x_center_goal_0), int(y_center_goal_0)), (int(x_center_goal_1),int(y_center_goal_1)),(255,0,0),2)
                        # print("X:%d Y:%d" % ((x_center_goal_0 + x_center_goal_1) / 2, (y_center_goal_0 + y_center_goal_1) / 2))
                        cv2.circle(img, (int((x_center_goal_0 + x_center_goal_1) / 2), int((y_center_goal_0 + y_center_goal_1) / 2)), 1, (0,0,255), 2)
                        self.send_goal_position((float)((x_center_goal_0 + x_center_goal_1) / 2), (float)((y_center_goal_0 + y_center_goal_1) / 2), 320.0, 240.0, 0)
                        self.send_goal_status("FOUND")
                        print('gawang 1+2 found')
                    else:
                        if np.abs(robot_heading) < 100 and np.abs(robot_heading) > 80: 
                        # send_goal_position((float)(x_goal_0+w_goal_0/2),(float)(y_goal_0+h_goal_0/2), 320.0, 240.0, 0)
                            self.send_goal_status("NOTFOUND")
                            print('gawang 1 found [in range]')
                        else:
                            self.send_goal_position((float)(x_goal_0+w_goal_0/2),(float)(y_goal_0+h_goal_0/2), 320.0, 240.0, 0)
                            self.send_goal_status("FOUND")
                            print('gawang 1 found [out range]')
                else:
                    # send_goal_position((float)(0), (float)(0), 320.0, 240.0, obj_size_goal)
                    self.send_goal_status("NOTFOUND")
                    print('gawang not found')
        cv2.imshow("INDONESIA", img)
        cv2.waitKey(1)

class goal_line_transform:

    def __init__(self):
        pass

    def compute_confidence(self, rho, theta):

        confidence = rho
        return confidence

    def send_keeper_status(self, data):
        keeperstatus = String()
        keeperstatus.data = data
        keeper_status_publisher.publish(keeperstatus)

    def hough_line(self):
        global pt1, pt2, batas_keeper_kiri,batas_keeper_kanan,batas_keeper_tengah

        image = capture.read()

        config_lapangan = configparser.ConfigParser()
        config_lapangan.read('/home/robotis/catkin_ws/src/DEWO-OP3/KRI2024/Imageprocessing/v1_detection/cfg/hsv_lapangan.ini')
        lower_h = int(config_lapangan['HSV_LAPANGAN']['lower_h'])
        upper_h = int(config_lapangan['HSV_LAPANGAN']['upper_h'])
        lower_s = int(config_lapangan['HSV_LAPANGAN']['lower_s'])
        upper_s = int(config_lapangan['HSV_LAPANGAN']['upper_s'])
        lower_v = int(config_lapangan['HSV_LAPANGAN']['lower_v'])
        upper_v = int(config_lapangan['HSV_LAPANGAN']['upper_v'])

        lower_hsv_field = np.array([lower_h,lower_s,lower_v])
        upper_hsv_field = np.array([upper_h,upper_s,upper_v])

        field_kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(7,7))
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        field_binary = cv2.inRange(hsv_image, lower_hsv_field, upper_hsv_field)
        field_binary = cv2.erode(field_binary, field_kernel, iterations = 2)
        field_binary = cv2.dilate(field_binary, field_kernel, iterations = 7)
        field_mask = np.zeros(image.shape[:2], np.uint8)
        field_contours, _ = cv2.findContours(field_binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        field_img = image.copy()
        if len(field_contours) > 0:
            field_cntr = max(field_contours, key=cv2.contourArea)
            hull = cv2.convexHull(field_cntr)
            cv2.drawContours(field_mask, [hull], 0, 255, cv2.FILLED, offset=(0,0))
            field_img = cv2.bitwise_and(image, image, mask=field_mask)

        frame_copy_lapangan = field_img.copy()
        # cv2.imshow("frame_copy_lapangan", frame_copy_lapangan)
        hsv_frame = cv2.cvtColor(frame_copy_lapangan, cv2.COLOR_BGR2HSV)

        config_keeper = configparser.ConfigParser()
        config_keeper.read('/home/robotis/catkin_ws/src/DEWO-OP3/KRI2024/Imageprocessing/v1_detection/cfg/hsv_keeper.ini')
        lower_h = int(config_keeper['HSV_KEEPER']['lower_h'])
        upper_h = int(config_keeper['HSV_KEEPER']['upper_h'])
        lower_s = int(config_keeper['HSV_KEEPER']['lower_s'])
        upper_s = int(config_keeper['HSV_KEEPER']['upper_s'])
        lower_v = int(config_keeper['HSV_KEEPER']['lower_v'])
        upper_v = int(config_keeper['HSV_KEEPER']['upper_v'])

        lower_bound = np.array([lower_h, lower_s, lower_v])
        upper_bound = np.array([upper_h, upper_s, upper_v])
        mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)
        masked_frame = cv2.bitwise_and(frame_copy_lapangan, frame_copy_lapangan, mask=mask)

        # deteksi bola
        # cv2.imshow('hasil bitwise and',masked_frame)
        frame_copy = masked_frame.copy()

        img_gray = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2GRAY)
        x = 200
        cannyedge = cv2.Canny(img_gray, x, x + 100, None, 7)

        cdst = cv2.cvtColor(cannyedge, cv2.COLOR_GRAY2BGR)
        lines = cv2.HoughLines(cannyedge, 1, np.pi / 180, 130, None, 0, 0)

        max_rho = 0
        max_theta = 0
        max_confidence = 0

        if lines is not None:
            for line in lines:
                rho = line[0][0]
                theta = line[0][1]
                a = math.cos(theta)
                b = math.sin(theta)
                x0 = a * rho
                y0 = b * rho

                confidence = self.compute_confidence(rho, theta)
                if confidence > max_confidence:
                    max_rho = rho
                    max_theta = theta
                    max_confidence = confidence

        if max_confidence > 0.5: 
            a = math.cos(max_theta)
            b = math.sin(max_theta)
            x0 = a * max_rho
            y0 = b * max_rho
            pt1 = ((int(x0 + 1000*(-b))-70), ((int(y0 + 1000*(a))-70)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
            cv2.line(cdst, pt1, pt2, (0, 0, 255), 3, cv2.LINE_AA)

        self.ball_location()
        # cv2.imshow("Source", img)
        # cv2.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst)
        cv2.waitKey(1)
    
    # def ball_location(self):
    #     global x_center_ball, y_center_ball, keeper_action,detect_status

    #     if keeper_action == False or detect_status == 'NOTFOUND':
    #         self.send_keeper_status("X")
    #     elif x_center_ball >= 140 and x_center_ball <= 180:
    #         print('bola berada di tengah')
    #         if keeper_action == True:
    #             # self.keeper_status_publisher.publish("B")
    #             self.send_keeper_status("B")
    #             print('MOTION HALANG DUDUK')
    #     elif x_center_ball < 140: # x_center_ball >= 100 and 
    #         print('di samping KIRI')
    #         if keeper_action == True:
    #             self.send_keeper_status("A")
    #             # self.keeper_status_publisher.publish("A")
    #             print('MOTION HALANG SAMPING KIRI')
    #     elif x_center_ball > 180: # and x_center_ball <= 220
    #         print('di samping KANAN')
    #         if keeper_action == True:
    #             self.send_keeper_status("C")
    #             # self.keeper_status_publisher.publish("C")
    #             print('MOTION HALANG SAMPING KANAN')

    # def ball_location(self):
    #     global x_center_ball, y_center_ball, keeper_action,detect_status, ball_area, prev_area
    #     if prev_area != 0:
    #         delta_area = ball_area - prev_area
    #         if delta_area > treeshold :
    #             if keeper_action == False or detect_status == 'NOTFOUND':
    #                 self.send_keeper_status("X")
    #             elif x_center_ball >= 120 and x_center_ball <= 200:
    #                 print('bola berada di tengah')
    #                 if keeper_action == True:
    #                     self.send_keeper_status("B")
    #                     print('MOTION HALANG DUDUK')
    #             elif x_center_ball < 120: # x_center_ball >= 100 and 
    #                 print('di samping KIRI')
    #                 if keeper_action == True:
    #                     self.send_keeper_status("A")
    #                     print('MOTION HALANG SAMPING KIRI')
    #             elif x_center_ball > 200: # and x_center_ball <= 220
    #                 print('di samping KANAN')
    #                 if keeper_action == True:
    #                     self.send_keeper_status("C")
    #                     print('MOTION HALANG SAMPING KANAN')
    #     prev_area = ball_area

    def ball_location(self):
        global center_ball, keeper_action
        if keeper_action == False or detect_status == 'NOTFOUND':
                self.send_keeper_status("X")
        elif cv2.pointPolygonTest(batas_keeper_kiri[0], center_ball, False) >= 0:
            if keeper_action == True:
                self.send_keeper_status("A")
        elif cv2.pointPolygonTest(batas_keeper_tengah[0], center_ball, False) >= 0:
            if keeper_action == True:
                self.send_keeper_status("B")
        elif cv2.pointPolygonTest(batas_keeper_kanan[0], center_ball, False) >= 0:
            if keeper_action == True:
                self.send_keeper_status("C")


# ---------------------SUBSCRIBER--------------------------------------------------

def detection_status(data):
    global status
    if data.data ==  "A":
        status = "LAWAN_MAGENTA"
    elif data.data == "B":
        status = "KEEPER"        
    elif data.data == "C":
        status = "TRACKBALL"
    else:
        status = "TRACKBALL"

robot_heading = 0
def get_robot_heading(msg):
    global robot_heading
    robot_heading = msg.data
# ---------------------SUBSCRIBER--------------------------------------------------

# --------------ITERASI 1x------------------------------------
subprocess.call(['sh', '/home/robotis/catkin_ws/src/DEWO-OP3/KRI2025/ImageProcessing/v2_detection/src/runs/camera_setting.sh'])
net = cv2.dnn.readNet("/home/robotis/catkin_ws/src/DEWO-OP3/KRI2024/Imageprocessing/v1_detection/cfg/bola.cfg","/home/robotis/catkin_ws/database/weights/bola_nasional.weights")
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
net_goal = cv2.dnn.readNet("/home/robotis/catkin_ws/src/DEWO-OP3/KRI2024/Imageprocessing/v1_detection/cfg/bola.cfg","/home/robotis/catkin_ws/database/weights/gawang_nasional.weights")
net_goal.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net_goal.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

ballstate = rospy.Publisher("/DEWO/image_processing/deteksi_bola/ball_state", BallState, queue_size=10)
ballcoor = rospy.Publisher("/DEWO/image_processing/deteksi_bola/coordinate", BallCoordinate, queue_size=10)
ballarea = rospy.Publisher("/DEWO/image_processing/deteksi_bola/ball_area", Ballarea, queue_size=10)
opponent_coordinate = rospy.Publisher("/DEWO/image_processing/deteksi_bola/opponent_coordinate", OpponentCoordinate, queue_size=10)
opponent_state = rospy.Publisher("/DEWO/image_processing/deteksi_bola/opponent_state", OpponentState, queue_size=10)
keeper_status_publisher = rospy.Publisher("/DEWO/image_processing/deteksi_bola/keeper_action", String, queue_size=10)

goal_pos = rospy.Publisher("/DEWO/image_processing/deteksi_bola/goal_coordinate", GoalCoordinate, queue_size=10)
goal_state = rospy.Publisher("/DEWO/image_processing/deteksi_bola/goal_state", GoalState, queue_size=10)

rospy.Subscriber("/DEWO/image_processing/deteksi_status/bola_lawan", String, detection_status)
rospy.Subscriber("/DEWO/MotionControl/yaw", Int16, get_robot_heading)

classes = []
with open("/home/robotis/catkin_ws/src/DEWO-OP3/KRI2024/Imageprocessing/v1_detection/class/bola_fisheye.txt", "r") as f:
    classes = f.read().splitlines()

classes_goal = []
with open("/home/robotis/catkin_ws/src/DEWO-OP3/KRI2024/Imageprocessing/v1_detection/class/gawang.txt", "r") as f:
    classes_goal = f.read().splitlines()

capture = WebcamVideoStream(src=0).start() 
deteksi = ball_detection()
opponent_detection_system = OpponentDetection()
line_detect = goal_line_transform()
goal_detect = goal_detection()

def hough_line_task():
    line_detect.hough_line()

def ball_detect_task():
    deteksi.ball_detect()

if __name__ == '__main__':
    try:
        rospy.init_node("vision_yv5", anonymous=False)
        
        while not rospy.is_shutdown():
            if status == 'TRACKBALL':
                deteksi.ball_detect()
                # goal_detect.detect()
                print(status)
            elif status == 'TRACKGOAL':
                goal_detect.detect()
                print(status)
            elif status == 'LAWAN_MAGENTA':
                opponent_detection_system.run()
                print(status)
            elif status == 'KEEPER':
                line_detect.hough_line()
                deteksi.ball_detect()
                
        capture.stop()

    except rospy.ROSInterruptException:
        pass