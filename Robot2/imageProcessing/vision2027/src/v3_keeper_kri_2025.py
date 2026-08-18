#!/usr/bin/env python3.7

import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

import time
import math
import rospy
from threading import Thread
import configparser
import numpy as np
from collections import deque
import subprocess
from std_msgs.msg import String, Bool
from v1_detection.msg import BallState, Ballarea, BallCoordinate, OpponentState, OpponentCoordinate, Desicion


min_h, min_s, min_v, max_h, max_s, max_v = 0, 0, 0, 0, 0, 0

x_ball, y_ball, w_ball, h_ball = 0, 0, 0, 0
positions = [[0,0],[0,0]]
pt1 = (0,0)
pt2 = (0,0)
pt1_result = (0,0)
pt2_result = (0,0)
fps = 0
status_kiper = "X"

# ---------------------------------------------------------------------------------
# jika meenggunakan fisheye maka gunakan 0.34
fisheye = 1
# jika tidak menggunakan fisheye maka gunakan 1
# ---------------------------------------------------------------------------------

detect_status = 'NOTFOUND'
move_status = False
waktu_sebelum = time.time()
ball_area = 0
center_ball = [0,0]
x_center_ball, y_center_ball = 0, 0
scan_area = [0,0]
framesize = [320,240]
MOVE_THRESHOLD = 36
status = 'KEEPER' # TRACKBALL , LAWAN_MAGENTA, KEEPER
blobsize = 416
keeper_action = False
fall_action = False
robot_heading = 0
extended_point = [0,0]
mapped_maxlen = 4

batas_keeper_kiri = np.array([[[0, 70], [150, 70], [90, 240], [0, 240], [0, 70]]], np.int32)
batas_keeper_kanan = np.array([[[230, 240], [170, 70], [320, 70], [320,240], [230, 240]]], np.int32)
batas_keeper_tengah = np.array([[[90, 240], [160, 40], [230, 240], [90, 240]]], np.int32)
x1_result, y1_result , x2_result, y2_result = 0, 0, 0, 0

class WebcamVideoStream:
    def __init__(self, src=0):
        self.stream = cv2.VideoCapture(src, cv2.CAP_V4L2)
        self.stream.set(cv2.CAP_PROP_FPS, 240)
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

# class opponent_detection:
#     def nrgb_lbp(self):
def min_value(a, b):
    if a <= b:
        return a
    else:
        return b

def max_value(a, b):
    if a >= b:
        return a
    else:
        return b


def map_value(source_val, source_min, source_max, target_min, target_max):
    source_val = (float) (min_value(source_val, max_value(source_min, source_max)))
    source_val = (float) (max_value(source_val, min_value(source_min, source_max)))
    return (float) (target_min + ((source_val - source_min) * ((target_max - target_min) / (source_max - source_min))))
        

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
        global keeper_action, detect_status
        x_obj, y_obj = center_object
        x1_line, y1_line = pt1
        x2_line, y2_line = pt2
        
        if x2_line - x1_line == 0:
            if x_obj < x1_line:
                None
            else:
                None
            return
        
        m = (y2_line - y1_line) / (x2_line - x1_line)
        c = y1_line - m * x1_line
        y_line = m * x_obj + c
        
        if y_obj > y_line:
            keeper_action=True
            send_action_status(1)
            send_keeper_determine(True)
        else:
            keeper_action=False
            send_keeper_determine(False)
            send_action_status(0)

    def determine_fall(self, center_object, pt1_result, pt2_result):
        global fall_action
        x_obj, y_obj = center_object
        x1_line, y1_line = pt1_result
        x2_line, y2_line = pt2_result
        
        if x2_line - x1_line == 0:
            if x_obj < x1_line:
                None
            else:
                None
            return
        
        m = (y2_line - y1_line) / (x2_line - x1_line)
        c = y1_line - m * x1_line
        y_line = m * x_obj + c
        
        if y_obj < y_line:
            fall_action=False
        else:
            fall_action=True

    def get_hsv_val(self, img):
        global x_ball,y_ball,w_ball,h_ball, min_h, min_s, min_v, max_h, max_s, max_v, ball_area
        min_h, min_s, min_v, max_h, max_s, max_v = 0, 0, 0, 0, 0, 0
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
        # print('LUAS BOLA :' ,ball_area)

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

        # print('min hue',min_h,'min sat', min_s,'min val', min_v, 'max hue',max_h,'max sat', max_s,'max val', max_v)
 
    def ball_detect(self):

        global x_ball, y_ball, w_ball, h_ball, detect_status, min_h, min_s, min_v, max_h, max_s, max_v, ball_area, waktu_sebelum, scan_area, framesize, status, pt1, pt2, center_ball, x_center_ball, y_center_ball, status, fps, blobsize, positions, move_status, extended_point, pt1_result, pt2_result, status_kiper
        img = capture.read()
        self.framecounter += 1
        img_result = img.copy()
        print(status_kiper)
        
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

            # deteksi bola
            frame_copy = field_img.copy()

            hsv = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2HSV)
            binary_ball = cv2.inRange(hsv, hsv_lower, hsv_upper)
            kernel_bball = np.ones((5,5), np.uint8)
            binary_ball = cv2.morphologyEx(binary_ball, cv2.MORPH_CLOSE, kernel_bball)
            binary_ball = cv2.morphologyEx(binary_ball, cv2.MORPH_OPEN, kernel_bball)
            frame_copy = cv2.bitwise_and(frame_copy, frame_copy, mask=binary_ball)
            contours, hierarchy = cv2.findContours(binary_ball, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contourLength = len(contours)
            objek_ditemukan = False

            for contour in contours:
                area = cv2.contourArea(contour)
                if (area > (ball_area//5)) and (area < (ball_area*1.1)) and (contourLength > 0):
                    x, y, w, h = cv2.boundingRect(contour)
                    x_center_ball = (x + w/2)
                    if scan_area[0] <= x_center_ball <= scan_area[1] and (w*h) > (2500 * fisheye):
                        cv2.rectangle(img_result, (x, y), (x + w, y + h), (0, 255, 255), 2) 
                        ball_area = (w*h)
                        y_center_ball = (y+h/2)
                        center_ball = (x_center_ball, y_center_ball)
                        self.send_ball_position((float)(x_center_ball), (float)(y_center_ball), framesize[0], framesize[1], ball_area)
                        self.send_ball_area(ball_area)
                        self.send_ball_status("FOUND")
                        objek_ditemukan = True
                        waktu_detect = self.map_value(ball_area, 0, 76800, 0.5, 80)
                        waktu_sesudah = time.time()
                        delta = waktu_sesudah - waktu_sebelum
                        
                        # print('Ball Area Result :', ball_area)
                        if delta >= waktu_detect:
                            waktu_sebelum = time.time()
                            detect_status = 'NOTFOUND'
                            break
                    break

            if not objek_ditemukan:
                detect_status = 'NOTFOUND'

        if detect_status == 'NOTFOUND':

            output_layers = net.getUnconnectedOutLayersNames()
            blob = cv2.dnn.blobFromImage(img, 0.00392, (blobsize, blobsize), (0, 0, 0), True, crop=False)
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
                print('not found ball')
                detect_status = 'NOTFOUND'
                self.send_ball_status("NOTFOUND")
                blobsize = 416

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
                    ball_area = w_ball*h_ball
                    if ball_area <= (5000*fisheye) :
                        in_area_ball = w_ball * 3
                    elif ball_area > (5000*fisheye) :
                        in_area_ball = w_ball + (35*fisheye)
                    scan_area = [x_center_ball-in_area_ball, x_center_ball+in_area_ball]
                    label = str(classes[class_ids[i]])
                    daftar.append(label)
                    cv2.rectangle(img_result, (x, y), (x + w, y + h), (255,0,0), 1)
                    
                    if 0 in class_ids:
                        self.send_ball_position((float)(x_center_ball), (float)(y_center_ball), framesize[0], framesize[1], ball_area)
                        self.send_ball_area(ball_area)
                        self.send_ball_status("FOUND")
                        # print('bola [{}%]'.format(np.round(confidences[class_ids.index(0)] * 100)))
                        self.get_hsv_val(img)
                        detect_status = 'FOUND'
                        if ball_area <= (2800*fisheye):
                            blobsize = 320
                        elif ball_area > (2800*fisheye):
                            blobsize = 224
                        break

                    else:
                        self.send_ball_position((float)(0), (float)(0), framesize[0], framesize[1], ball_area)
                        self.send_ball_status("NOTFOUND")
                        detect_status = 'NOTFOUND'
                        print('bola not found')
                        break

            # print(daftar)

        if status == 'KEEPER':
            # cv2.polylines(img_result, [batas_keeper_kiri], isClosed=False, color=(0, 255, 255), thickness=2)
            # cv2.polylines(img_result, [batas_keeper_kanan], isClosed=False, color=(0, 255, 255), thickness=2)
            # cv2.polylines(img_result, [batas_keeper_tengah], isClosed=False, color=(255, 0, 0), thickness=2)
            if pt1 and pt2 is not None:
                cv2.line(img_result, pt1, pt2, (0, 0, 255), 2, cv2.LINE_AA)
                cv2.line(img_result, pt1_result, pt2_result, (255, 255, 255), 2, cv2.LINE_AA)
                print(pt1)
                print("-------------")
                print(pt2)
                self.determine_position(center_ball, pt1, pt2)
                # self.determine_fall(center_ball, pt1_result, pt2_result)
                print("JAATUH TIDAK>", fall_action)
            if move_status == True:
                cv2.line(img_result, positions[0], positions[1], (0,0,255),2)
                cv2.line(img_result, (positions[0]), (extended_point), (0, 255, 255), 2)
                cv2.circle(img, extended_point, 5, (255, 255, 255), -1)
                cv2.circle(img_result, (int(x_center_ball), int(y_center_ball)), 2, (0, 255, 255), 2)
            if status_kiper == "A":
                merah = (0,0,255)
            elif status_kiper == "B":
                merah = (0,255,0)
            elif status_kiper == "C":
                merah = (255,0,0)
            elif status_kiper == "X":
                merah = (255,255,0)
            cv2.rectangle(img_result, (140, 10),(180,20), merah, -1)

        # print(detect_status)
        self.calculate_fps()
        fps_str = "FPS: {:.2f}".format(fps)
        # print('POSISISIIII :', positions)
        cv2.putText(img_result, fps_str, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(img_result, str(blobsize), (280, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.line(img_result, (x1_result, y1_result), (x2_result, y2_result), (0, 0, 255), 2)
        cv2.imshow("GANDAMANA_VISION", img_result)
        cv2.waitKey(1)

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

keeperline = False

class goal_line_transform:

    def __init__(self):
        global fps, mapped_maxlen
        self.movetreeshold = 0
        self.distance = 0
        self.prev_area = None
        self.prev_time = time.time()
        self.time_threshold = 1.0
        mapped_maxlen = int(map_value(fps, 1, 100, 3, 20))
        self.areas = deque(maxlen=mapped_maxlen)

    def compute_confidence(self, rho, theta):
        confidence = rho
        return confidence

    def send_keeper_status(self, data):
        keeperstatus = String()
        keeperstatus.data = data
        keeper_status_publisher.publish(keeperstatus)

# ---------------------- MAP DARI NILAI KECIL KE BESAR ----------------------------------------------------------------------

# ---------------------- MAP DARI NILAI BESAR KE KECIL ----------------------------------------------------------------------

    def min_value(self, a, b):
        return a if a <= b else b

    def max_value(self, a, b):
        return a if a >= b else b

    def map_value(self, source_val, source_min, source_max, target_min, target_max):
        # Pastikan source_val berada dalam rentang source_min dan source_max
        source_val = self.max_value(source_min, self.min_value(source_val, source_max))
        
        # Lakukan pemetaan nilai
        return target_min + ( (source_val - source_min) * (target_max - target_min) / (source_max - source_min) )

    def predict_ball_path(self, positions):
        if len(positions) < 2:
            return None

        x_coords = np.array([pos[0] for pos in positions])
        y_coords = np.array([pos[1] for pos in positions])
        A = np.vstack([x_coords, np.ones(len(x_coords))]).T
        m, c = np.linalg.lstsq(A, y_coords, rcond=None)[0]
        return m, c
    
    def calculate_distance(self, pos0, pos1):
        distance = np.sqrt((pos1[0] - pos0[0])**2 + (pos1[1] - pos0[1])**2)
        return distance
    
    def calculate_extended_points(self, start_point, m):
        global framesize
        x1, y1 = start_point
        print ("NILAI GRADIENNYA ADALAH :", m)
        if m > 0.75:
            resultpoint = (((framesize[1]-y1)/m)+x1)
            return (int(resultpoint),framesize[1])
        if m == 0.75:
            return (framesize[0],framesize[1])
        if m < 0.75:
            resultpoint = m * (framesize[0]-x1) + y1
            return (framesize[0],int(resultpoint))

    def clip_point(self, point, frame_size):
        x, y = point
        x = max(0, min(frame_size[0], x))
        y = max(0, min(frame_size[1], y))
        return (x, y)

    def hough_line(self):
        global fps, mapped_maxlen, pt1, pt2, batas_keeper_kiri,batas_keeper_kanan,batas_keeper_tengah, positions, framesize, move_status, extended_point, pt1_result, pt2_result, MOVE_THRESHOLD, keeperline

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
                pt1 = ((int(x0 + 1000*(-b))), ((int(y0 + 1000*(a)))-0)) # -70
                pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a))-0)
                pt1_result = ((int(x0 + 1000*(-b))), ((int(y0 + 1000*(a)))+20))
                pt2_result = (int(x0 - 1000*(-b)), int(y0 - 1000*(a))+20)
                keeperline = True
        keeperline = False

        self.distance = self.calculate_distance(positions[0], positions[1])
        # keeper_desicion = get_desicion(self.distance)
        # print("JARAK DISTANCEEEE :", self.distance)
        # print("Desicion: ", keeper_desicion)
        # self.movetreeshold = self.map_value(fps, 2, 100, 36, 0)
        self.movetreeshold = self.map_value(fps, 2, 160, 45, 2)
        print("MOVE TREESHOLD NEW:", self.movetreeshold)

        if self.distance > self.movetreeshold:
            print("moving ball")
            move_status = True
            dy = positions[1][1] - positions[1][0] # (y2-y1)
            dx = positions[0][1] - positions[0][0] # (x2-x1)
            if dx != 0:
                m = dy / dx
                if m == 0:
                    extended_point = (framesize[0],positions[1][0])
                elif m != 0:
                    extended_point = self.calculate_extended_points(positions[0], m)
            elif dx == 0:
                extended_point = (positions[0][0], framesize[1])
            # dx = positions[0][1] - positions[0][0]
            # if dx == 0:
            #     dx = 1
            # dy = positions[1][1] - positions[1][0]
            # m = dy / dx
            # extended_point = self.calculate_extended_points(dx, positions[0], m, framesize[0], framesize[1])
            print("EXXTENDED POINT", extended_point)
            positions[0] = positions[1]
        else:
            print("stopped ball")
            move_status = False 
            # positions[0] = (int(x_center_ball), int(y_center_ball))

        positions[1] = (int(x_center_ball), int(y_center_ball))

        self.ball_location()

    def ball_location(self):
        global status_kiper, ball_area
        self.areas.append(ball_area)
        print("NILAI MAXLEEENNNN :", mapped_maxlen)
        print("AREAAAA :", self.areas)

        if len(self.areas) > 1:
            print("AREAA  1 :", self.areas[0])
            print("AREAA   2 :", self.areas[-1])
            initial_area = self.areas[0]
            final_area = self.areas[-1]

            if detect_status == 'FOUND' and final_area > 2 + initial_area:
                # rospy.loginfo("Action: A/B/C (Ball Approaching Significantly)")
                if detect_status == 'FOUND' and keeper_action and (cv2.pointPolygonTest(batas_keeper_kiri[0], center_ball, False) >= 0): #  and self.distance > self.movetreeshold
                    status_kiper = "A"
                    self.send_keeper_status("A")
                elif detect_status == 'FOUND' and keeper_action and (cv2.pointPolygonTest(batas_keeper_tengah[0], center_ball, False) >= 0):
                    self.send_keeper_status("B")
                    status_kiper = "B"
                elif detect_status == 'FOUND' and keeper_action and (cv2.pointPolygonTest(batas_keeper_kanan[0], center_ball, False) >= 0) :
                    self.send_keeper_status("C")
                    status_kiper = "C"
                else :
                    self.send_keeper_status("X")
                    status_kiper = "X"
            elif detect_status == 'FOUND' and fall_action and (cv2.pointPolygonTest(batas_keeper_tengah[0], center_ball, False) >= 0):
                status_kiper = "B"
                self.send_keeper_status("B")
            elif detect_status == 'FOUND' and fall_action and (cv2.pointPolygonTest(batas_keeper_kiri[0], center_ball, False) >= 0):
                status_kiper = "A"
                self.send_keeper_status("A")
            elif detect_status == 'FOUND' and fall_action and (cv2.pointPolygonTest(batas_keeper_kanan[0], center_ball, False) >= 0):
                status_kiper = "C"
                self.send_keeper_status("C")

                # Add your logic for actions A, B, C here
            # elif final_area <= initial_area:
            # rospy.loginfo("Action: X (Ball Not Approaching Significantly or Receding)")
            else :
                print("aksi jatuh:",fall_action)
                self.send_keeper_status("X")
                status_kiper = "X"
            # else:
            #     print("aksi jatuh:",fall_action)
            #     self.send_keeper_status("X")
            #     status_kiper = "X"

        else :
            print("aksi jatuh:",fall_action)
            self.send_keeper_status("X")
            status_kiper = "X"

    # def ball_location(self, current_area):
    #     current_time = time.time()
    #     if self.prev_time is not None and (current_time - self.prev_time) >= self.time_threshold:
    #         if self.prev_area is not None:
    #             delta_area = current_area - self.prev_area
    #             if delta_area > 0:
    #                 rospy.loginfo("Action: A/B/C (Ball Approaching)")
    #                 # Add your logic for actions A, B, C here
    #             else:
    #                 rospy.loginfo("Action: X (Ball Receding)")
    #         else:
    #             rospy.loginfo("Initial detection, no previous area to compare")

    #         self.prev_area = current_area  # Update previous area
    #         self.prev_time = current_time  # Update previous time

# -------------------------------------------------------------------- FIX 1 ------------------------------------------------------------------------------------

    # def ball_location(self):
    #     global center_ball, keeper_action, fall_action, status_kiper
    #     # if keeper_action == False or detect_status == 'NOTFOUND' or self.distance <= self.MOVE_THRESHOLD:
    #     #         self.send_keeper_status("X")
    #     if keeper_action and detect_status == 'FOUND': #and self.distance > self.MOVE_THRESHOLD
    #         if detect_status == 'FOUND' and self.distance > self.movetreeshold and (cv2.pointPolygonTest(batas_keeper_kiri[0], center_ball, False) >= 0):
    #             status_kiper = "A"
    #             self.send_keeper_status("A")
    #         elif detect_status == 'FOUND'and self.distance > self.movetreeshold and (cv2.pointPolygonTest(batas_keeper_tengah[0], center_ball, False) >= 0):
    #             self.send_keeper_status("B")
    #             status_kiper = "B"
    #         elif detect_status == 'FOUND'and self.distance > self.movetreeshold and (cv2.pointPolygonTest(batas_keeper_kanan[0], center_ball, False) >= 0) :
    #             self.send_keeper_status("C")
    #             status_kiper = "C"
    #         # elif fall_action and (cv2.pointPolygonTest(batas_keeper_tengah[0], center_ball, False) >= 0):
    #         #     status_kiper = "B"
    #         #     self.send_keeper_status("B")
    #         # elif fall_action and (cv2.pointPolygonTest(batas_keeper_kiri[0], center_ball, False) >= 0):
    #         #     status_kiper = "A"
    #         #     self.send_keeper_status("A")
    #         # elif fall_action and (cv2.pointPolygonTest(batas_keeper_kanan[0], center_ball, False) >= 0):
    #         #     status_kiper = "C"
    #         #     self.send_keeper_status("C")
    #         else:
    #             print("aksi jatuh:",fall_action)
    #             self.send_keeper_status("X")
    #             status_kiper = "X"
    #     else:
    #         self.send_keeper_status("X")
    #         status_kiper = "X"
# ---------------------PUBLISHER--------------------------------------------------

def send_keeper_determine(data):
    determinekeeper = Bool()
    determinekeeper.data = data
    keeper_determine_publisher.publish(determinekeeper)

def send_keeper_line(data):
    keeperline = Bool()
    keeperline.data = data
    keeper_line_publisher.publish(keeperline)

# ---------------------PUBLISHER--------------------------------------------------

# ---------------------SUBSCRIBER--------------------------------------------------

def vision_status(data):
    global status
    if data.data ==  "LAWAN_MAGENTA":
        status = "LAWAN_MAGENTA"
    elif data.data == "KEEPER":
        status = "KEEPER"        
    elif data.data == "TRACKBALL":
        status = "TRACKBALL"
    else:
        status = "TRACKBALL"

def get_desicion(jarak_bola):
    desicion_msg = Desicion()
    if jarak_bola > 1: #satuan meter
        desicion_msg = "HADANG"
    else:
        desicion_msg = "TAHAN"
    desicion_status_publisher.publish(desicion_msg)
    return desicion_msg

def send_action_status(data):
    if data == 0:
        status = "BERSIAP"
    elif data == 1:
        status = "HADANG"
    desicion_status_publisher.publish(status)
    



# ---------------------SUBSCRIBER--------------------------------------------------

# --------------ITERASI 1x------------------------------------
subprocess.call(['sh', '/home/robotis/catkin_ws/src/DEWO-OP3/KRI_2023/ImageProcess/object_detect/src/camera_setting.sh'])
net = cv2.dnn.readNet("/home/robotis/catkin_ws/src/DEWO-OP3/KRI2024/Imageprocessing/v1_detection/cfg/bola.cfg","/home/robotis/catkin_ws/database/weights/bola_nasional.weights")
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

ballstate = rospy.Publisher("/DEWO/image_processing/deteksi_bola/ball_state", BallState, queue_size=10)
ballcoor = rospy.Publisher("/DEWO/image_processing/deteksi_bola/coordinate", BallCoordinate, queue_size=10)
ballarea = rospy.Publisher("/DEWO/image_processing/deteksi_bola/ball_area", Ballarea, queue_size=10)
opponent_coordinate = rospy.Publisher("/DEWO/image_processing/deteksi_bola/opponent_coordinate", OpponentCoordinate, queue_size=10)
opponent_state = rospy.Publisher("/DEWO/image_processing/deteksi_bola/opponent_state", OpponentState, queue_size=10)
keeper_status_publisher = rospy.Publisher("/DEWO/image_processing/deteksi_bola/keeper_action", String, queue_size=10)
keeper_determine_publisher = rospy.Publisher("/DEWO/image_processing/deteksi_bola/keeper_determine", Bool, queue_size=10)
desicion_status_publisher = rospy.Publisher("/DEWO/image_processing/deteksi_bola/Desicion_Status", Desicion, queue_size=1)
keeper_line_publisher = rospy.Publisher("/DEWO/image_processing/deteksi_bola/keeper_line", Bool, queue_size=10)

def Init():
    rospy.init_node("Gandamana_DarknetVision")
    rospy.Subscriber("/DEWO/image_processing/deteksi_bola/vision_status", String, vision_status)
    time.sleep(1)

classes = []
with open("/home/robotis/catkin_ws/src/DEWO-OP3/KRI2024/Imageprocessing/v1_detection/class/bola_fisheye.txt", "r") as f:
    classes = f.read().splitlines()

capture = WebcamVideoStream(src=0).start() 
deteksi = ball_detection()
opponent_detection_system = OpponentDetection()
line_detect = goal_line_transform()

def hough_line_task():
    line_detect.hough_line()

def ball_detect_task():
    deteksi.ball_detect()

if __name__ == '__main__':
    try:
        Init()
        while not rospy.is_shutdown():
            print("STATUS SAAT INI : ", status)
            if status == 'TRACKBALL':
                print(ball_area)
                deteksi.ball_detect()
                # print(status)
            elif status == 'LAWAN_MAGENTA':
                opponent_detection_system.run()
                # print(status)
            elif status == 'KEEPER':
                rospy.Rate(10)
                line_detect.hough_line()
                deteksi.ball_detect()
                
        capture.stop()

    except rospy.ROSInterruptException:
        pass