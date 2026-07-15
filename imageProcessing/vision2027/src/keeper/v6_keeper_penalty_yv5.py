#!/home/robotis/yolov5-env/bin/python3.8
# -*- coding: utf-8 -*-
from __future__ import annotations
import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'

if ros_path in sys.path:
    sys.path.remove(ros_path)
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

# -------------- PyTorch --------------
_YOLO_AVAILABLE = True
try:
    import torch
except Exception as e:
    _YOLO_AVAILABLE = False
    print("[WARN] PyTorch tidak tersedia:", e)

import cv2
import time
import math
import rospy
from threading import Thread
import configparser
import numpy as np
import subprocess
import os
from std_msgs.msg import String, Int16
from v1_detection.msg import BallState, Ballarea, BallCoordinate, OpponentState, OpponentCoordinate, GoalState, GoalCoordinate
import warnings

# --------- Global defaults / inisialisasi variabel ---------
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
# jika menggunakan fisheye maka gunakan 0.34
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

x_goal_0 = y_goal_0 = w_goal_0 = h_goal_0 = 0
x_goal_1 = y_goal_1 = w_goal_1 = h_goal_1 = 0

# ========= VAR ============
static_bbox = None
last_ball_position = None
stationary_start_time = None
last_detection_time = None
batas_bola_diam = 2.0
timeout_bbox = 2.0
toleransi_movement = 20
current_direction = ""

WEIGHTS_PATH = os.path.expanduser('/home/robotis/catkin_ws/src/DEWO-OP3/KRI2025/ImageProcessing/v2_detection/src/runs/train/bola_yolov5n19/weights/best.pt')

device = 'cuda' if torch.cuda.is_available() else 'cpu'
model = torch.hub.load('ultralytics/yolov5', 'custom', path=WEIGHTS_PATH, trust_repo=True)
model.to(device)
model.conf = 0.5   # confidence threshold
model.iou = 0.45   # IOU threshold
model.classes = [0]
classes = model.names

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

subprocess.call(['sh', '/home/robotis/catkin_ws/src/DEWO-OP3/KRI2025/ImageProcessing/v2_detection/src/camera_setting_yv5n.sh'])

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
        x_obj, y_obj = center_object
        x1_line, y1_line = pt1
        x2_line, y2_line = pt2
        
        if x2_line - x1_line == 0:
            if x_obj < x1_line:
                keeper_action=False
            else:
                keeper_action=True
            return
        
        m = (y2_line - y1_line) / (x2_line - x1_line)
        c = y1_line - m * x1_line
        
        y_line = m * x_obj + c
        
        if y_obj < y_line:
            keeper_action=False
        else:
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
        
        x = x_ball
        y = y_ball
        w = w_ball
        h = h_ball

        if x == None or y == None or w == None or h == None:
            return
        
        x1 = x
        y1 = y
        x2 = (x + w)
        y2 = (y + h)

        ball_area = w * h
        # print('LUAS BOLA :' ,ball_area)
        
        dot_tengah = ((x1 + x2) //2, (y1 + y2)//2)
        dot_sepertiga_kanan = ((x1 + dot_tengah[0]) // 2 ,(y1 + dot_tengah[1]) // 2)
        dot_duapertiga_kanan = ((x2 + dot_tengah[0]) // 2 ,(y2 + dot_tengah[1]) // 2)
        dot_sepertiga_kiri = ((dot_tengah[0] + (x1 + (x2 - x1))) // 2 ,(dot_tengah[1] + y1) // 2)
        dot_duapertiga_kiri = ((x1 + dot_tengah[0]) // 2, (y1 + (y2 - y1) + dot_tengah[1]) // 2)
        dot_atas_tengah = (dot_tengah[0], (dot_tengah[1] + y1 + h//5)//2)
        dot_bawah_tengah = (dot_tengah[0], (dot_tengah[1] + y2)//2)

        points = [
            (dot_tengah, "TENGAH"),
            (dot_sepertiga_kanan, "KANAN-ATAS"),
            (dot_duapertiga_kanan, "KANAN-BAWAH"),
            (dot_sepertiga_kiri, "KIRI-ATAS"),
            (dot_duapertiga_kiri, "KIRI-BAWAH"),
            (dot_atas_tengah, "ATAS"),
            (dot_bawah_tengah, "BAWAH")
        ]

        hsv_vals_h = []
        hsv_vals_s = []
        hsv_vals_v = []

        for point, label in points:
            sx, sy = point
            sx = max(0, min(sx, framesize[0]-1))
            sy = max(0, min(sy, framesize[1]-1))
            
            colorsB = img[sy, sx, 0]
            colorsG = img[sy, sx, 1]
            colorsR = img[sy, sx, 2]
            
            hsv_value = np.uint8([[[colorsB, colorsG, colorsR]]])
            hsv = cv2.cvtColor(hsv_value, cv2.COLOR_BGR2HSV)[0,0]
            
            hsv_vals_h.append(hsv[0])
            hsv_vals_s.append(hsv[1])
            hsv_vals_v.append(hsv[2])

        min_h = np.min(hsv_vals_h)
        min_s = np.min(hsv_vals_s)
        min_v = np.min(hsv_vals_v)
        max_h = np.max(hsv_vals_h)
        max_s = np.max(hsv_vals_s)
        max_v = np.max(hsv_vals_v)

        nilai_maksimum_h = 33
        if max_h >= nilai_maksimum_h:
            max_h = np.uint8(nilai_maksimum_h)

        nilai_minimum_s = 160
        if min_s <= nilai_minimum_s:
            min_s = np.uint8(nilai_minimum_s)

        # print('min hue',min_h,'min sat', min_s,'min val', min_v, 'max hue',max_h,'max sat', max_s,'max val', max_v)

    def update_keeper_direction(self, x_center, y_center, ball_area):
        global static_bbox, last_ball_position, stationary_start_time, last_detection_time, current_direction
        
        current_time = time.time()
        direction = ""
        
        current_position = (x_center, y_center)
        
        if last_ball_position is not None:
            dist = math.hypot(current_position[0] - last_ball_position[0], 
                             current_position[1] - last_ball_position[1])
            
            if dist < toleransi_movement:
                if stationary_start_time is None:
                    stationary_start_time = current_time
                elif current_time - stationary_start_time > batas_bola_diam:
                    if static_bbox is None:
                        if ball_area <= (900*fisheye):
                            margin_w = int(w_ball * 2.5)
                            margin_h = int(h_ball * 0.15)
                        else:
                            margin_w = int(w_ball * 2.0)
                            margin_h = int(h_ball * 0.12)

                        x_s = max(0, int(x_ball - margin_w))
                        y_s = max(0, int(y_ball - margin_h))
                        w_s = min(framesize[0], int(w_ball + 2 * margin_w))
                        h_s = min(framesize[1], int(h_ball + 2 * margin_h))
                        static_bbox = (x_s, y_s, w_s, h_s)
                        print("Static bbox ON")
            else:
                stationary_start_time = None
                static_bbox = None
                print("!!!!!!!! Bola Gerak, RESET !!!!!!!!!!!")
        else:
            stationary_start_time = current_time

        last_ball_position = current_position
        last_detection_time = current_time

        if static_bbox is not None:
            x_s, y_s, w_s, h_s = static_bbox
            
            if (x_center < x_s or x_center > x_s + w_s or 
                y_center < y_s or y_center > y_s + h_s):
                
                rel_x = x_center - x_s
                if rel_x < w_s * 0.4:
                    direction = "kiri"
                elif rel_x > 2 * w_s * 0.3:
                    direction = "kanan"
                else:
                    direction = "tengah"

                if direction != current_direction:
                    # current_direction = direction
                    keeper_direction_pub.publish(direction)
                    print(f"Direction: {direction}")
                    time.sleep(0.2)
                    direction = "X"
                    static_bbox = None

        return static_bbox, direction

    def check_timeout_and_reset(self):
        global static_bbox, last_ball_position, stationary_start_time, last_detection_time #current_direction
        
        current_time = time.time()
        
        if last_detection_time is not None and (current_time - last_detection_time) > timeout_bbox:
            static_bbox = None
            last_ball_position = None
            stationary_start_time = None
            #current_direction = ""
            print("!!!! Timeout, BBOX RESET !!!!")

    def ball_detect(self):
        global x_ball, y_ball, w_ball, h_ball, detect_status, min_h, min_s, min_v, max_h, max_s, max_v, ball_area, waktu_sebelum, scan_area, framesize, status, pt1, pt2, center_ball,  x_center_ball, y_center_ball, status, fps, blobsize
        global x_center_goal_0, y_center_goal_0, x_center_goal_1, y_center_goal_1, x_goal_0, y_goal_0, w_goal_0, h_goal_0, x_goal_1, y_goal_1, w_goal_1, h_goal_1
        global static_bbox, last_ball_position, stationary_start_time, last_detection_time # current_direction
        
        img = capture.read()
        self.framecounter += 1
        img_result = img.copy()

        self.check_timeout_and_reset()

        ball_detected_in_frame = False
        current_x_center = 0
        current_y_center = 0
        current_ball_area = 0

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

            frame_copy = field_img.copy()

            # x_center_frame = (int) (framesize[0]/2)
            # y_center_frame = (int) (framesize[1]/2)

            hsv = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2HSV)

            binary_ball = cv2.inRange(hsv, hsv_lower, hsv_upper)
            kernel_bball = np.ones((5,5), np.uint8)
            binary_ball = cv2.morphologyEx(binary_ball, cv2.MORPH_CLOSE, kernel_bball)
            binary_ball = cv2.morphologyEx(binary_ball, cv2.MORPH_OPEN, kernel_bball)
            frame_copy = cv2.bitwise_and(frame_copy, frame_copy, mask=binary_ball)
            contours, hierarchy = cv2.findContours(binary_ball, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # result = img.copy()
            contourLength = len(contours)
            objek_ditemukan = False

            for contour in contours:
                area = cv2.contourArea(contour)
                if area > ball_area//5 and area < ball_area*1.1 and contourLength > 0:
                    x, y, w, h = cv2.boundingRect(contour)
                    x_center = x + w/2
                    if scan_area[0] <= x_center <= scan_area[1] and (w*h) > (3000 * fisheye):
                        cv2.rectangle(img_result, (x, y), (x + w, y + h), (0, 255, 255), 2) 
                        cv2.putText(img_result, "HSV", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                        
                        area_bola = (w*h)
                        x_center_rect = (x+w/2)
                        y_center_rect = (y+h/2)
                        center_ball = (x_center_rect, y_center_rect)
                        
                        ball_detected_in_frame = True
                        current_x_center = x_center_rect
                        current_y_center = y_center_rect
                        current_ball_area = area_bola
                        
                        self.send_ball_position((float)(x_center_rect), (float)(y_center_rect), framesize[0], framesize[1], area_bola)
                        self.send_ball_area(area_bola)
                        self.send_ball_status("FOUND")
                        objek_ditemukan = True
                        waktu_detect = self.map_value(area_bola, 0, 76800, 0.5, 80)
                        waktu_sesudah = time.time()
                        delta = waktu_sesudah - waktu_sebelum
                        
                        # print('Ball Area Result :', area_bola)
                        if delta >= waktu_detect:
                            waktu_sebelum = time.time()
                            detect_status = 'NOTFOUND'
                        break

            if not objek_ditemukan:
                detect_status = 'NOTFOUND'

        if detect_status == 'NOTFOUND':
            # height, width, channels = img.shape
            try:
                results = model(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), size=blobsize)
                dets = results.xyxy[0]
                if hasattr(dets, 'cpu'):
                    dets = dets.cpu().numpy()
                else:
                    dets = dets.numpy()
            except Exception as e:
                print('[ERROR] Inference YOLOv5 gagal:', e)
                dets = np.array([])

            # jika_ditemukan = False
            daftar = []

            if dets.size == 0:
                # print('not found all (yolov5)')
                self.send_ball_status("NOTFOUND")
                blobsize = 416
                # print("Did not detect ball, setting blobsize to ",blobsize)
            else:
                for det in dets:
                    x1, y1, x2, y2, conf, cls = det
                    conf = float(conf)
                    cls = int(cls)
                    if conf < model.conf:
                        continue
                    label = classes.get(cls, str(cls)) if isinstance(classes, dict) else classes[cls]
                    daftar.append(label)

                    x = int(x1)
                    y = int(y1)
                    w = int(x2 - x1)
                    h = int(y2 - y1)

                    if cls == 0:
                        x_ball, y_ball, w_ball, h_ball = x, y, w, h
                        x_center_ball = (x_ball + w_ball/2)
                        y_center_ball = (y_ball + h_ball/2)
                        center_ball = (x_center_ball, y_center_ball)
                        obj_size_ball = w_ball * h_ball

                        ball_detected_in_frame = True
                        current_x_center = x_center_ball
                        current_y_center = y_center_ball
                        current_ball_area = obj_size_ball

                        x1_bb, y1_bb, x2_bb, y2_bb = x_ball, y_ball, x_ball + w_ball, y_ball + h_ball
                        
                        dot_tengah = ((x1_bb + x2_bb) // 2, (y1_bb + y2_bb) // 2)
                        dot_sepertiga_kanan = ((x1_bb + dot_tengah[0]) // 2, (y1_bb + dot_tengah[1]) // 2)
                        dot_duapertiga_kanan = ((x2_bb + dot_tengah[0]) // 2, (y2_bb + dot_tengah[1]) // 2)
                        dot_sepertiga_kiri = ((dot_tengah[0] + (x1_bb + (x2_bb - x1_bb))) // 2, (dot_tengah[1] + y1_bb) // 2)
                        dot_duapertiga_kiri = ((x1_bb + dot_tengah[0]) // 2, (y1_bb + (y2_bb - y1_bb) + dot_tengah[1]) // 2)
                        dot_atas_tengah = (dot_tengah[0], (dot_tengah[1] + y1_bb + h_ball//5) // 2)
                        dot_bawah_tengah = (dot_tengah[0], (dot_tengah[1] + y2_bb) // 2)

                        points = [
                            (dot_tengah, (0, 255, 0), "TENGAH"),
                            (dot_sepertiga_kanan, (255, 0, 0), "KANAN-ATAS"),
                            (dot_duapertiga_kanan, (0, 0, 255), "KANAN-BAWAH"),
                            (dot_sepertiga_kiri, (255, 255, 0), "KIRI-ATAS"),
                            (dot_duapertiga_kiri, (255, 0, 255), "KIRI-BAWAH"),
                            (dot_atas_tengah, (0, 255, 255), "ATAS"),
                            (dot_bawah_tengah, (255, 255, 255), "BAWAH")
                        ]
                        
                        for point, color, label_text in points:
                            cv2.circle(img_result, point, 3, color, -1)
                            cv2.circle(img_result, point, 6, color, 1)
                            cv2.putText(img_result, label_text, (point[0] + 8, point[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.3, color, 1)

                        if ball_area <= (5000*fisheye) :
                            in_area_ball = w_ball * 3
                        elif ball_area > (5000*fisheye) :
                            in_area_ball = w_ball + (35*fisheye)
                        scan_area = [x_center_ball-in_area_ball, x_center_ball+in_area_ball]

                        cv2.rectangle(img_result, (x, y), (x + w, y + h), (255,0,0), 1)
                        text = "{}: {:.2f}".format(label, conf)
                        cv2.putText(img_result, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)

                        self.send_ball_position((float)(x_center_ball), (float)(y_center_ball), framesize[0], framesize[1], obj_size_ball)
                        self.send_ball_area(obj_size_ball)
                        self.send_ball_status("FOUND")
                        # print('bola [{}%]'.format(np.round(conf * 100)))

                        self.get_hsv_val(img)
                        detect_status = 'FOUND'

                        if obj_size_ball <= (2800*fisheye):
                            blobsize = 320
                        elif obj_size_ball > (2800*fisheye):
                            blobsize = 224
                        break

                if detect_status != 'FOUND':
                    self.send_ball_position((float)(0), (float)(0), framesize[0], framesize[1], 0)
                    self.send_ball_status("NOTFOUND")
                    print('bola not found (after yolov5)')
            # print(daftar)

        if ball_detected_in_frame:
            static_bbox, direction = self.update_keeper_direction(current_x_center, current_y_center, current_ball_area)

        if static_bbox is not None:
            x_s, y_s, w_s, h_s = static_bbox
            cv2.rectangle(img_result, (x_s, y_s), (x_s + w_s, y_s + h_s), (0, 255, 0), 2)
            cv2.line(img_result, (int(x_s + w_s * 0.4), y_s), (int(x_s + w_s * 0.4), y_s + h_s), (0,0,255), 1)
            cv2.line(img_result, (int(x_s + 2*w_s * 0.3), y_s), (int(x_s + 2*w_s * 0.3), y_s + h_s), (0,0,255), 1)
            cv2.putText(img_result, "Dir", (x_s, y_s-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
            
            # if current_direction:
            #     cv2.putText(img_result, f"DIR: {current_direction}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)

        # print(f"Detect Status: {detect_status}, Static BBox: {static_bbox is not None}, Direction: {current_direction}")
        self.calculate_fps()
        fps_str = "FPS: {:.2f}".format(fps)
        print(ball_area)
        if pt1 and pt2 is not None:
            cv2.line(img_result, pt1, pt2, (0, 0, 255), 3, cv2.LINE_AA)
            self.determine_position(center_ball, pt1, pt2)
        if x_goal_0 and x_goal_1 is not None:
            cv2.rectangle(img_result, (x_goal_0, y_goal_0), (x_goal_0 + w_goal_0, y_goal_0 + h_goal_0), (255, 0, 255), 2)
            cv2.rectangle(img_result, (x_goal_1, y_goal_1), (x_goal_1 + w_goal_1, y_goal_1 + h_goal_1), (255, 0, 255), 2)
        cv2.putText(img_result, fps_str, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(img_result, str(blobsize), (280, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.imshow("GANDAMANA_VISION_KEEPER", img_result)
        cv2.waitKey(1)

ballstate = rospy.Publisher("/DEWO/image_processing/deteksi_bola/ball_state", BallState, queue_size=10)
ballcoor = rospy.Publisher("/DEWO/image_processing/deteksi_bola/coordinate", BallCoordinate, queue_size=10)
ballarea = rospy.Publisher("/DEWO/image_processing/deteksi_bola/ball_area", Ballarea, queue_size=10)
opponent_coordinate = rospy.Publisher("/DEWO/image_processing/deteksi_bola/opponent_coordinate", OpponentCoordinate, queue_size=10)
opponent_state = rospy.Publisher("/DEWO/image_processing/deteksi_bola/opponent_state", OpponentState, queue_size=10)
keeper_status_publisher = rospy.Publisher("/DEWO/image_processing/deteksi_bola/keeper_action", String, queue_size=10)
keeper_direction_pub = rospy.Publisher("/DEWO/image_processing/deteksi_bola/ball_direction", String, queue_size=10)

deteksi = ball_detection()
capture = WebcamVideoStream(src=0).start() 

if __name__ == '__main__':
    try:
        rospy.init_node("Gandamana_YoloV5n_Keeper")
        while not rospy.is_shutdown():
            warnings.filterwarnings("ignore", category=FutureWarning)
            deteksi.ball_detect()
        capture.stop()
    except rospy.ROSInterruptException:
        pass