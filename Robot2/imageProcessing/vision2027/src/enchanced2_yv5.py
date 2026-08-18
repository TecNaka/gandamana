#!/home/robotis/yolov5-env/bin/python3.8
# -*- coding: utf-8 -*-
from __future__ import annotations
import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'

if ros_path in sys.path:
    sys.path.remove(ros_path)
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

import cv2
import time
import rospy
from threading import Thread
import configparser
import numpy as np
import subprocess
import os
import torch
import warnings
from v1_detection.msg import BallState, Ballarea, BallCoordinate

# ----------- Konfigurasi -----------
fisheye = 1
framesize = [320, 240]
blobsize = 416

WEIGHTS_PATH = os.path.expanduser('/home/robotis/catkin_ws/src/DEWO-OP3/KRI2025/ImageProcessing/v2_detection/src/runs/train/bola_yolov5n19/weights/best.pt')
device = 'cuda' if torch.cuda.is_available() else 'cpu'
model = torch.hub.load('ultralytics/yolov5', 'custom', path=WEIGHTS_PATH, trust_repo=True)
model.to(device)
model.conf = 0.35  # 0.5
model.iou = 0.3
model.classes = [0]

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

class BallDetector:
    def __init__(self):
        self.detect_status = 'NOTFOUND'
        self.ball_area = 0
        self.fps = 0
        self.framecounter = 0
        self.startfpstime = time.time()
        self.waktu_sebelum = time.time()
        self.scan_area = [0, framesize[0]]
        self.center_ball = (0, 0)
        self.blobsize = 416
        
        self.small_ball_mode = False
        self.small_ball_counter = 0
        self.small_ball_threshold = 1300
        
        self.ballstate_pub = rospy.Publisher("/DEWO/image_processing/deteksi_bola/ball_state", BallState, queue_size=10)
        self.ballcoor_pub = rospy.Publisher("/DEWO/image_processing/deteksi_bola/coordinate", BallCoordinate, queue_size=10)
        self.ballarea_pub = rospy.Publisher("/DEWO/image_processing/deteksi_bola/ball_area", Ballarea, queue_size=10)
    
    def calculate_fps(self):
        current_time = time.time()
        elapsed_time = current_time - self.startfpstime
        if elapsed_time >= 0.2:
            self.fps = self.framecounter / elapsed_time
            self.framecounter = 0
            self.startfpstime = current_time
    
    def send_ball_position(self, x_pos, y_pos, size_obj):
        ballposition = BallCoordinate()
        fix_x = float(x_pos / framesize[0] * 2 - 1)
        fix_y = float(y_pos / framesize[1] * 2 - 1)
        ballposition.pos_x = fix_x
        ballposition.pos_y = fix_y
        ballposition.obj_size = size_obj
        self.ballcoor_pub.publish(ballposition)
    
    def send_ball_area(self, areaball):
        area = Ballarea()
        area.ballarea = areaball
        self.ballarea_pub.publish(area)
    
    def send_ball_status(self, msg):
        ballstatus = BallState()
        ballstatus.ball_status = msg
        self.ballstate_pub.publish(ballstatus)
    
    def detect_ball(self, img):
        self.framecounter += 1
        img_result = img.copy()
        
        if self.small_ball_mode:
            try:
                results = model(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), size=self.blobsize)
                dets = results.xyxy[0]
                
                if len(dets) > 0:
                    for det in dets:
                        x1, y1, x2, y2, conf, cls = det
                        if conf < model.conf or cls != 0:
                            continue
                        
                        x, y, w, h = int(x1), int(y1), int(x2-x1), int(y2-y1)
                        area = w * h
                        
                        if area < self.small_ball_threshold:
                            x_center = x + w/2
                            y_center = y + h/2
                            
                            cv2.rectangle(img_result, (x, y), (x+w, y+h), (255, 100, 0), 1)
                            cv2.putText(img_result, f"Small: {conf:.2f}", (x, y-5), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 100, 0), 1)
                            
                            self.center_ball = (x_center, y_center)
                            self.ball_area = area
                            self.send_ball_position(x_center, y_center, area)
                            self.send_ball_area(area)
                            self.send_ball_status("FOUND")
                            
                            self.small_ball_counter = 0
                            self.waktu_sebelum = time.time()
                            
                            if area < 800:
                                self.blobsize = 320
                            else:
                                self.blobsize = 416
                            
                            return img_result, True
            
            except Exception as e:
                print(f"[ERROR] YOLO detection failed: {e}")
            
            self.small_ball_counter += 1
            if self.small_ball_counter > 10:
                self.small_ball_mode = False
                print("[INFO] Switching back to normal mode")
            
            return img_result, False
        
        if self.detect_status == 'NOTFOUND':
            try:
                results = model(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), size=self.blobsize)
                dets = results.xyxy[0]
                
                if len(dets) > 0:
                    for det in dets:
                        x1, y1, x2, y2, conf, cls = det
                        if conf < model.conf or cls != 0:
                            continue
                        
                        x, y, w, h = int(x1), int(y1), int(x2-x1), int(y2-y1)
                        area = w * h
                        
                        if area < self.small_ball_threshold:
                            print(f"[INFO] Ball too small ({area}px), using small ball mode")
                            self.small_ball_mode = True
                            self.small_ball_counter = 0
                            
                            x_center = x + w/2
                            y_center = y + h/2
                            cv2.rectangle(img_result, (x, y), (x+w, y+h), (255, 100, 0), 1)
                            
                            self.center_ball = (x_center, y_center)
                            self.ball_area = area
                            self.send_ball_position(x_center, y_center, area)
                            self.send_ball_area(area)
                            self.send_ball_status("FOUND")
                            
                            return img_result, True
                        
                        self.ball_area = area
                        self.center_ball = (x + w/2, y + h/2)
                        
                        roi = img[y:y+h, x:x+w]
                        if roi.size > 0:
                            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                            
                            h_vals = hsv_roi[:,:,0].flatten()
                            s_vals = hsv_roi[:,:,1].flatten()
                            v_vals = hsv_roi[:,:,2].flatten()
                            
                            orange_mask = (h_vals <= 30)
                            if np.any(orange_mask):
                                h_vals = h_vals[orange_mask]
                                s_vals = s_vals[orange_mask]
                                v_vals = v_vals[orange_mask]
                            
                            h_margin = 5 if area < 3000 else 3
                            s_margin = 40 if area < 3000 else 20
                            v_margin = 40 if area < 3000 else 20
                            
                            min_h = max(0, int(np.min(h_vals)) - h_margin)
                            max_h = min(179, int(np.max(h_vals)) + h_margin)
                            min_s = max(0, int(np.min(s_vals)) - s_margin)
                            max_s = min(255, int(np.max(s_vals)) + s_margin)
                            min_v = max(0, int(np.min(v_vals)) - v_margin)
                            max_v = min(255, int(np.max(v_vals)) + v_margin)
                            
                            max_h = min(max_h, 33)
                            min_s = max(min_s, 100 if area < 3000 else 140)
                            
                            self.hsv_lower = np.array([min_h, min_s, min_v])
                            self.hsv_upper = np.array([max_h, max_s, max_v])
                            
                            print(f"[HSV] Init for area {area}: {self.hsv_lower} - {self.hsv_upper}")
                        
                        self.detect_status = 'FOUND'
                        self.waktu_sebelum = time.time()
                        
                        scan_margin = w * 4 if area < 5000 else w + 35
                        x_center = x + w/2
                        self.scan_area = [
                            max(0, int(x_center - scan_margin)),
                            min(framesize[0], int(x_center + scan_margin))
                        ]
                        
                        cv2.rectangle(img_result, (x, y), (x+w, y+h), (255, 0, 0), 1)
                        cv2.putText(img_result, f"YOLO: {conf:.2f}", (x, y-5), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                        
                        self.send_ball_position(x + w/2, y + h/2, area)
                        self.send_ball_area(area)
                        self.send_ball_status("FOUND")
                        
                        if area < 2800:
                            self.blobsize = 320
                        else:
                            self.blobsize = 224
                        
                        return img_result, True
            
            except Exception as e:
                print(f"[ERROR] YOLO failed: {e}")
            
            self.send_ball_status("NOTFOUND")
            return img_result, False
        
        else:
            config_lapangan = configparser.ConfigParser()
            config_lapangan.read('/home/robotis/catkin_ws/src/DEWO-OP3/KRI2024/Imageprocessing/v1_detection/cfg/hsv_lapangan.ini')
            
            lower_h = int(config_lapangan['HSV_LAPANGAN']['lower_h'])
            upper_h = int(config_lapangan['HSV_LAPANGAN']['upper_h'])
            lower_s = int(config_lapangan['HSV_LAPANGAN']['lower_s'])
            upper_s = int(config_lapangan['HSV_LAPANGAN']['upper_s'])
            lower_v = int(config_lapangan['HSV_LAPANGAN']['lower_v'])
            upper_v = int(config_lapangan['HSV_LAPANGAN']['upper_v'])
            
            lower_field = np.array([lower_h, lower_s, lower_v])
            upper_field = np.array([upper_h, upper_s, upper_v])
            
            hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            field_mask = cv2.inRange(hsv_img, lower_field, upper_field)
            
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
            field_mask = cv2.morphologyEx(field_mask, cv2.MORPH_CLOSE, kernel)
            field_mask = cv2.morphologyEx(field_mask, cv2.MORPH_OPEN, kernel)
            field_img = cv2.bitwise_and(img, img, mask=field_mask)
            hsv_field = cv2.cvtColor(field_img, cv2.COLOR_BGR2HSV)
            ball_mask = cv2.inRange(hsv_field, self.hsv_lower, self.hsv_upper)
            ball_mask = cv2.morphologyEx(ball_mask, cv2.MORPH_CLOSE, kernel)
            ball_mask = cv2.morphologyEx(ball_mask, cv2.MORPH_OPEN, kernel)
            contours, _ = cv2.findContours(ball_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            found = False
            for contour in contours:
                area = cv2.contourArea(contour)
                x, y, w, h = cv2.boundingRect(contour)
                x_center = x + w/2
                
                area_min = max(50, self.ball_area // 6) if self.ball_area < 3000 else max(100, self.ball_area // 4)
                area_max = self.ball_area * 1.8 if self.ball_area < 3000 else self.ball_area * 1.3
                
                if area_min < area < area_max:
                    scan_min = self.scan_area[0] - w
                    scan_max = self.scan_area[1] + w
                    
                    if scan_min <= x_center <= scan_max:
                        cv2.rectangle(img_result, (x, y), (x+w, y+h), (0, 255, 255), 2)
                        
                        self.center_ball = (x_center, y + h/2)
                        current_area = w * h
                        
                        self.send_ball_position(x_center, y + h/2, current_area)
                        self.send_ball_area(current_area)
                        self.send_ball_status("FOUND")
                        found = True
                        waktu_sesudah = time.time()
                        delta = waktu_sesudah - self.waktu_sebelum
                        
                        timeout = max(0.5, min(40, current_area / 1000))
                        if delta >= timeout:
                            self.detect_status = 'NOTFOUND'
                            print(f"[INFO] HSV timeout after {delta:.1f}s")
                        
                        break
            
            if not found:
                self.detect_status = 'NOTFOUND'
                self.send_ball_status("NOTFOUND")
                print("[INFO] Lost ball in HSV tracking")
            
            return img_result, found
    
    def draw_overlay(self, img):
        self.calculate_fps()
        fps_text = f"FPS: {self.fps:.1f}"
        cv2.putText(img, fps_text, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        status_text = f"Status: {self.detect_status}"
        status_color = (0, 255, 0) if self.detect_status == 'FOUND' else (0, 0, 255)
        cv2.putText(img, status_text, (5, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 2)
        
        mode_text = "Mode: "
        if self.small_ball_mode:
            mode_text += "YOLO (Small Ball)"
            mode_color = (255, 100, 0)
        elif self.detect_status == 'FOUND':
            mode_text += "HSV Tracking"
            mode_color = (0, 255, 255)
        else:
            mode_text += "YOLO Search"
            mode_color = (255, 0, 0)
        
        cv2.putText(img, mode_text, (5, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, mode_color, 2)
        
        area_text = f"Area: {self.ball_area}"
        cv2.putText(img, area_text, (5, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        
        cv2.putText(img, f"Blob: {self.blobsize}", (280, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        if self.detect_status == 'FOUND' and not self.small_ball_mode:
            cv2.line(img, (self.scan_area[0], 0), (self.scan_area[0], framesize[1]), (0, 255, 255), 1)
            cv2.line(img, (self.scan_area[1], 0), (self.scan_area[1], framesize[1]), (0, 255, 255), 1)
        return img

if __name__ == '__main__':
    subprocess.call(['sh', '/home/robotis/catkin_ws/src/DEWO-OP3/KRI2025/ImageProcessing/v2_detection/src/camera_setting_yv5n.sh'])
    
    detector = BallDetector()
    capture = WebcamVideoStream(src=0).start()
    
    try:
        rospy.init_node("Gandamana_Ball_Detector")
        
        print("GANDAMANA VISION ON")
        
        while not rospy.is_shutdown():
            frame = capture.read()
            if frame is None:
                continue
            
            processed_frame, detected = detector.detect_ball(frame)
            processed_frame = detector.draw_overlay(processed_frame)
            
            cv2.imshow("GANDAMANA VISION", processed_frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                detector.detect_status = 'NOTFOUND'
                detector.small_ball_mode = False
                print("[INFO] Reset detection")
        
        capture.stop()
        cv2.destroyAllWindows()
        
    except rospy.ROSInterruptException:
        pass