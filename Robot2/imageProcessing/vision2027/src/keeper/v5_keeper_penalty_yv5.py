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
import math
import threading
import subprocess
from threading import Thread
import numpy as np
import cv2
import rospy
from std_msgs.msg import String, Int16
from v2_detection.msg import BallState, Ballarea, BallCoordinate
import warnings

_YOLO_AVAILABLE = True
try:
    import torch
except Exception as e:
    _YOLO_AVAILABLE = False
    print("[WARN] PyTorch tidak tersedia:", e)

WEIGHTS_PATH = os.path.expanduser(
    '/home/robotis/catkin_ws/src/DEWO-OP3/KRI2025/ImageProcessing/v2_detection/src/runs/train/bola_yolov5n19/weights/best_test.pt'
)

FRAME_W = 320
FRAME_H = 240

device = 'cuda' if _YOLO_AVAILABLE and torch.cuda.is_available() else 'cpu'
model = None
if _YOLO_AVAILABLE:
    model = torch.hub.load('ultralytics/yolov5', 'custom', path=WEIGHTS_PATH, force_reload=False)
    model.to(device)
    model.conf = 0.5
    model.iou = 0.45
    model.classes = [0]

static_bbox = None
last_ball_position = None
stationary_start_time = None
last_detection_time = None
batas_bola_diam = 2.0
timeout_bbox = 2.0
toleransi_movement = 20

min_h = 0
min_s = 0
min_v = 0
max_h = 0
max_s = 0
max_v = 0
hsv_ready = False

class WebcamVideoStream:
    def __init__(self, src=0):
        self.stream = cv2.VideoCapture(src, cv2.CAP_V4L2)
        self.stream.set(cv2.CAP_PROP_FPS, 60)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False
    def start(self):
        Thread(target=self.update, args=(), daemon=True).start()
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
        try:
            self.stream.release()
        except Exception:
            pass

pub_state = rospy.Publisher("/DEWO/image_processing/deteksi_bola/ball_state", BallState, queue_size=10)
pub_area = rospy.Publisher("/DEWO/image_processing/deteksi_bola/ball_area", Ballarea, queue_size=10)
pub_coord = rospy.Publisher("/DEWO/image_processing/deteksi_bola/coordinate", BallCoordinate, queue_size=10)
pub_direction = rospy.Publisher("/DEWO/image_processing/deteksi_bola/ball_direction", String, queue_size=10)

def send_ball_position(x_pos, y_pos, w_frame, h_frame, size_obj):
    msg = BallCoordinate()
    fix_x = float(x_pos / w_frame * 2 - 1)
    fix_y = float(y_pos / h_frame * 2 - 1)
    msg.pos_x = fix_x
    msg.pos_y = fix_y
    msg.obj_size = int(size_obj)
    pub_coord.publish(msg)

def send_ball_status(status_str):
    msg = BallState()
    msg.ball_status = status_str
    pub_state.publish(msg)

def compute_hsv_range_from_bbox(img, bbox):
    global FRAME_W, FRAME_H
    x, y, w, h = bbox
    h_img, w_img = img.shape[:2]
    x1 = max(0, min(x, w_img-1))
    y1 = max(0, min(y, h_img-1))
    x2 = max(0, min(x + w, w_img-1))
    y2 = max(0, min(y + h, h_img-1))

    if x2 <= x1 or y2 <= y1:
        return None

    cx = (x1 + x2) // 2
    cy = (y1 + y2) // 2
    samples = []
    samples.append((cx, cy))
    samples.append(((x1 + cx)//2, (y1 + cy)//2))
    samples.append(((x2 + cx)//2, (y2 + cy)//2))
    samples.append(((x1 + cx)//2, (cy + y2)//2))
    samples.append(((x2 + cx)//2, (cy + y1)//2))
    samples.append((cx, (y1 + cy)//2))
    samples.append((cx, (cy + y2)//2))

    hsv_vals = []
    for sx, sy in samples:
        sx = int(max(0, min(sx, w_img-1)))
        sy = int(max(0, min(sy, h_img-1)))
        b,g,r = img[sy, sx]
        hsv = cv2.cvtColor(np.uint8([[[b,g,r]]]), cv2.COLOR_BGR2HSV)[0,0]
        hsv_vals.append(hsv)

    hsv_vals = np.array(hsv_vals)
    min_h = int(np.min(hsv_vals[:,0]))
    min_s = int(np.min(hsv_vals[:,1]))
    min_v = int(np.min(hsv_vals[:,2]))
    max_h = int(np.max(hsv_vals[:,0]))
    max_s = int(np.max(hsv_vals[:,1]))
    max_v = int(np.max(hsv_vals[:,2]))

    if max_h >= 33:
        max_h = 33
    if min_s <= 160:
        min_s = 160

    return (min_h, min_s, min_v, max_h, max_s, max_v)

def field_mask_from_hsv(img, lower_field=(30, 40, 40), upper_field=(90, 255, 255)):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower_field), np.array(upper_field))
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
    mask = cv2.erode(mask, kernel, iterations=2)
    mask = cv2.dilate(mask, kernel, iterations=5)
    return mask

subprocess.call(['sh', '/home/robotis/catkin_ws/src/DEWO-OP3/KRI_2023/ImageProcess/object_detect/src/camera_setting.sh'])
vs = WebcamVideoStream(src=0).start()

def deteksi_ball_loop():
    global static_bbox, last_ball_position, stationary_start_time, last_detection_time
    global min_h, min_s, min_v, max_h, max_s, max_v, hsv_ready

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        warnings.filterwarnings("ignore", category=FutureWarning)
        start_time = time.time()
        img = vs.read()
        if img is None:
            rospy.logwarn_throttle(5, "Frame kosong dari kamera")
            rate.sleep()
            continue

        frame_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        det = np.array([])
        if _YOLO_AVAILABLE and model is not None:
            try:
                results = model(frame_rgb, size=416)
                det = results.xyxy[0].cpu().numpy()
            except Exception as e:
                print('[WARN] YOLO inference gagal:', e)

        current_time = time.time()
        ball_detected = False
        x_ball = y_ball = w_ball = h_ball = 0
        detect_status = 'NOTFOUND'
        ball_area = 0
        x_center_ball = 0
        y_center_ball = 0
        besar = 0

        if det.size and det.shape[0] > 0:
            x1, y1, x2, y2, conf, cls = det[0]
            x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
            w = x2 - x1
            h = y2 - y1
            center_x = x1 + w / 2
            center_y = y1 + h / 2
            besar = w + h

            x_ball, y_ball, w_ball, h_ball = x1, y1, w, h
            ball_detected = True
            detect_status = 'FOUND'
            ball_area = w * h
            x_center_ball = int(center_x)
            y_center_ball = int(center_y)

            last_detection_time = current_time

            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 255), 1)
            cv2.putText(img, f"ball: {conf:.2f}", (x1, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,255,255), 1)
            cv2.putText(img, f"size: {int(besar)}", (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255,0,0), 1)

            if last_ball_position is not None:
                dist = math.hypot(center_x - last_ball_position[0], center_y - last_ball_position[1])
                if dist < toleransi_movement:
                    if stationary_start_time is None:
                        stationary_start_time = current_time
                    elif current_time - stationary_start_time > batas_bola_diam:
                        if static_bbox is None:
                            if besar < 71:
                                margin_w = int(w_ball * 2.5)
                                margin_h = int(h_ball * 0.15)
                            else:
                                margin_w = int(w_ball * 2.0)
                                margin_h = int(h_ball * 0.15)

                            x_s = max(0, int(x_ball - margin_w))
                            y_s = max(0, int(y_ball - margin_h))
                            w_s = min(FRAME_W, int(w_ball + 2 * margin_w))
                            h_s = min(FRAME_H, int(h_ball + 2 * margin_h))
                            static_bbox = (x_s, y_s, w_s, h_s)
                else:
                    stationary_start_time = None
                    static_bbox = None
            else:
                stationary_start_time = current_time

            last_ball_position = (center_x, center_y)

            sample = compute_hsv_range_from_bbox(img, (x_ball, y_ball, w_ball, h_ball))
            if sample is not None:
                min_h, min_s, min_v, max_h, max_s, max_v = sample
                hsv_ready = True

        else:
            detect_status = 'NOTFOUND'
            ball_detected = False
            ball_area = 0
            x_center_ball = 0
            y_center_ball = 0
            last_ball_position = None
            stationary_start_time = None
            if last_detection_time is not None and (current_time - last_detection_time) > timeout_bbox:
                static_bbox = None

            if hsv_ready:
                hsv_lower = np.array([min_h, min_s, min_v])
                hsv_upper = np.array([max_h, max_s, max_v])
                fmask = field_mask_from_hsv(img)
                hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                ball_binary = cv2.inRange(hsv_img, hsv_lower, hsv_upper)
                ball_binary = cv2.bitwise_and(ball_binary, ball_binary, mask=fmask)
                kernel = np.ones((5,5), np.uint8)
                ball_binary = cv2.morphologyEx(ball_binary, cv2.MORPH_CLOSE, kernel)
                ball_binary = cv2.morphologyEx(ball_binary, cv2.MORPH_OPEN, kernel)
                contours, _ = cv2.findContours(ball_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if contours:
                    cnt = max(contours, key=cv2.contourArea)
                    area = cv2.contourArea(cnt)
                    if area > 200:  # tunable min area
                        x,y,w,h = cv2.boundingRect(cnt)
                        cx = x + w/2
                        cy = y + h/2
                        cv2.rectangle(img, (x,y), (x+w, y+h), (0,255,255), 1)
                        detect_status = 'FOUND'
                        ball_detected = True
                        x_ball, y_ball, w_ball, h_ball = x,y,w,h
                        x_center_ball = int(cx)
                        y_center_ball = int(cy)
                        ball_area = int(area)
                        last_detection_time = current_time

        direction = ""
        if static_bbox is not None:
            x_s, y_s, w_s, h_s = static_bbox
            cv2.rectangle(img, (x_s, y_s), (x_s + w_s, y_s + h_s), (0, 255, 0), 2)
            cv2.line(img, (int(x_s + w_s * 0.4), y_s), (int(x_s + w_s * 0.4), y_s + h_s), (0,0,255), 1)
            cv2.line(img, (int(x_s + 2*w_s * 0.3), y_s), (int(x_s + 2*w_s * 0.3), y_s + h_s), (0,0,255), 1)

            if ball_detected:
                bx = x_ball + w_ball/2
                by = y_ball + h_ball/2
                if (bx < x_s) or (bx > x_s + w_s) or (by < y_s) or (by > y_s + h_s):
                    rel_x = bx - x_s
                    if rel_x < w_s * 0.4:
                        direction = "kiri"
                    elif rel_x > 2 * w_s * 0.3:
                        direction = "kanan"
                    else:
                        direction = "tengah"

                    pub_direction.publish(direction)
                    static_bbox = None

        send_ball_status(detect_status)
        pub_area_msg = Ballarea()
        pub_area_msg.ballarea = int(ball_area)
        pub_area.publish(pub_area_msg)
        print(f"TESSS : {ball_area}")

        send_ball_position(float(x_center_ball), float(y_center_ball), float(FRAME_W), float(FRAME_H), int(ball_area))

        cv2.putText(img, f"FPS: {1.0 / max(1e-6, time.time() - start_time):.1f}", (10, FRAME_H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,255), 2)
        cv2.imshow("Keeper PENALTY", img)
        print(detect_status)
        print(direction)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        rate.sleep()

    try:
        vs.stop()
    except Exception:
        pass
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        rospy.init_node("Gandamana_YOLOv5n_KeeperPenalty", anonymous=False)
        deteksi_ball_loop()
    except rospy.ROSInterruptException:
        pass