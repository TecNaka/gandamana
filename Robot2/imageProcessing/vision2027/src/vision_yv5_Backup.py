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
import configparser
from dataclasses import dataclass
from typing import List, Tuple, Optional

import numpy as np
import cv2

# ---------------- ROS ----------------F
import rospy
from std_msgs.msg import String, Int16
from v2_detection.msg import BallState, Ballarea, BallCoordinate,OpponentState, OpponentCoordinate,GoalState, GoalCoordinate


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
framesize = [320, 240]
detect_status = 'NOTFOUND'
ball_area = 0
x_center_ball = 0
y_center_ball = 0

# ------------------------------------------------------------
# ROS Node & Publisher
# ------------------------------------------------------------
pub_state = rospy.Publisher("/vision/ball_state", BallState, queue_size=1)
pub_area = rospy.Publisher("/vision/ball_area", Ballarea, queue_size=1)
pub_coord = rospy.Publisher("/vision/ball_coordinate", BallCoordinate, queue_size=1)

rospy.init_node("vision_yv5", anonymous=False)

# ------------------------------------------------------------
# Kamera Capture (default /dev/video0)
# ------------------------------------------------------------
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, framesize[0])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, framesize[1])

rate = rospy.Rate(30)  # 30 Hz

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret:
        rospy.logwarn("Camera not detected!")
        continue

    # Run YOLOv5 inference
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = model(frame_rgb, size=416)
    det = results.xyxy[0].cpu().numpy()
    # print(model.names)


    if len(det) > 0:
        # Ambil deteksi dengan confidence tertinggi
        x1, y1, x2, y2, conf, cls = det[0]
        x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])

        # Hitung center dan area bola
        x_center_ball = int((x1 + x2) / 2)
        y_center_ball = int((y1 + y2) / 2)
        ball_area = (x2 - x1) * (y2 - y1)
        detect_status = 'FOUND'

        # Gambar bounding box
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(frame, (x_center_ball, y_center_ball), 5, (0, 0, 255), -1)
        cv2.putText(frame, f"Ball {conf:.2f}", (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

    else:
        detect_status = 'NOTFOUND'
        ball_area = 0
        x_center_ball, y_center_ball = 0, 0

    # --------------------------------------------------------
    # Publish ROS messages
    # --------------------------------------------------------
    msg_state = BallState()
    msg_state.ball_status = detect_status
    pub_state.publish(msg_state)

    msg_area = Ballarea()
    msg_area.ballarea = int(ball_area)
    pub_area.publish(msg_area)

    msg_coord = BallCoordinate()
    msg_coord.pos_x = int(x_center_ball)
    msg_coord.pos_y = int(y_center_ball)
    pub_coord.publish(msg_coord)

    # Tampilkan frame (opsional)
    cv2.imshow("YOLOv5 Ball Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    rate.sleep()

cap.release()
cv2.destroyAllWindows()