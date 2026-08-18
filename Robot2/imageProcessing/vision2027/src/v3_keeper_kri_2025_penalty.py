#!/usr/bin/env python3.7

import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'

if ros_path in sys.path:

    sys.path.remove(ros_path)

import cv2
import math

sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
# import cv2
import subprocess
import rospy
import numpy as np
import glob
import random
import time
from threading import Thread
from deteksi_bola.msg import BallState, BallCoordinate
from std_msgs.msg import String

class WebcamVideoStream:
    def __init__(self, src=0):
        self.stream = cv2.VideoCapture(src, cv2.CAP_V4L2)
        self.stream.set(cv2.CAP_PROP_FPS, 30)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        (self.grabbed, self.frame) = self.stream.read()
        self.blank_frame = np.zeros_like(self.frame)
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
net = cv2.dnn.readNet("/home/robotis/catkin_ws/src/DEWO-OP3/KRI2024/Imageprocessing/v1_detection/cfg/bola.cfg","/home/robotis/catkin_ws/database/weights/bola_wilayah.weights")
vs = WebcamVideoStream(src=0).start()
ball_pos = rospy.Publisher("/DEWO/image_processing/deteksi_bola/coordinate", BallCoordinate, queue_size=10)
ball_state = rospy.Publisher("/DEWO/image_processing/deteksi_bola/ball_state", BallState, queue_size=10)
motion_keeper = rospy.Publisher("/DEWO/image_processing/deteksi_bola/decision", String, queue_size=10)
grid_cols = 32
grid_rows = 24
cell_width = 320 // grid_cols
cell_height = 240 // grid_rows


classes = []
with open("/home/robotis/catkin_ws/src/DEWO-OP3/KRI2024/Imageprocessing/v1_detection/class/bola_fisheye.txt", "r") as f:
    classes = f.read().splitlines()

def draw_arrow(image, start_point, end_point, color, thickness, tip_length):
    cv2.arrowedLine(image, start_point, end_point, color, thickness, tipLength=tip_length)

def trajectory_prediction(pos_prev, pos_latest, grid_cols, grid_rows, blank_frame, cell_width, cell_height):
    dx = pos_latest[0] - pos_prev[0]
    dy = pos_latest[1] - pos_prev[1]
    vector_length = np.sqrt(dx ** 2 + dy ** 2)
    if vector_length == 0:
        return
    theta = math.atan2(dy, dx)
    arrow_length = int(vector_length * 0.5)
    pred_x = pos_latest[0] + int(arrow_length * math.cos(theta))
    pred_y = pos_latest[1] + int(arrow_length * math.sin(theta))
    end_point = (pred_x, pred_y)
    
    #decision lama
    # if pred_x >= 140 and pred_y <= 180:
    #     print('bola berada di tengah')
    #     if pred_y >= 80:
    #         print('MOTION HALANG DUDUK')
    # elif pred_x >= 100 and pred_y < 140:
    #     print('di samping KIRI')
    #     if pred_y >= 150:
    #         print('MOTION HALANG SAMPING KIRI')
    # elif pred_x > 180 and pred_y <= 220:
    #     print('di samping KANAN')
    #     if pred_y >= 150:
    #         print('MOTION HALANG SAMPING KANAN')
    # elif pred_x > 220:
    #     print('BAHAYA KANAN')
    #     if pred_y > 195:
    #         print('JALAN JATUH KANAN')
    # elif pred_x < 100:
    #     print('BAHAYA KANAN')
    #     if pred_y > 195:
    #         print('JALAN JATUH KANAN')

    #decision baru 
    if pred_x >= 0 and pred_x <= 100:
        print('BAHAYA KANAN')
        if pred_y >= 195:
            print('MOTION KIRI')
            motion_keeper.publish('JATUH KIRI')
    if pred_x > 100 and pred_x <= 220:
        print('BAHAYA TENGAH')
        if pred_y >= 195:
            print('MOTION TENGAH')
            motion_keeper.publish('DUDUK')
    if pred_x > 120 and pred_x <= 1320:
        print('BAHAYA KANAN')
        if pred_y >= 195:
            print('MOTION KANAN')
            motion_keeper.publish('JATUH KANAN')

    draw_arrow(blank_frame, pos_latest, end_point, (255, 255, 255), 2, tip_length=0.3)
    pred_cell_x = pred_x // cell_width
    pred_cell_y = pred_y // cell_height
    cv2.rectangle(blank_frame, (pred_cell_x * cell_width, pred_cell_y * cell_height), ((pred_cell_x + 1) * cell_width, (pred_cell_y + 1) * cell_height), (0, 0, 255), 2)


def send_ball_position(x_pos, y_pos, w_frame, h_frame, size_obj):
    ballposition = BallCoordinate()
    fix_x = (float) (x_pos / w_frame * 2 - 1)
    fix_y = (float) (y_pos / h_frame * 2 - 1)
    ballposition.pos_x = fix_x
    ballposition.pos_y = fix_y
    ballposition.obj_size = size_obj
    ball_pos.publish(ballposition)

def send_ball_status(msg):
    ballstatus = BallState()
    ballstatus.ball_status = msg
    ball_state.publish(ballstatus)

def darknet_process():
    tracking_points = []
    while not rospy.is_shutdown():
        img = vs.read()
        blank_frame = np.zeros_like(img)
        layer_names = net.getLayerNames()
        # output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
        output_layers = net.getUnconnectedOutLayersNames()
        # img = cv2.resize(img, (320,240))
        height, width, channels = img.shape
        blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        net.setInput(blob)
        start = time.time()
        outs = net.forward(output_layers)
        end = time.time()
        # print("[INFO] Waktu deteksi yolo {:.2f} detik".format(end - start))
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
        # font = cv2.FONT_HERSHEY_PLAIN
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        # unique, counts = np.unique(class_ids, return_counts=True)
        # tambah=0
        # cv2.rectangle(img, (3, 3), (165, 80), (0,0,255), 1)
        # for i in range (len(counts)):
        #                 cv2.putText(img,str(classes[i])+" = "+str(counts[i]), (5,15+tambah),font,1, (0,0,255), 1)
        #                 tambah=tambah+15
        # print(indexes)
        daftar=[]
        if len(boxes) == 0:
            # print('not found all')
            tracking_points = []
            send_ball_status("NOTFOUND")
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h =  boxes[i]
                try:
                    x_goal, y_goal, w_goal, h_goal = boxes[class_ids.index(1)]
                except ValueError as e:
                    x_goal, y_goal, w_goal, h_goal = [0, 0, 0, 0]
                try:
                    x_ball, y_ball, w_ball, h_ball = boxes[class_ids.index(0)]     
                except ValueError as e:
                    x_ball, y_ball, w_ball, h_ball = [0, 0, 0, 0]
                    print(e)
                x_center_ball = int((x_ball+w_ball/2))
                y_center_ball = int((y_ball+h_ball/2))
                tracking_points.append((x_center_ball, y_center_ball))
                print(f"x: {x}, y: {y}")
                obj_size_ball = w_ball*h_ball
                label = str(classes[class_ids[i]])
                daftar.append(label)
                cv2.rectangle(img, (x, y), (x + w, y + h), (0,255,255), 1)
                text = "{}: {:.2f}".format(label, confidences[i])
                cv2.putText(img, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)
                
                # if 0 in class_ids:
                #     send_ball_position((float)(x_center_ball), (float)(y_center_ball), 320.0, 240.0, obj_size_ball)
                #     send_ball_status("FOUND")
                #     # print('bola found')
                #     if x_center_ball >= 140 and x_center_ball <= 180:
                #         print('bola berada di tengah')
                #         if y_center_ball >= 80:
                #             print('MOTION HALANG DUDUK')
                #     elif x_center_ball >= 100 and x_center_ball < 140:
                #         print('di samping KIRI')
                #         if y_center_ball >= 150:
                #             print('MOTION HALANG SAMPING KIRI')
                #     elif x_center_ball > 180 and x_center_ball <= 220:
                #         print('di samping KANAN')
                #         if y_center_ball >= 150:
                #             print('MOTION HALANG SAMPING KANAN')
                #     elif x_center_ball > 220:
                #         print('BAHAYA KANAN')
                #         if y_center_ball > 195:
                #             print('JALAN JATUH KANAN')
                #     elif x_center_ball < 100:
                #         print('BAHAYA KANAN')
                #         if y_center_ball > 195:
                #             print('JALAN JATUH KANAN')

                # else:
                #     send_ball_position((float)(0), (float)(0), 320.0, 240.0, obj_size_ball)
                #     send_ball_status("NOTFOUND")
                #     print('bola not found')

        # # print(daftar)
        # cv2.polylines(img, [np.array([[[220, 195], [320,195]]], np.int32)], isClosed=False, color=(0, 0, 255), thickness=2)
        # cv2.polylines(img, [np.array([[[0, 195], [100,195]]], np.int32)], isClosed=False, color=(0, 0, 255), thickness=2)
        # cv2.polylines(img, [np.array([[[100, 240], [100, 150], [140,150]]], np.int32)], isClosed=False, color=(0, 255, 255), thickness=2)
        # cv2.polylines(img, [np.array([[[220, 240], [220, 150], [180,150]]], np.int32)], isClosed=False, color=(0, 255, 255), thickness=2)
        # cv2.polylines(img, [np.array([[[140, 240], [140, 80], [180,80], [180, 240]]], np.int32)], isClosed=False, color=(255, 0, 0), thickness=2)

        #new line 
        cv2.polylines(img, [np.array([[[220, 195], [320,195]]], np.int32)], isClosed=False, color=(0, 0, 255), thickness=2) #garis merah kanan
        cv2.polylines(img, [np.array([[[0, 195], [100,195]]], np.int32)], isClosed=False, color=(0, 0, 255), thickness=2) #garis merah kiri
        cv2.polylines(img, [np.array([[[100, 240], [100, 195],[220, 195], [220, 240]]], np.int32)], isClosed=False, color=(0, 255, 255), thickness=2) #garis kuning

        #draw grid
        for i in range(0, 320, cell_width):
            cv2.line(blank_frame, (i, 0), (i, 240), (255, 255, 255), 1)
        for i in range(0, 240, cell_height):
            cv2.line(blank_frame, (0, i), (320, i), (255, 255, 255), 1)

        #draw trajectory
        for i in range(1, len(tracking_points)):
            start = tracking_points[i - 1]
            end = tracking_points[i]
            cv2.arrowedLine(blank_frame, start, end, (0, 0, 255), 2, tipLength=0.3)

        #draw trajectory prediction
        if len(tracking_points) >= 2:
            trajectory_prediction(tracking_points[-2], tracking_points[-1], grid_cols, grid_rows, blank_frame, cell_width, cell_height)
        else: 
            print('Ball Not Found')
        

        cv2.imshow("Flow Prediction", blank_frame)
        cv2.imshow("GANDAMANA_VISION", img)
        if cv2.waitKey(1) == ord('q'):
                break

    vs.stop()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        rospy.init_node("Gandamana_DarknetVision")
        darknet_process()
    except rospy.ROSInterruptException():
        pass