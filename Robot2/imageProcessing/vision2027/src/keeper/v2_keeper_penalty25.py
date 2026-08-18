#!/usr/bin/env python3

import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'

if ros_path in sys.path:
    sys.path.remove(ros_path)

import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import subprocess
import rospy
import numpy as np
import time
from threading import Thread
from deteksi_bola.msg import BallState, BallCoordinate
from std_msgs.msg import String

class WebcamVideoStream:
    def __init__(self, src=0):
        self.stream = cv2.VideoCapture(src, cv2.CAP_V4L2)
        self.stream.set(cv2.CAP_PROP_FPS, 60)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
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

static_bbox = None
last_ball_position = None
stationary_start_time = None
last_detection_time = None
batas_bola_diam = 2.0
timeout_bbox = 2.0
toleransi_movement = 30

subprocess.call(['sh', '/home/robotis/catkin_ws/src/DEWO-OP3/KRI_2023/ImageProcess/object_detect/src/camera_setting.sh'])
net = cv2.dnn.readNet("/home/robotis/catkin_ws/src/DEWO-OP3/KRI2024/Imageprocessing/v1_detection/cfg/bola.cfg","/home/robotis/catkin_ws/database/weights/bola.weights")
vs = WebcamVideoStream(src=0).start()
ball_pos = rospy.Publisher("/DEWO/image_processing/deteksi_bola/coordinate", BallCoordinate, queue_size=10)
ball_state = rospy.Publisher("/DEWO/image_processing/deteksi_bola/ball_state", BallState, queue_size=10)
ball_direction = rospy.Publisher("/DEWO/image_processing/deteksi_bola/ball_direction", String, queue_size=10)

classes = []
with open("/home/robotis/catkin_ws/src/DEWO-OP3/KRI2024/Imageprocessing/v1_detection/class/bola.txt", "r") as f:
    classes = f.read().splitlines()

def send_ball_position(x_pos, y_pos, w_frame, h_frame, size_obj):
    ballposition = BallCoordinate()
    fix_x = (float)(x_pos / w_frame * 2 - 1)
    fix_y = (float)(y_pos / h_frame * 2 - 1)
    ballposition.pos_x = fix_x
    ballposition.pos_y = fix_y
    ballposition.obj_size = int(size_obj)
    ball_pos.publish(ballposition)

def send_ball_status(msg):
    ballstatus = BallState()
    ballstatus.ball_status = msg
    ball_state.publish(ballstatus)

def darknet_process():
    global static_bbox, last_ball_position, stationary_start_time, last_detection_time
    
    while not rospy.is_shutdown():
        start_time = time.time()
        img = vs.read()
        layer_names = net.getLayerNames()
        output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
        height, width, channels = img.shape
        blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        net.setInput(blob)
        outs = net.forward(output_layers)
        current_time = time.time()
        
        class_ids = []
        confidences = []
        boxes = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:
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
        ball_detected = False
        x_ball, y_ball, w_ball, h_ball = 0, 0, 0, 0
        
        if len(indexes) > 0:
            last_detection_time = current_time
        elif last_detection_time is not None and current_time - last_detection_time > timeout_bbox:
            static_bbox = None
        
        if len(boxes) == 0:
            last_ball_position = None
            stationary_start_time = None
        
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(classes[class_ids[i]])
                
                if label == "bola":
                    x_ball, y_ball, w_ball, h_ball = x, y, w, h
                    ball_detected = True
                    center_x = x + w/2
                    center_y = y + h/2
                    besar = w_ball + h_ball
                    
                    if last_ball_position:
                        distance = np.sqrt((center_x - last_ball_position[0])**2 + 
                                         (center_y - last_ball_position[1])**2)
                        
                        if distance < toleransi_movement:
                            if stationary_start_time is None:
                                stationary_start_time = current_time
                            elif current_time - stationary_start_time > batas_bola_diam:
                                if static_bbox is None and besar < 71: #makin jauh bolanya, makin gede bbox e. Tujuannya biar kalo deket langsung gercep panggil motion
                                    margin_w = int(w_ball * 3.0)
                                    margin_h = int(h_ball * 0.45)
                                    static_bbox = (
                                        max(0, x_ball - margin_w),
                                        max(0, y_ball - margin_h),
                                        min(width, w_ball + 2 * margin_w),
                                        min(height, h_ball + 2 * margin_h)
                                    )
                                elif static_bbox is None and besar > 70:
                                    margin_w = int(w_ball * 2.0)
                                    margin_h = int(h_ball * 0.15)
                                    static_bbox = (
                                        max(0, x_ball - margin_w),
                                        max(0, y_ball - margin_h),
                                        min(width, w_ball + 2 * margin_w),
                                        min(height, h_ball + 2 * margin_h)
                                    )
                            else :
                                static_bbox = None
                        else:
                            stationary_start_time = None
                            static_bbox = None
                    else:
                        stationary_start_time = current_time
                    
                    last_ball_position = (center_x, center_y)
                
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 1)
                text = "{}: {:.2f}".format(label, confidences[i])
                size = h_ball + w_ball
                besarBola = "size: {:.2f}".format(size)
                cv2.putText(img, besarBola, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                cv2.putText(img, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        if static_bbox:
            x_s, y_s, w_s, h_s = static_bbox
            # bkin BBOX statis
            cv2.rectangle(img, (x_s, y_s), (x_s + w_s, y_s + h_s), (0, 255, 0), 2)
            cv2.line(img, (int(x_s + w_s/3), y_s), (int(x_s + w_s/3), y_s + h_s), (0, 0, 255), 1)
            cv2.line(img, (int(x_s + 2*w_s/3), y_s), (int(x_s + 2*w_s/3), y_s + h_s), (0, 0, 255), 1)
            
            if ball_detected:
                ball_center_x = x_ball + w_ball/2
                ball_center_y = y_ball + h_ball/2
                
                if (ball_center_x < x_s or ball_center_x > x_s + w_s or
                    ball_center_y < y_s or ball_center_y > y_s + h_s):
                    
                    rel_x = ball_center_x - x_s
                    
                    if rel_x < w_s / 3:
                        direction = "kiri"
                        print("kiri")
                    elif rel_x > 2 * w_s / 3:
                        direction = "kanan"
                        print("kanan")
                    else:
                        direction = "tengah"
                        print("tengah")
                    
                    ball_direction.publish(direction)
                    static_bbox = None
        
        if ball_detected:
            x_center_ball = x_ball + w_ball/2
            y_center_ball = y_ball + h_ball/2
            obj_size_ball = w_ball * h_ball
            
            send_ball_position((float)(x_center_ball), (float)(y_center_ball), 320.0, 240.0, obj_size_ball)
            send_ball_status("FOUND")
        else:
            send_ball_position(0.0, 0.0, 320.0, 240.0, 0)
            send_ball_status("NOTFOUND")
            last_ball_position = None
            stationary_start_time = None

        # cv2.polylines(img, [np.array([[[220, 195], [320, 195]]], np.int32)], False, (0, 0, 255), 2)
        # cv2.polylines(img, [np.array([[[0, 195], [100, 195]]], np.int32)], False, (0, 0, 255), 2)
        # cv2.polylines(img, [np.array([[[100, 240], [100, 150], [140, 150]]], np.int32)], False, (0, 255, 255), 2)
        # cv2.polylines(img, [np.array([[[220, 240], [220, 150], [180, 150]]], np.int32)], False, (0, 255, 255), 2)
        # cv2.polylines(img, [np.array([[[140, 240], [140, 80], [180, 80], [180, 240]]], np.int32)], False, (255, 0, 0), 2)
        processing_time = time.time() - start_time
        fps = 1.0 / processing_time if processing_time > 0 else 0
        cv2.putText(img, "FPS: {:.1f}".format(fps), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
        cv2.imshow("GANDAMANA_VISION", img)
        if cv2.waitKey(1) == ord('q'):
            break

    vs.stop()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        rospy.init_node("Gandamana_DarknetVision")
        darknet_process()
    except rospy.ROSInterruptException:
        pass