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
import math
from threading import Thread
from deteksi_bola.msg import BallState, BallCoordinate
from std_msgs.msg import String

TRACKING_YOLO = 0
TRACKING_HSV = 1
tracking_state = TRACKING_YOLO
hsv_lower = None
hsv_upper = None
last_hsv_update = 0
HSV_UPDATE_INTERVAL = 1.5
MIN_HSV_CONFIDENCE = 5
MAX_LOST_COUNT = 15
MAX_YOLO_FALLBACK = 10
hsv_quality_score = 0
yolo_quality_score = 0
QUALITY_THRESHOLD = 0.7
MIN_YOLO_CONFIDENCE = 0.5

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
        while not self.stopped:
            grabbed, frame = self.stream.read()
            if grabbed:
                self.frame = frame
    
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
toleransi_movement = 50
MIN_BALL_AREA = 60
MAX_BALL_AREA = 2000
FISHEYE_CORRECTION = 1.0
lost_tracking_count = 0
hsv_confidence_count = 0
yolo_fallback_count = 0

subprocess.call(['sh', '/home/robotis/catkin_ws/src/DEWO-OP3/KRI_2023/ImageProcess/object_detect/src/camera_setting.sh'])
net = cv2.dnn.readNet("/home/robotis/catkin_ws/src/DEWO-OP3/KRI2024/Imageprocessing/v1_detection/cfg/bola.cfg", "/home/robotis/catkin_ws/database/weights/bola.weights")
vs = WebcamVideoStream(src=0).start()

rospy.init_node("Gandamana_DarknetVision", anonymous=True)
ball_pos = rospy.Publisher("/DEWO/image_processing/deteksi_bola/coordinate", BallCoordinate, queue_size=10)
ball_state = rospy.Publisher("/DEWO/image_processing/deteksi_bola/ball_state", BallState, queue_size=10)
ball_direction = rospy.Publisher("/DEWO/image_processing/deteksi_bola/ball_direction", String, queue_size=10)

classes = []
with open("/home/robotis/catkin_ws/src/DEWO-OP3/KRI2024/Imageprocessing/v1_detection/class/bola.txt", "r") as f:
    classes = f.read().splitlines()

def send_ball_position(x_pos, y_pos, w_frame, h_frame, size_obj):
    ballposition = BallCoordinate()
    fix_x = float(x_pos / w_frame * 2 - 1)
    fix_y = float(y_pos / h_frame * 2 - 1)
    ballposition.pos_x = fix_x
    ballposition.pos_y = fix_y
    ballposition.obj_size = int(size_obj)
    ball_pos.publish(ballposition)

def send_ball_status(msg):
    ballstatus = BallState()
    ballstatus.ball_status = msg
    ball_state.publish(ballstatus)

def create_field_mask(frame):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_green = np.array([35, 40, 40], dtype=np.uint8)
    upper_green = np.array([90, 255, 255], dtype=np.uint8)
    field_mask = cv2.inRange(hsv_frame, lower_green, upper_green)
    
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    field_mask = cv2.morphologyEx(field_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    field_mask = cv2.morphologyEx(field_mask, cv2.MORPH_OPEN, kernel, iterations=1)
    
    contours, _ = cv2.findContours(field_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    
    max_contour = max(contours, key=cv2.contourArea)
    hull = cv2.convexHull(max_contour)
    
    final_mask = np.zeros_like(field_mask)
    cv2.drawContours(final_mask, [hull], 0, 255, cv2.FILLED)
    
    return final_mask

def calculate_hsv_bounds(frame, x, y, w, h):
    if w <= 10 or h <= 10:
        return None, None
        
    points = []
    center_x = x + w//2
    center_y = y + h//2
    points.append((center_x, center_y))  # Tengah
    points.append((x + w//4, y + h//4))  # Kiri atas
    points.append((x + 3*w//4, y + h//4))  # Kanan atas
    points.append((x + w//4, y + 3*h//4))  # Kiri bawah
    points.append((x + 3*w//4, y + 3*h//4))  # Kanan bawah
    
    h_vals, s_vals, v_vals = [], [], []
    
    for px, py in points:
        px, py = int(px), int(py)
        if 0 <= px < frame.shape[1] and 0 <= py < frame.shape[0]:
            bgr_val = frame[py, px]
            hsv_val = cv2.cvtColor(np.uint8([[bgr_val]]), cv2.COLOR_BGR2HSV)[0][0]
            h_vals.append(hsv_val[0])
            s_vals.append(hsv_val[1])
            v_vals.append(hsv_val[2])
    
    if not h_vals:
        return None, None
        
    min_h = max(0, min(h_vals) - 5)
    max_h = min(22, max(h_vals) + 5)
    min_s = max(100, min(s_vals) - 20)
    max_s = min(255, max(s_vals) + 20)
    min_v = max(50, min(v_vals) - 30)
    max_v = min(255, max(v_vals) + 30)
    
    lower_bound = np.array([min_h, min_s, min_v], dtype=np.uint8)
    upper_bound = np.array([max_h, max_s, max_v], dtype=np.uint8)
    
    return lower_bound, upper_bound

def track_with_hsv(frame, prev_bbox=None):
    global hsv_lower, hsv_upper
    
    if hsv_lower is None or hsv_upper is None:
        return None
        
    field_mask = create_field_mask(frame)
    if field_mask is None:
        field_mask = np.ones(frame.shape[:2], dtype=np.uint8) * 255
    
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    ball_mask = cv2.inRange(hsv_frame, hsv_lower, hsv_upper)
    combined_mask = cv2.bitwise_and(ball_mask, field_mask)
    
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    processed_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
    processed_mask = cv2.morphologyEx(processed_mask, cv2.MORPH_CLOSE, kernel)
    
    if prev_bbox is not None and prev_bbox[2] > 0 and prev_bbox[3] > 0:
        try:
            x, y, w, h = prev_bbox
            padding = int(50 * FISHEYE_CORRECTION)
            x1 = max(0, x - padding)
            y1 = max(0, y - padding)
            x2 = min(frame.shape[1], x + w + padding)
            y2 = min(frame.shape[0], y + h + padding)
            
            search_mask = np.zeros_like(processed_mask)
            cv2.rectangle(search_mask, (x1, y1), (x2, y2), 255, cv2.FILLED)
            processed_mask = cv2.bitwise_and(processed_mask, search_mask)
        except Exception as e:
            rospy.logwarn("ROI masking error: {}".format(e))
    
    contours, _ = cv2.findContours(processed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return None
        
    best_contour = None
    best_circularity = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_BALL_AREA or area > MAX_BALL_AREA:
            continue
            
        perimeter = cv2.arcLength(cnt, True)
        if perimeter < 1e-5:
            continue
            
        circularity = 4 * np.pi * area / (perimeter ** 2)
        if circularity > 0.6 and circularity > best_circularity:
            best_contour = cnt
            best_circularity = circularity
    
    if best_contour is None:
        return None
        
    x, y, w, h = cv2.boundingRect(best_contour)
    
    aspect_ratio = w / float(h)
    if aspect_ratio < 0.7 or aspect_ratio > 1.3:
        return None
        
    return (x, y, w, h)

def cal_detection_quality(bbox, prev_bbox=None):
    if bbox is None:
        return 0.0
    
    x, y, w, h = bbox
    area = w * h
    aspect_ratio = w / float(h)

    area_quality = min(1.0, area / 2000.0) if area < 2000 else max(0.3, 2000.0 / area)
    aspect_quality = 1.0 if 0.7 <= aspect_ratio <= 1.3 else max(0.2, 1.0 - abs(aspect_ratio - 1.0))
    
    base_quality = (area_quality + aspect_quality) / 2.0
    
    if prev_bbox is not None:
        prev_x, prev_y, prev_w, prev_h = prev_bbox
        prev_center = (prev_x + prev_w/2, prev_y + prev_h/2)
        curr_center = (x + w/2, y + h/2)
        
        distance = math.sqrt((curr_center[0] - prev_center[0])**2 + 
                           (curr_center[1] - prev_center[1])**2)
        size_diff = abs(area - (prev_w * prev_h)) / max(area, prev_w * prev_h)
        
        distance_score = max(0.0, 1.0 - distance / 100.0)
        size_score = max(0.0, 1.0 - size_diff)
        consistency_score = distance_score * size_score
        
        return (base_quality + consistency_score) / 2.0
    
    return base_quality

def ganti_hsv(yolo_detections, hsv_bounds_quality):
    global hsv_confidence_count, tracking_state
    
    if tracking_state == TRACKING_HSV:
        return False
    
    if yolo_detections and hsv_bounds_quality > 0.7:
        hsv_confidence_count += 1
        if hsv_confidence_count >= MIN_HSV_CONFIDENCE:
            hsv_confidence_count = 0
            return True
    else:
        hsv_confidence_count = max(0, hsv_confidence_count - 1)
    
    return False

def back_yolo(hsv_quality, consecutive_failures):
    global yolo_fallback_count
    
    if hsv_quality < QUALITY_THRESHOLD or consecutive_failures >= MAX_LOST_COUNT:
        yolo_fallback_count += 1
        if yolo_fallback_count >= MAX_YOLO_FALLBACK:
            yolo_fallback_count = 0
            return True
    else:
        yolo_fallback_count = max(0, yolo_fallback_count - 1)
    
    return False

def cal_hsv_bounds_quality(frame, x, y, w, h):
    if w <= 10 or h <= 10:
        return 0.0
    
    try:
        roi = frame[y:y+h, x:x+w]
        if roi.size == 0:
            return 0.0
            
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        h_var = np.var(hsv_roi[:,:,0])
        s_var = np.var(hsv_roi[:,:,1])
        v_var = np.var(hsv_roi[:,:,2])
        
        quality = 1.0 / (1.0 + (h_var + s_var + v_var) / 3000.0)
        return min(1.0, quality)
    except Exception:
        return 0.0

def darknet_process():
    global tracking_state, hsv_lower, hsv_upper, last_hsv_update
    global static_bbox, last_ball_position, stationary_start_time
    global last_detection_time, lost_tracking_count
    global hsv_quality_score, yolo_quality_score
    global hsv_confidence_count, yolo_fallback_count
    
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    
    consecutive_hsv_failures = 0
    consecutive_yolo_detections = 0
    
    while not rospy.is_shutdown():
        start_time = time.time()
        img = vs.read()
        if img is None:
            continue
            
        current_time = time.time()
        ball_detected = False
        x_ball, y_ball, w_ball, h_ball = 0, 0, 0, 0
        prev_bbox = last_ball_position
        
        height, width = img.shape[:2]
        
        if tracking_state == TRACKING_YOLO:
            blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
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
                    if confidence > MIN_YOLO_CONFIDENCE:
                        center_x = int(detection[0] * width)
                        center_y = int(detection[1] * height)
                        w = int(detection[2] * width)
                        h = int(detection[3] * height)
                        x = int(center_x - w / 2)
                        y = int(center_y - h / 2)
                        boxes.append([x, y, w, h])
                        confidences.append(float(confidence))
                        class_ids.append(class_id)
            
            indexes = cv2.dnn.NMSBoxes(boxes, confidences, MIN_YOLO_CONFIDENCE, 0.4)
            
            if len(indexes) > 0:
                last_detection_time = current_time
                consecutive_yolo_detections += 1
            else:
                consecutive_yolo_detections = 0
                if last_detection_time and (current_time - last_detection_time > timeout_bbox):
                    static_bbox = None
            
            if len(boxes) == 0:
                last_ball_position = None
                stationary_start_time = None
            
            for i in range(len(boxes)):
                if i in indexes:
                    x, y, w, h = boxes[i]
                    label = str(classes[class_ids[i]]) if class_ids[i] < len(classes) else "unknown"
                    
                    if label == "bola":
                        x_ball, y_ball, w_ball, h_ball = x, y, w, h
                        ball_detected = True
                        center_x = x + w/2
                        center_y = y + h/2
                        ball_area = w * h

                        yolo_quality_score = cal_detection_quality((x, y, w, h), prev_bbox)
                        hsv_bounds_quality = cal_hsv_bounds_quality(img, x, y, w, h)
                        
                        if ganti_hsv(consecutive_yolo_detections >= 3, hsv_bounds_quality):
                            new_lower, new_upper = calculate_hsv_bounds(img, x, y, w, h)
                            if new_lower is not None and new_upper is not None:
                                hsv_lower = new_lower
                                hsv_upper = new_upper
                                tracking_state = TRACKING_HSV
                                last_hsv_update = current_time
                                lost_tracking_count = 0
                                consecutive_hsv_failures = 0
                                rospy.loginfo("Ganti ke HSV (Quality: {:.2f})".format(hsv_bounds_quality))
                        
                        if last_ball_position is not None:
                            prev_x, prev_y, prev_w, prev_h = last_ball_position
                            prev_center_x = prev_x + prev_w/2
                            prev_center_y = prev_y + prev_h/2
                            distance = math.sqrt((center_x - prev_center_x)**2 + (center_y - prev_center_y)**2)
                            
                            if distance < toleransi_movement:
                                if stationary_start_time is None:
                                    stationary_start_time = current_time
                                elif current_time - stationary_start_time > batas_bola_diam:
                                    if static_bbox is None:
                                        if ball_area < 5000 * FISHEYE_CORRECTION:
                                            margin_w = int(w_ball * 3.0)
                                            margin_h = int(h_ball * 0.45)
                                        else:
                                            margin_w = int(w_ball * 2.0)
                                            margin_h = int(h_ball * 0.15)
                                        
                                        static_bbox = (
                                            max(0, x_ball - margin_w),
                                            max(0, y_ball - margin_h),
                                            min(width, w_ball + 2 * margin_w),
                                            min(height, h_ball + 2 * margin_h)
                                        )
                            else:
                                stationary_start_time = None
                                static_bbox = None
                        else:
                            stationary_start_time = current_time
                        
                        last_ball_position = (x_ball, y_ball, w_ball, h_ball)
                    
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 1)
                    text = "{}: {:.2f}".format(label, confidences[i])
                    size = w + h
                    besarBola = "size: {:.2f}".format(size)
                    cv2.putText(img, besarBola, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
                    cv2.putText(img, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
        
        elif tracking_state == TRACKING_HSV:
            if hsv_lower is None or hsv_upper is None:
                tracking_state = TRACKING_YOLO
                rospy.logwarn("Balik ke YOLO")
                result = None
            else:
                result = track_with_hsv(img, prev_bbox)
            
            if result is not None:
                x_ball, y_ball, w_ball, h_ball = result
                ball_detected = True
                lost_tracking_count = 0
                consecutive_hsv_failures = 0
                center_x = x_ball + w_ball/2
                center_y = y_ball + h_ball/2
                ball_area = w_ball * h_ball
                
                hsv_quality_score = cal_detection_quality((x_ball, y_ball, w_ball, h_ball), prev_bbox)
                
                if (current_time - last_hsv_update > HSV_UPDATE_INTERVAL and hsv_quality_score > QUALITY_THRESHOLD):
                    new_lower, new_upper = calculate_hsv_bounds(img, x_ball, y_ball, w_ball, h_ball)
                    if new_lower is not None and new_upper is not None:
                        hsv_lower = new_lower
                        hsv_upper = new_upper
                        last_hsv_update = current_time
                
                if last_ball_position is not None:
                    prev_x, prev_y, prev_w, prev_h = last_ball_position
                    prev_center_x = prev_x + prev_w/2
                    prev_center_y = prev_y + prev_h/2
                    distance = math.sqrt((center_x - prev_center_x)**2 + (center_y - prev_center_y)**2)
                    
                    if distance < toleransi_movement:
                        if stationary_start_time is None:
                            stationary_start_time = current_time
                        elif current_time - stationary_start_time > batas_bola_diam:
                            if static_bbox is None:
                                if ball_area < 5000 * FISHEYE_CORRECTION:
                                    margin_w = int(w_ball * 3.0)
                                    margin_h = int(h_ball * 0.45)
                                else:
                                    margin_w = int(w_ball * 2.0)
                                    margin_h = int(h_ball * 0.15)
                                
                                static_bbox = (
                                    max(0, x_ball - margin_w),
                                    max(0, y_ball - margin_h),
                                    min(width, w_ball + 2 * margin_w),
                                    min(height, h_ball + 2 * margin_h)
                                )
                    else:
                        stationary_start_time = None
                        static_bbox = None
                else:
                    stationary_start_time = current_time
                
                cv2.rectangle(img, (x_ball, y_ball), (x_ball + w_ball, y_ball + h_ball), (0, 255, 0), 2)
                cv2.putText(img, "HSV Tracking (Q:{:.2f})".format(hsv_quality_score), (x_ball, y_ball - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                
                last_ball_position = (x_ball, y_ball, w_ball, h_ball)
            else:
                lost_tracking_count += 1
                consecutive_hsv_failures += 1
                hsv_quality_score = max(0.0, hsv_quality_score - 0.1)
                
                if back_yolo(hsv_quality_score, consecutive_hsv_failures):
                    tracking_state = TRACKING_YOLO
                    ball_detected = False
                    last_ball_position = None
                    stationary_start_time = None
                    consecutive_hsv_failures = 0
                    rospy.loginfo("Balik ke Yolo")
        
        if static_bbox is not None and ball_detected:
            x_s, y_s, w_s, h_s = static_bbox
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
                    elif rel_x > 2 * w_s / 3:
                        direction = "kanan"
                    else:
                        direction = "tengah"
                    
                    ball_direction.publish(direction)
                    static_bbox = None
        
        if ball_detected:
            x_center_ball = x_ball + w_ball/2
            y_center_ball = y_ball + h_ball/2
            obj_size_ball = w_ball * h_ball
            
            send_ball_position(float(x_center_ball), float(y_center_ball), 320.0, 240.0, obj_size_ball)
            send_ball_status("FOUND")
        else:
            send_ball_position(0.0, 0.0, 320.0, 240.0, 0)
            send_ball_status("NOTFOUND")
            if tracking_state == TRACKING_YOLO:
                last_ball_position = None
            stationary_start_time = None
        
        mode_text = "YOLO" if tracking_state == TRACKING_YOLO else "HSV"
        quality_score = yolo_quality_score if tracking_state == TRACKING_YOLO else hsv_quality_score
        cv2.putText(img, "Mode: {} (Q:{:.2f})".format(mode_text, quality_score), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
        
        processing_time = time.time() - start_time
        fps = 1.0 / processing_time if processing_time > 0 else 0
        cv2.putText(img, "FPS: {:.1f}".format(fps), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
        cv2.putText(img, "HSV Fails: {}".format(consecutive_hsv_failures), (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
        cv2.imshow("GANDAMANA_VISION", img)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    vs.stop()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        darknet_process()
    except rospy.ROSInterruptException:
        pass