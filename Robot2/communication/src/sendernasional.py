#!/usr/bin/python
#-*- coding:utf-8 -*-
import rospy
import socket
import time
import json
from std_msgs.msg import String, Float32
from deteksi_bola.msg import BallState, BallCoordinate
from geometry_msgs.msg import Pose2D

enabel = ""
def send_Callback(data):
    global enabel
    enabel = data.data



ball_size = 0
def ballsize_callback(data):
    global ball_size, detect
    ball_size = data.obj_size
    # ball_size = 0


Xpos = 0
Ypos = 0
Wpos = 0
def location_Callback(data):
    global Xpos, Ypos, Wpos
    Xpos = round(data.x, 1)
    Ypos = round(data.y, 1)
    Wpos = round(data.theta, 1)

task = "o"
def Task_callback(data):
    global task
    task = data.data

bodystate = "o"
def Bodystate_callback(data):
    global bodystate
    bodystate = data.data
    if bodystate == "fallen_forward" or bodystate == "fallen_backward":
        bodystate = "Y"
    else:
        bodystate = "N"

goal_orientation = 0
def goalangle_Callback(data):
    global goal_orientation
    goal_orientation = data.data

detect = "o"
def ComCB_callback(data):
    global detect
    detect = data.data

detect = "o"
def ball_state_callback(data):
    global detect, ball_size
    if data.ball_status ==  "FOUND":
        detect = "Y"
    elif data.ball_status == "NOTFOUND":
        detect = "N"
        ball_size = 0
    else:
        detect = "N"

def init():
    rospy.init_node("sendernasional")
    rospy.Subscriber("/DEWO/Communication/kickoff", String, send_Callback)
    rospy.Subscriber("/DEWO/Communication/Task", String, Task_callback)
    rospy.Subscriber("/DEWO/Communication/Bodystate", String, Bodystate_callback)
    # rospy.Subscriber("/DEWO/Communication/com_to_cb", String, ComCB_callback)
    rospy.Subscriber("/DEWO/image_processing/deteksi_bola/ball_state", BallState, ball_state_callback)
    rospy.Subscriber("/DEWO/image_processing/deteksi_bola/coordinate", BallCoordinate, ballsize_callback)
    rospy.Subscriber("/DEWO/Odometry/position", Pose2D, location_Callback)
    rospy.Subscriber("/DEWO/Odometry/goalangle", Float32, goalangle_Callback)
    time.sleep(0.2)

if __name__ == '__main__':
    init()
    try:
        rospy.loginfo("GANDAMANA SENDER REDIII !!!")
        host='192.168.1.32' #IP KITA (ROBOT3)
        port = 6604

        # robot3 = ('192.168.1.33', 6604)#IP, PORT TUJUAN(ROBOT1)
        # robot2 = ('192.168.1.32', 6604)#IP, PORT TUJUAN(ROBOT2)
        # robot4 = ('192.168.1.34', 6604)#IP, PORT TUJUAN(ROBOT4)
        robot5 = ('192.168.1.35', 6604)#IP, PORT TUJUAN(ROBOT5)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        while not rospy.is_shutdown():
            data = ([2,bodystate, task, Xpos, Ypos, Wpos, detect, ball_size, goal_orientation])
            msg = json.dumps(data)
            s.sendto(msg.encode(), robot5)
            # s.sendto(msg.encode(), robot2)
            print("SEND: ", data)
            time.sleep(0.1) 
    except rospy.ROSInterruptException:
        s.close()
