#!/usr/bin/python
import random as r
import time
import math

import rospy
import numpy as np
from std_msgs.msg import (Bool, Float32, Float32MultiArray, Float64, Int16, Int32, String)
from geometry_msgs.msg import Pose2D
from communication.msg import (RobotInfo, StrategiInfo)
from protocolgc.msg import GameControllerState
from robotis_controller_msgs.msg import SyncWriteItem
from sensor_msgs.msg import (Imu, JointState)

# Koordinat strategis di lapangan
KOORDINAT_TITIK = {
    "TENGAH": (0, 0),
    "KIRI": (-1, 0),
    "KANAN": (1, 0),
    "DEPAN_GAWANG": (0, 2),
    "BELAKANG": (0, -1),
    "SUDUT_KIRI": (-2, 1),
    "SUDUT_KANAN": (2, 1)
}

Task = 0
readygate = initgate = setgate = playgate = finishgate = 0
JERSEY = "MAGENTA"
STATE = ""
pos_x, pos_y = 0, 0
status_robot = "MENUNGGU"
deteksi_bola = "N"
target_koordinat = "TENGAH"
robot_id = "ROBOT 2"
status_robot_kawan = "MENUNGGU"
last_ball_position = (0, 0)

id_1 = id_2 = id_3 = 0
fall_1 = fall_2 = fall_3 = ""
task_1 = task_2 = task_3 = ""
posx_1 = posx_2 = posx_3 = 0
posy_1 = posy_2 = posy_3 = 0
posw_1 = posw_2 = posw_3 =  0
detect_1 = detect_2 = detect_3 = ""
ballarea_1 = ballarea_2 = ballarea_3 = 0
goalorient_1 = goalorient_2 = goalorient_3 = 0
koneksi = ""

def GCState_Callback(data):
    global STATE, JERSEY, status_robot, Task, readygate, initgate, setgate, playgate, finishgate
    ID = data.robot_id
    JERSEY = data.jersey
    KICKOFF = data.kickoff
    STATE = data.state
    PENALTY = data.penalty
    PENALTY_TIME = data.penalty_time

    if STATE == "STATE_INITIAL":
        initgate +=1
        readygate = setgate = playgate =finishgate = 0
        if initgate <= 1:
            status_robot = 'READY'
    elif STATE == 'STATE_READY':
        readygate +=1
        initgate = setgate = playgate = finishgate = 0
        if readygate <= 1:
            status_robot = 'PRE_POSITIONING_KICKOFF'
    elif STATE == "STATE_SET":
        setgate +=1
        initgate = readygate = playgate = finishgate = 0
        if setgate <= 1:
            status_robot = 'SET'
    elif STATE == 'STATE_PLAYING':
        playgate +=1
        initgate = setgate = readygate = finishgate = 0
        if playgate <= 1:
            status_robot = 'PRE_RUN'
    elif STATE == 'STATE_FINISHED':
        finishgate += 1
        initgate = setgate = readygate = playgate = 0
        if finishgate <= 1:
            status_robot = 'PRE_STOP'

def posisi_callback(data):
    global pos_x, pos_y
    pos_x, pos_y = data.x, data.y
    evaluasi_posisi()

def task_callback(data):
    global status_robot
    if data.data == "KICK":
        status_robot = "MENYERANG"
        rospy.loginfo(f"{robot_id} menendang bola!")
        time.sleep(1)
        status_robot = "GERAK_KE_KOORDINAT"
        evaluasi_posisi()

def ball_state_callback(data):
    global deteksi_bola, last_ball_position
    deteksi_bola = "Y" if data.data == "FOUND" else "N"
    if deteksi_bola == "Y":
        last_ball_position = (pos_x, pos_y)
    if status_robot == "MENGEJAR_BOLA" and deteksi_bola == "Y":
        rospy.loginfo(f"{robot_id} menemukan bola, menyerang!")
        status_robot = "MENYERANG"
        time.sleep(1)
        status_robot = "MENUNGGU"
    elif status_robot == "MENGEJAR_BOLA" and deteksi_bola == "N":
        rospy.loginfo(f"{robot_id} kehilangan bola, mencari ulang!")
        status_robot = "CARI_BOLA"
        evaluasi_posisi()
    publish_strategi_status()

def strategi_kawan_callback(data):
    global status_robot_kawan, target_koordinat
    status_robot_kawan = data.status
    rospy.loginfo(f"{robot_id} menerima status kawan: {status_robot_kawan}")
    if status_robot_kawan == "MENUNGGU" and status_robot != "MENYERANG":
        status_robot = "GERAK_KE_KOORDINAT"
        target_koordinat = tentukan_titik_terdekat(last_ball_position)
        evaluasi_posisi()

def evaluasi_posisi():
    global status_robot, target_koordinat
    if status_robot == "GERAK_KE_KOORDINAT":
        target_koordinat = tentukan_titik_terdekat(last_ball_position if robot_id == "ROBOT 2" else (pos_x, pos_y))
        rospy.loginfo(f"{robot_id} menuju {target_koordinat} ({KOORDINAT_TITIK[target_koordinat]})")
        time.sleep(2)
        status_robot = "MENGEJAR_BOLA"
        rospy.loginfo(f"{robot_id} mulai mengejar bola!")
    elif status_robot == "CARI_BOLA":
        rospy.loginfo(f"{robot_id} mencari bola...")
        target_koordinat = "TENGAH" if pos_y < 0 else "DEPAN_GAWANG"
        rospy.loginfo(f"{robot_id} bergerak ke {target_koordinat}")
        time.sleep(2)
        status_robot = "MENUNGGU"
    publish_strategi_status()

def tentukan_titik_terdekat(pos):
    return min(KOORDINAT_TITIK.keys(), key=lambda k: math.sqrt((pos[0] - KOORDINAT_TITIK[k][0])**2 + (pos[1] - KOORDINAT_TITIK[k][1])**2))

def publish_strategi_status():
    msg = StrategiInfo()
    msg.status = status_robot
    msg.target_koordinat = target_koordinat
    msg.pos_x = pos_x
    msg.pos_y = pos_y
    msg.deteksi_bola = deteksi_bola
    strategi_pub.publish(msg)
    rospy.loginfo(f"{robot_id} mengirim status: {msg}")

def RobotInfo_Callback(data):
    global koneksi, id_1,fall_1, task_1, posx_1, posy_1, posw_1, detect_1, ballarea_1, id_2,fall_2, task_2, posx_2, posy_2, posw_2, detect_2, ballarea_2, id_3,fall_3, task_3, posx_3, posy_3, posw_3, detect_3, ballarea_3, goalorient_1, goalorient_2, goalorient_3

    id = data.playerID
    fall = data.fall
    task = data.task
    posx = data.posx
    posy = data.posy
    posw = data.posw
    detect = data.detect
    ballarea = data.ballsize
    goalorient = data.goalorient
    koneksi = "Y"
    if id == 1:
        id_1 = id
        fall_1 = fall
        task_1 = task
        posx_1 = posx
        posy_1 = posy
        posw_1 = posw
        detect_1 = detect
        ballarea_1 = ballarea
        goalorient_1 = goalorient
    elif id == 2:
        id_2 = id
        fall_2 = fall
        task_2 = task
        posx_2 = posx
        posy_2 = posy
        posw_2 = posw
        detect_2 = detect
        ballarea_2 = ballarea
        goalorient_2 = goalorient
    elif id == 3:
        id_3 = id
        fall_3 = fall
        task_3 = task
        posx_3 = posx
        posy_3 = posy
        posw_3 = posw
        detect_3 = detect
        ballarea_3 = ballarea
        goalorient_3 = goalorient
    elif id == 100:
        id_1 = id_2 = id_3 = 0
        fall_1 = fall_2 = fall_3 = ""
        task_1 = task_2 = task_3 = ""
        posx_1 = posx_2 = posx_3 = 0
        posy_1 = posy_2 = posy_3 = 0
        posw_1 = posw_2 = posw_3 =  0
        detect_1 = detect_2 = detect_3 = ""
        ballarea_1 = ballarea_2 = ballarea_3 = 0
        goalorient_1 = goalorient_2 = goalorient_3 = 0
        koneksi = "N"


comm_status = ""
def from_udp_to_ros(data):
    global comm_status, Task
    comm_status = data.data
    if comm_status == 'rbt_positioning':
        Task = 'walk_inarea'
    elif comm_status == 'rbt_following':
        Task = 'follow_ball'


if __name__ == '__main__':
    rospy.init_node("StrategiKoordinat")
    robot_id = rospy.get_param("ROBOT 2")
    SyncWritePub = rospy.Publisher("/robotis/sync_write_item", SyncWriteItem, queue_size=1)
    task_Pub = rospy.Publisher("/DEWO/Odometry/task", String, queue_size=1 )
    OdomCommand = rospy.Publisher("/DEWO/Odometry/cmd", String, queue_size=1)
    goalpospub = rospy.Publisher("/DEWO/Odometry/goal_position",Pose2D, queue_size=1)
    initpointpub = rospy.Publisher("/DEWO/Odometry/initpoint", Pose2D, queue_size = 1)

    TaskcomPub = rospy.Publisher("/DEWO/Communication/Task", String, queue_size=1) 
    BodystatePub = rospy.Publisher("/DEWO/Communication/Bodystate", String, queue_size=1)
    from_ros_to_udp = rospy.Publisher("/DEWO/Communication/from_ros_to_udp", String, queue_size=1)
    strategi_pub = rospy.Publisher(f"/DEWO/Communication/strategi_info", StrategiInfo, queue_size=10)
    
    
    rospy.Subscriber("/DEWO/Odometry/position", Pose2D, posisi_callback)
    rospy.Subscriber("/DEWO/image_processing/ball_state", String, ball_state_callback)

    rospy.Subscriber("/DEWO/Communication/robotinfo", RobotInfo, RobotInfo_Callback)
    rospy.Subscriber("/DEWO/Communication/from_udp_to_ros", String, from_udp_to_ros)
    rospy.Subscriber("/DEWO/Communication/shooter", String, task_callback)
    rospy.Subscriber(f"/DEWO/Communication/{'ROBOT 2' if robot_id == 'ROBOT 3' else 'ROBOT 1'}/strategi_info", StrategiInfo, strategi_kawan_callback)

    rospy.Subscriber("/DEWO/GameController/allstate", GameControllerState, GCState_Callback)
    
    rospy.spin()
