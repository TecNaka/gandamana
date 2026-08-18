#!/usr/bin/python
import math
import random as r
import time

import numpy as np
import rospy
from communication.msg import (RobotInfo, StrategyInfo)
from deteksi_bola.msg import (BallCoordinate, BallState, GoalCoordinate,
                              GoalState)
from geometry_msgs.msg import Pose2D
from op3_walking_module_msgs.msg import WalkingParam
from protocolgc.msg import GameControllerState
from robotis_controller_msgs.msg import SyncWriteItem
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import (Bool, Float32, Float32MultiArray, Float64, Int16,
                          Int32, String)
from tf.transformations import euler_from_quaternion
from numpy import ones,vstack
from numpy.linalg import lstsq

#from kri2024.msg import JointKaki

Status = "READY"
Task = 0
imuData = Imu()
orientation_to_goal = 0
Xgoal = Ygoal = 0
Xinit = Yinit = 0
readygate = initgate = setgate = playgate = finishgate = 0
JERSEY = "CYAN"
KICKOFF = False
STATE = ""
def GCState_Callback(data):
    global STATE, JERSEY,KICKOFF, Status, Task, readygate, initgate, setgate, playgate, finishgate
    ID = data.ID_Robot
    JERSEY = data.jersey
    KICKOFF = data.kickoff
    STATE = data.state
    PENALTY = data.penalty
    PENALTY_TIME = data.penalty_time
    # print(ID, JERSEY, KICKOFF, STATE, PENALTY, PENALTY_TIME)

    if STATE == "STATE_INITIAL":
        initgate +=1
        readygate = setgate = playgate =finishgate = 0
        if initgate <= 1:
            Status = 'READY'
    elif STATE == 'STATE_READY':
        readygate +=1
        initgate = setgate = playgate = finishgate = 0
        if readygate <= 1:
            Status = 'PRE_POSITIONING_KICKOFF'
    elif STATE == "STATE_SET":
        setgate +=1
        initgate = readygate = playgate = finishgate = 0
        if setgate <= 1:
            Status = 'SET'
    elif STATE == 'STATE_PLAYING':
        playgate +=1
        initgate = setgate = readygate = finishgate = 0
        if playgate <= 1:
            Status = 'PRE_RUN'
    elif STATE == 'STATE_FINISHED':
        finishgate += 1
        initgate = setgate = readygate = playgate = 0
        if finishgate <= 1:
            Status = 'PRE_STOP'
'''
syan = 0
magenta = 180
'''
initX = initY = initW = 0
def initpoint(position):
    global initX, initY, initW, yawOffset, X_global, Y_global
    BELAKANG = 200
    DEPAN = 100
    CyanBelakang = [0 + BELAKANG , 600]
    CyanTengah = [450 - DEPAN , 600]
    CyanBelakang_seberang = [BELAKANG , 0]
    CyanTengah_seberang = [450 - DEPAN , 0]
    MagentaBelakang = [900 - BELAKANG , 600]
    MagentaTengah = [450 + DEPAN , 600]
    MagentaBelakang_seberang = [900 - BELAKANG , 0]
    MagentaTengah_seberang = [450 + DEPAN , 0]

    if position == "CyanBelakang":
        initX = CyanBelakang[0]
        initY = CyanBelakang[1]
        initW = 90
    elif position == "CyanTengah":
        initX = CyanTengah[0]
        initY = CyanTengah[1]
        initW = 90
    elif position == "CyanBelakang_seberang":
        initX = CyanBelakang_seberang[0]
        initY = CyanBelakang_seberang[1]
        initW = -90
    elif position == "CyanTengah_seberang":
        initX = CyanTengah_seberang[0]
        initY = CyanTengah_seberang[1]
        initW = -90
    elif position == "MagentaBelakang":
        initX = MagentaBelakang[0]
        initY = MagentaBelakang[1]
        initW = 90
    elif position == "MagentaTengah":
        # initX = 550#90
        # initY = 200
        initX = MagentaTengah[0]
        initY = MagentaTengah[1]
        initW = 90
    elif position == "MagentaBelakang_seberang":
        initX = MagentaBelakang_seberang[0]
        initY = MagentaBelakang_seberang[1]
        initW = -90
    elif position == "MagentaTengah_seberang":
        initX = MagentaTengah_seberang[0]
        initY = MagentaTengah_seberang[1]
        initW = -90
    elif position == "Positioning_Kick_Off":
        if JERSEY == "CYAN":
            initX = 100#90
            initY = 300
            initW = 0
        else:
            initX = 800#90
            initY = 300
            initW = 180
        X_global = initX
        Y_global = initY
    yawOffset = -yawHeading + initW
    init = Pose2D()
    init.x = initX
    init.y = initY
    init.theta = initW
    X_global = initX
    Y_global = initY
    print (X_global, Y_global)
    initpointpub.publish(init)

button_mode = "Running"
initpoint_area = ""
def opencr_button2(data):
    global Status, yawOffset, button_mode, initpoint_area
    if data.data == 'mode':
        # print ("test")
        if button_mode == "Running":
            if Status == 'STOP':
                Status = 'PRE_IDLE' #menuju duduk
                rospy.loginfo("Pose Duduk")
            elif Status == 'IDLE': #menuju berdiri
                Status = 'READY' #berdiri
                rospy.loginfo("Pose Berdiri")
        elif button_mode == "initpoint":
            if initpoint_area == "magenta":
                LED_RGB(255, 0, 255)
                LED_Status(0, 0, 1)
                BuzzerTone(1 , 80)
                initpoint("MagentaBelakang")
                print("InitPoint : MagentaBelakang")
            elif initpoint_area == "cyan":
                print("NONE")
        
    elif data.data == 'start':
        # button_mode = "Running"
        # initpoint_area = ""
        if button_mode == "Running":
            if not Status == 'IDLE' and Status !='STOP':
                Status = 'PRE_STOP' #STOP
            elif Status == 'STOP':
                Status = 'PRE_RUN' #RUN
            
        elif button_mode == "initpoint":
            if initpoint_area == "magenta":
                LED_RGB(255, 0, 255)
                LED_Status(1, 0, 0)
                BuzzerTone(1 , 80)
                initpoint("MagentaTengah")
                print("InitPoint : MagentaTengah")
            elif initpoint_area == "cyan":
                LED_RGB(0, 255, 255)
                LED_Status(0, 0, 1)
                BuzzerTone(1 , 80)
                initpoint("CyanTengah")
                print("InitPoint : CyanTengah")

    elif data.data == 'user':
        if button_mode == "Running":
            task = "kalibrasi"
            rospy.loginfo('Orientation set to zero!')
            yawOffset = -yawHeading#offset akan menghasilkan bilangan yaw = 0(kalibrasi yaw)
        elif button_mode == "initpoint" and initpoint_area == "cyan":
            LED_RGB(0, 255, 255)
            LED_Status(1, 0, 0)
            BuzzerTone(1 , 80)
            initpoint("CyanBelakang")
            print("InitPoint : CyanBelakang")

    elif data.data == 'start_long':
        if button_mode == "Running":
            # print("NONE")
            initpoint("Positioning_Kick_Off")
            print ("InitPoint : Positioning Kick Off")
            BuzzerTone(1 , 100)
            time.sleep(0.4)
            BuzzerTone(2 , 50)
        elif button_mode == "initpoint" and initpoint_area == "magenta":
            LED_RGB(255, 0, 255)
            LED_Status(0, 1, 1)
            BuzzerTone(2 , 50)
            initpoint("MagentaTengah_seberang")
            print ("InitPoint : MagentaTengah_seberang")
        elif button_mode == "initpoint" and initpoint_area == "cyan":
            LED_RGB(0, 255, 255)
            LED_Status(1, 1, 0)
            BuzzerTone(2 , 50)
            initpoint("CyanTengah_seberang")
            print("InitPoint : CyanTengah_seberang")

    elif data.data == 'mode_long':
        if button_mode == "Running":
            BuzzerTone(1 , 500)
            button_mode = "initpoint"
            initpoint_area = "cyan"
            rospy.loginfo("Masuk Mode Initpoint: Cyan Area")
            LED_RGB(0, 255, 255)
            LED_Status(0, 0, 0)
            # print(button_mode, initpoint_area)
        elif button_mode == "initpoint" and initpoint_area == "cyan":
            button_mode = "Running"
            initpoint_area = ""
            BuzzerTone(1 , 500)
            rospy.loginfo("Keluar Mode Initpoint : Cyan Area")
            LED_RGB(0, 255, 255)
            LED_Status(1, 1, 1)
        elif button_mode == "initpoint" and initpoint_area == "magenta":
            LED_RGB(255, 0, 255)
            LED_Status(1, 1, 0)
            BuzzerTone(2 , 50)
            initpoint("MagentaBelakang_seberang")
            print("InitPoint : MagentaBelakang_seberang")

    elif data.data == 'user_long':
        if button_mode == "Running":
            BuzzerTone(1 , 500)
            button_mode = "initpoint"
            initpoint_area = "magenta"
            rospy.loginfo("Masuk Mode Initpoint : Magenta Area")
            LED_RGB(255, 0, 255)
            LED_Status(0, 0, 0)
            # print(button_mode, initpoint_area)
        elif button_mode == "initpoint" and initpoint_area == "magenta":
            button_mode = "Running"
            initpoint_area = ""
            BuzzerTone(1 , 500)
            rospy.loginfo("Keluar Mode Initpoint : Magenta Area")
            LED_RGB(255, 0, 255)
            LED_Status(1, 1, 1)
        elif button_mode == "initpoint" and initpoint_area == "cyan":
            LED_RGB(0, 255, 255)
            LED_Status(0, 1, 1)
            BuzzerTone(2 , 50)
            initpoint("CyanBelakang_seberang")
            print("InitPoint : CyanBelakang_seberang")

def normalize_deg(angle):
    normalized = angle - 360.0 * np.floor(angle/360.0)
    return normalized

yawHeading = 0
yawOffset = 0
YawInput = 0
yaw_fixed = 0
yaw_fixed2 = 0
waktu_sebelum = 0
mode = "r"
def yaw_heading(data):
    global YawInput, yawHeading, yawOffset, yaw_fixed, yaw_fixed2, waktu_sebelum, mode
    waktu_sekarang = time.time()
    delta = waktu_sekarang - waktu_sebelum
    # yawHeading = data.data
    if yawHeading >177 and yawHeading < 180 or yawHeading <-177 and yawHeading > -180:
        yawHeading = 179
        mode = "s"
    else:
        yawHeading = data.data
        waktu_sebelum = time.time()

    if mode == "s":
        if delta <= 0.3:
            yawHeading = 179
            # print("test")
        else:
            yawHeading = data.data
            # print("done")
            mode = "r"
    else:
        yawHeading = data.data
        waktu_sebelum = time.time()

    YawInput = yawHeading + yawOffset#membuat acuan.
    yaw_fixed2 = normalize_deg(YawInput)
    yaw_fixed = YawInput
    if yaw_fixed2 > 180:
        yaw_fixed2 %= -180
    
    
    yawfixedpub.publish(yaw_fixed2)
    # print(normalize_deg(YawInput))
    # print('[heading] = %f ' % (yaw_fixed))

led_status_ball = False
status_ball = 0
def ball_state_status(data):
    global led_status_ball, status_ball
    if led_status_ball:
        if data.ball_status ==  "FOUND":
            status_ball = 1
            LED_RGB(0, 255, 0)
            LED_Status(0, 0, 0)
        elif data.ball_status == "NOTFOUND":
            status_ball = 0
            LED_RGB(255, 0, 0)
            LED_Status(1, 0, 0)
        else:
            status_ball = 0
            LED_RGB(255, 255, 255)
            LED_Status(1, 1, 1)
        print("dari fungsi ballstate: ", data.ball_status)

status_goal = 0
rotational_angle = 1
def goal_state_status(data):
    global status_goal, rotational_angle
    if data.goal_status ==  "FOUND":
        status_goal = 1
        pan_degree = pan_present * 180 / np.pi
        if pan_degree > 0:
            rotational_angle = 2
        else:
            rotational_angle = 1
        # print("haha")
    elif data.goal_status == "NOTFOUND":
        status_goal = 0
    else:
        status_goal = 0

ball_x = ball_y = ball_size = ball_x_angle = ball_y_angle = 0
def ball_coordinate(data):
    global ball_x, ball_y, ball_size, ball_x_angle, ball_y_angle
    ball_x = data.pos_x
    ball_y = data.pos_y
    ball_size = data.obj_size
    ball_x_angle = (pan_present + pan_error) * 180 / np.pi
    ball_y_angle = (tilt_present + tilt_error) * 180 / np.pi
    # print('ball_x_angle:', ball_x_angle)

goal_x = goal_y = goal_x_angle = goal_y_angle = 0
def goal_coordinate(data):
    global goal_x, goal_y, goal_x_angle, goal_y_angle
    goal_x = data.pos_x
    goal_y = data.pos_y
    goal_x_angle = (pan_present + pan_error) * 180 / np.pi
    goal_y_angle = (tilt_present + tilt_error) * 180 / np.pi
    # print('ball_x_angle:', ball_x_angle)

pan_error = tilt_error = 0
def pan_tilt_error(data):
    global pan_error, tilt_error
    pan_error = data.data[0]
    tilt_error = data.data[1]

pan_present = tilt_present = 0
def present_joint_states(data):
    global pan_present, tilt_present
    pan_present = data.position[0]
    tilt_present = data.position[1]

body_status = ""
def imu_orientation(data):
    global body_status, Task, Task_sec, Task_goto ,Task_secCF, Task_kickoff, Task_pos
    present_pitch = 0
    orientation = data.orientation
    get_orientation = [orientation.x, orientation.y, orientation.z, orientation.w]
    (r, p, y) = euler_from_quaternion(get_orientation) #yaw is not function
    alpha = 0.4
    if present_pitch == 0:
        present_pitch = p
    else:
        present_pitch = present_pitch * (1 - alpha) + p * alpha

    if present_pitch * 180 / np.pi > 60 and (Status == 'RUN' or Status == "POSITIONING_KICKOFF"):
        body_status = 'fallen_forward'
        rospy.loginfo('Get Up Forward executed!')
        Task = Task_goto = Task_sec = Task_secCF= Task_kickoff = Task_pos= 'get_up'
    elif present_pitch * 180 / np.pi < -60 and (Status == 'RUN' or Status == "POSITIONING_KICKOFF"):
        body_status = 'fallen_backward'
        rospy.loginfo('Get Up Backward executed!')
        Task = Task_goto = Task_sec = Task_secCF= Task_kickoff= Task_pos= 'get_up'

comm_status = ""
def from_udp_to_ros(data):
    global comm_status, Task
    comm_status = data.data
    if comm_status == 'rbp2':
        Task = 'walk_inarea'
    # elif comm_status == 'rbf2':
    #     Task = 'follow_ball'

#def nyoba(data):
#    return 0

yawfixedpub = rospy.Publisher('/DEWO/MotionControl/yaw', Float32, queue_size=1)
HeadPanPub = rospy.Publisher("/DEWO/MotionControl/HeadPan", Float64, queue_size=1)
CommandPub = rospy.Publisher("/DEWO/MotionControl/Command", String, queue_size=1)
HeadScanPub = rospy.Publisher("/DEWO/MotionControl/HeadScan", Bool, queue_size=1)
HeadTiltPub = rospy.Publisher("/DEWO/MotionControl/HeadTilt", Float64, queue_size=1)
ActionNumPub = rospy.Publisher("/DEWO/MotionControl/ActionNum", Int32, queue_size=1)
MotionModulePub = rospy.Publisher("/DEWO/MotionControl/MotionModule", String, queue_size=1)
WalkingParamsPub = rospy.Publisher("/DEWO/MotionControl/WalkingParams", WalkingParam, queue_size=1)
SyncWritePub = rospy.Publisher("/robotis/sync_write_item", SyncWriteItem, queue_size=1)

coba = rospy.Publisher("cobacoba", Float64, queue_size=1)
task_Pub = rospy.Publisher("/DEWO/Odometry/task", String, queue_size=1 )
OdomCommand = rospy.Publisher("/DEWO/Odometry/cmd", String, queue_size=1)
goalpospub = rospy.Publisher("/DEWO/Odometry/goal_position",Pose2D, queue_size=1)
initpointpub = rospy.Publisher("/DEWO/Odometry/initpoint", Pose2D, queue_size = 1)

TaskcomPub = rospy.Publisher("/DEWO/Communication/Task", String, queue_size=1) 
BodystatePub = rospy.Publisher("/DEWO/Communication/Bodystate", String, queue_size=1)
from_ros_to_udp = rospy.Publisher('/DEWO/Communication/from_ros_to_udp', String, queue_size=1)
comcb = rospy.Publisher('/DEWO/Communication/com_to_cb', String, queue_size=1)

trackingstatepub = rospy.Publisher('/DEWO/image_processing/deteksi_bola/tracking_state', String, queue_size=1)
positioningstatepub = rospy.Publisher('/DEWO/image_processing/deteksi_bola/positioning_state', String, queue_size=1)

# callback_ = rospy.Publisher("/DEWO/image_processing/deteksi_bola/callback_state", callbackstate, queue_size=1)


def Init():
    rospy.init_node("Process")
    rospy.Subscriber("/robotis/open_cr/yaw", Int16, yaw_heading)
    rospy.Subscriber('/robotis/open_cr/imu', Imu, imu_orientation)
    rospy.Subscriber("/robotis/open_cr/button", String, opencr_button2)
    rospy.Subscriber("/robotis/goal_joint_states", JointState, present_joint_states)

    rospy.Subscriber("/DEWO/image_processing/deteksi_bola/ball_state", BallState, ball_state_status)
    rospy.Subscriber("/DEWO/image_processing/deteksi_bola/coordinate", BallCoordinate, ball_coordinate)
    # rospy.Subscriber("/DEWO/image_processing/deteksi_bola/opponent_coordinate", opponentcoordinate, opponent_coordinate)
    # rospy.Subscriber("/DEWO/image_processing/deteksi_bola/opponent_state", opponentstate, opponent_state_status)
    rospy.Subscriber("/DEWO/image_processing/deteksi_bola/goal_coordinate", GoalCoordinate, goal_coordinate)
    rospy.Subscriber("/DEWO/image_processing/deteksi_bola/goal_state", GoalState, goal_state_status)

    rospy.Subscriber("/DEWO/MotionControl/pan_tilt_error", Float32MultiArray, pan_tilt_error)

    rospy.Subscriber("/DEWO/Communication/robotinfo", RobotInfo, RobotInfo_Callback)
    rospy.Subscriber("/DEWO/Communication/from_udp_to_ros", String, from_udp_to_ros)

    rospy.Subscriber("/DEWO/Odometry/position", Pose2D, location_Callback)
    rospy.Subscriber("/DEWO/Odometry/goalangle", Float32, goalangle_Callback)

    rospy.Subscriber("/DEWO/GameController/allstate", GameControllerState, GCState_Callback)
    time.sleep(1)

robotstatus = "UTAMA"
ballsize_treshold = 1000 #untuk menentukan robot status
ballsize_range = 2400
reason = ""
def Com_decision():
    global robotstatus, reason
    '''
    *)Taskcom, ball_size, yaw_fixed2, body_status --> atribut sendiri
    *)sisanya atribut kawan sesuai indeks
    *)indeks2 khusus pemain bertahan
    '''
    if JERSEY == "CYAN":
        setpoint = 0#arah serang ke magenta
    else:
        setpoint = 180
    if koneksi == "Y":
        #gerbang menentukan status robot
        if ball_size >= ballsize_treshold: 
            if ball_size > ballarea_1 and ball_size > ballarea_2 and ball_size > ballarea_3:#filter terdekat
                robotstatus = "UTAMA"
                reason = "terdekat"
            else:
                robotstatus = "SECOND"
                reason = "terdekat"

            #gerbang penentuan
            if ball_size >= ballsize_range:
                if ballarea_1 >= ballsize_range: #filter menguntungkan
                    reason = "filter menguntungkan1"
                    print(yaw_fixed2, "", posw_1)
                    if np.abs(np.abs(yaw_fixed2) - setpoint) < np.abs(np.abs(posw_1) - setpoint):
                        robotstatus = "UTAMA"
                    else:
                        robotstatus = "SECOND"
                else:
                    pass  #GA MASUK DALAM RANGE
                #-------------------------------------------
                if ballarea_2 >= ballsize_range:
                    reason = "filter menguntungkan2"
                    print(yaw_fixed2, "", posw_2)
                    if np.abs(np.abs(yaw_fixed2) - setpoint) < np.abs(np.abs(posw_2) - setpoint):
                        robotstatus = "UTAMA"
                    else:
                        robotstatus = "SECOND"
                else:
                    pass #GA MASUK DALAM RANGE
                #-------------------------------------------
                if ballarea_3 >= ballsize_range:
                    reason = "filter menguntungkan3"
                    print(yaw_fixed2, "", posw_3)
                    if np.abs(np.abs(yaw_fixed2) - setpoint) < np.abs(np.abs(posw_3) - setpoint):
                        robotstatus = "UTAMA"
                    else:
                        robotstatus = "SECOND"
                else:
                    pass #GA MASUK DALAM RANGE
                #------------------------------------------- 
            else:
                robotstatus = "GOTO"
                reason = "tidak dalam treshold"

        #MENGINTRUPSI FILTER TERDEKAT ATAU MENGUNTUNGKAN
        #filter menendang
        if Taskcom == "kck":
            robotstatus = "SECOND"
            reason = "robot 2 menendang"
        elif task_3 == "kck":
            time.sleep(2)#bagian ini hanya contoh, ini hanya dipakai 2 robot penyerang dan ((bertahan)(2penyerang))
            robotstatus = "UTAMA"
            reason = "robot 3 menendang"
        elif task_1 == "kck":#pemain bertahan
            time.sleep(3)
            reason = "robot 1 menendang"
            robotstatus = "GOTO"

        #filter ahhh
        if task_1 == "posbl" or task_3 == "posbl":
            robotstatus = "SECOND"
            reason = "filter psbl"

        #gerbang agar tidak bertabrakan saat mau menendang(keduanya UTAMA)
        if (Taskcom == "posbl" or Taskcom == "rt"):
            if task_3 == "posbl" and Taskcom == "rt":
                robotstatus = "SECOND"
                reason = "robot 2 rotate // robot 3 posball"
            elif task_3 == "rt" and Taskcom == "posbl":
                robotstatus =  "UTAMA"
                reason = "robot 3 rotate // robot 2 posball"
            elif task_3 == Taskcom:#ktika status mereka sama" rt atau posbl
                if np.abs(np.abs(yaw_fixed2) - setpoint) < np.abs(np.abs(posw_3) - setpoint):
                    robotstatus = "UTAMA"
                else:
                    robotstatus = "SECOND"
                reason = "robot 2 dan 3 sama sama rt atau posball"
            else:
                robotstatus = "UTAMA"
                reason = "robot 2 rotate atau posbl  sendiri"

        #gerbang agar tidak keduanya second
        if (Taskcom == "fb_sec" or Taskcom == "stp_sec" or Taskcom == "rt_sec") and (task_3 == "fb_sec" or task_3 == "stp_sec" or task_3 == "rt_sec"):
            if ballarea_3 >= ballsize_range:
                print (yaw_fixed2, "", posw_3)
                if np.abs(np.abs(yaw_fixed2) - setpoint) < np.abs(np.abs(posw_3) - setpoint):
                    robotstatus = "UTAMA"
                    reason = "filter_menguntungkan2 || keduanya second"
                else:
                    robotstatus = "SECOND"
                    reason = "filter_menguntungkan2 || keduanya second"
            else:
                pass #GA MASUK DALAM RANGE
        #filter jatuh
        if body_status == "fallen_forward" or body_status == "fallen_backward":
            robotstatus = "SECOND"
            reason = "robot 2 jatuh"
        elif fall_1 == "Y":
            robotstatus = "UTAMA"#bagian ini hanya contoh, ini hanya dipakai 2 robot penyerang
            reason = "robot 1 jatuh"
        elif fall_3 == "Y":
            reason = "robot 3 jatuh"
            robotstatus = "UTAMA"
    else:
        robotstatus = "UTAMA"
        rospy.logwarn("Com_decicion: tidak menerima data...")
    rospy.logwarn('Com_decision: %s', robotstatus)
    rospy.logwarn("-> Reason: %s", reason)
    
    # return robotstatus

'''
if detect_4 == "Y":
           ###################################################
            #filter terdekat || antar robot penyerang
            if ball_size > ballarea_3:
                robotstatus = "UTAMA"
                reason = "terdekat || CB AKTIF"
                com_to_cb = "mid mode"
                comcb.publish(com_to_cb)
                
            else:
                robotstatus = "SECOND_CF"
                reason = "terdekat || CB AKTIF"
                com_to_cb = "stkr mode"
                comcb.publish(com_to_cb)
            #filter terdekat || robot penyerang dan bertahan
            if ball_size >= ballsize_treshold:
                if ball_size > ballsize_range:
                    if ball_size > ballarea_4:#filter terdekat
                        robotstatus = "UTAMA"
                        reason = "terdekat || CB AKTIF"
                        com_to_cb = "mid mode"
                        comcb.publish(com_to_cb)
                    else:
                        robotstatus = "SECOND"
                        reason = "terdekat || CB AKTIF"
            else:
                robotstatus = "GOTO"
                reason = "tidak dalam treshold || CB AKTIF"
            #MENGINTRUPSI FILTER TERDEKAT ATAU MENGUNTUNGKAN
            #filter menendang
            if Taskcom == "kck":
                robotstatus = "SECOND"
                reason = "robot 2 menendang || CB AKTIF"
            elif task_4 == "kck":
                time.sleep(2)#bagian ini hanya contoh, ini hanya dipakai 2 robot penyerang dan ((bertahan)(2penyerang))
                robotstatus = "UTAMA"
                reason = "robot 4 menendang || CB AKTIF"
                com_to_cb = "mid mode"
                comcb.publish(com_to_cb)
            #filter ahhh
            if task_4 == "posbl":
                robotstatus = "SECOND"
                reason = "filter psbl || CB AKTIF"
            #gerbang agar tidak bertabrakan saat mau menendang(keduanya UTAMA)
            if (Taskcom == "posbl" or Taskcom == "rt"):
                if task_4 == "posbl" and Taskcom == "rt":
                    robotstatus = "SECOND"
                    reason = "robot 2 rotate // robot 4 posball || CB AKTIF"
                elif task_4 == "rt" and Taskcom == "posbl":
                    robotstatus =  "UTAMA"
                    reason = "robot 34rotate // robot 2 posball || CB AKTIF"
                    com_to_cb = "mid mode"
                    comcb.publish(com_to_cb)
                elif task_4 == Taskcom:#ktika status mereka sama" rt atau posbl
                    if np.abs(np.abs(yaw_fixed2) - setpoint) < np.abs(np.abs(posw_4) - setpoint):
                        robotstatus = "UTAMA"
                        com_to_cb = "mid mode"
                        comcb.publish(com_to_cb)
                    else:
                        robotstatus = "SECOND"
                    reason = "robot 2 dan 4 sama sama rt atau posball || CB AKTIF"
                else:
                    robotstatus = "UTAMA"
                    reason = "robot 2 rotate atau posbl  sendiri || CB AKTIF"
                    com_to_cb = "mid mode"
                    comcb.publish(com_to_cb)
            #gerbang agar tidak keduanya second
            if (Taskcom == "fb_sec" or Taskcom == "stp_sec" or Taskcom == "rt_sec") and (task_4 == "fb_sec" or task_4 == "stp_sec" or task_4 == "rt_sec"):
                if ballarea_4 >= ballsize_range:
                    print (yaw_fixed2, "", posw_4)
                    if np.abs(np.abs(yaw_fixed2) - setpoint) < np.abs(np.abs(posw_4) - setpoint):
                        robotstatus = "UTAMA"
                        reason = "filter_menguntungkan2 || keduanya second || CB AKTIF"
                        com_to_cb = "mid mode"
                        comcb.publish(com_to_cb)
                    else:
                        robotstatus = "SECOND"
                        reason = "filter_menguntungkan2 || keduanya second || CB AKTIF"
                else:
                    pass #GA MASUK DALAM RANGE
            #filter jatuh
            if body_status == "fallen_forward" or body_status == "fallen_backward":
                robotstatus = "SECOND"
                reason = "robot 2 jatuh || CB AKTIF"
            elif fall_4 == "Y":
                robotstatus = "UTAMA"#bagian ini hanya contoh, ini hanya dipakai 2 robot penyerang
                reason = "robot 4 jatuh|| CB AKTIF"
                com_to_cb = "mid mode"
                comcb.publish(com_to_cb)
           ###################################################
#-------------------------------------------------------------------------------------------------------------------------------
        else:
            #gerbang menentukan status robot
            if ball_size >= ballsize_treshold: 
                if ball_size > ballarea_1 and ball_size > ballarea_2 and ball_size > ballarea_3:#filter terdekat
                    robotstatus = "UTAMA"
                    reason = "terdekat"
                else:
                    robotstatus = "SECOND"
                    reason = "terdekat"

                #gerbang penentuan
                if ball_size >= ballsize_range:

                    if ballarea_1 >= ballsize_range: #filter menguntungkan
                        reason = "filter menguntungkan1"
                        print(yaw_fixed2, "", posw_1)
                        if np.abs(np.abs(yaw_fixed2) - setpoint) < np.abs(np.abs(posw_1) - setpoint):
                            robotstatus = "UTAMA"
                        else:
                            robotstatus = "SECOND"
                    else:
                        pass  #GA MASUK DALAM RANGE
                    #-------------------------------------------
                    if ballarea_2 >= ballsize_range:
                        reason = "filter menguntungkan2"
                        print(yaw_fixed2, "", posw_2)
                        if np.abs(np.abs(yaw_fixed2) - setpoint) < np.abs(np.abs(posw_2) - setpoint):
                            robotstatus = "UTAMA"
                        else:
                            robotstatus = "SECOND"
                    else:
                        pass #GA MASUK DALAM RANGE
                    #-------------------------------------------
                    if ballarea_3 >= ballsize_range:
                        reason = "filter menguntungkan3"
                        print(yaw_fixed2, "", posw_3)
                        if np.abs(np.abs(yaw_fixed2) - setpoint) < np.abs(np.abs(posw_3) - setpoint):
                            robotstatus = "UTAMA"
                        else:
                            robotstatus = "SECOND"
                    else:
                        pass #GA MASUK DALAM RANGE
                    #-------------------------------------------
                else:
                    pass 
            else:
                robotstatus = "GOTO"
                reason = "tidak dalam treshold"

            #MENGINTRUPSI FILTER TERDEKAT ATAU MENGUNTUNGKAN
            #filter menendang
            if Taskcom == "kck":
                robotstatus = "SECOND"
                reason = "robot 2 menendang"
            elif task_3 == "kck":
                time.sleep(2)#bagian ini hanya contoh, ini hanya dipakai 2 robot penyerang dan ((bertahan)(2penyerang))
                robotstatus = "UTAMA"
                reason = "robot 3 menendang"
            elif task_1 == "kck":#pemain bertahan
                time.sleep(3)
                reason = "robot 1 menendang"
                robotstatus = "GOTO"

            #filter ahhh
            if task_1 == "posbl" or task_3 == "posbl":
                robotstatus = "SECOND"
                reason = "filter psbl"

            #gerbang agar tidak bertabrakan saat mau menendang(keduanya UTAMA)
            if (Taskcom == "posbl" or Taskcom == "rt"):
                if task_3 == "posbl" and Taskcom == "rt":
                    robotstatus = "SECOND"
                    reason = "robot 2 rotate // robot 3 posball"
                elif task_3 == "rt" and Taskcom == "posbl":
                    robotstatus =  "UTAMA"
                    reason = "robot 3 rotate // robot 2 posball"
                elif task_3 == Taskcom:#ktika status mereka sama" rt atau posbl
                    if np.abs(np.abs(yaw_fixed2) - setpoint) < np.abs(np.abs(posw_3) - setpoint):
                        robotstatus = "UTAMA"
                    else:
                        robotstatus = "SECOND"
                    reason = "robot 2 dan 3 sama sama rt atau posball"
                else:
                    robotstatus = "UTAMA"
                    reason = "robot 2 rotate atau posbl  sendiri"

            #gerbang agar tidak keduanya second
            if (Taskcom == "fb_sec" or Taskcom == "stp_sec" or Taskcom == "rt_sec") and (task_3 == "fb_sec" or task_3 == "stp_sec" or task_3 == "rt_sec"):
                if ballarea_3 >= ballsize_range:
                    print (yaw_fixed2, "", posw_3)
                    if np.abs(np.abs(yaw_fixed2) - setpoint) < np.abs(np.abs(posw_3) - setpoint):
                        robotstatus = "UTAMA"
                        reason = "filter_menguntungkan2 || keduanya second"
                    else:
                        robotstatus = "SECOND"
                        reason = "filter_menguntungkan2 || keduanya second"
                else:
                    pass #GA MASUK DALAM RANGE
            #filter jatuh
            if body_status == "fallen_forward" or body_status == "fallen_backward":
                robotstatus = "SECOND"
                reason = "robot 2 jatuh"
            elif fall_1 == "Y":
                robotstatus = "UTAMA"#bagian ini hanya contoh, ini hanya dipakai 2 robot penyerang
                reason = "robot 1 jatuh"
            elif fall_3 == "Y":
                reason = "robot 3 jatuh"
                robotstatus = "UTAMA"
    else:
        robotstatus = "UTAMA"
        rospy.logwarn("Com_decicion: tidak menerima data...")
    rospy.logwarn('Com_decision: %s', robotstatus)
    rospy.logwarn("-> Reason: %s", reason)
    
    # return robotstatus'''
    

id_1 = id_2 = id_3 = id_4 = id_5 = id_mid = id_stkr =  0
fall_1 = fall_2 = fall_3 =fall_4 =fall_5 = fall_mid = fall_stkr= ""
task_1 = task_2 = task_3 =task_4 =task_5 =task_mid=task_stkr= ""
posx_1 = posx_2 = posx_3 =posx_4 =posx_5 =posx_mid=posx_stkr= 0
posy_1 = posy_2 = posy_3 =posy_4 =posy_5 =posy_mid=posy_stkr= 0
posw_1 = posw_2 = posw_3 =posw_4 =posw_5 =posw_mid=posw_stkr=  0
detect_1 = detect_2 = detect_3 =detect_4 =detect_5 =detect_mid=detect_stkr= ""
ballarea_1 = ballarea_2 = ballarea_3 =ballarea_4 =ballarea_5 =ballarea_mid=ballarea_stkr= 0
goalorient_1 = goalorient_2 = goalorient_3 =goalorient_4 =goalorient_5 =goalorient_mid=goalorient_stkr= 0
koneksi = ""
def RobotInfo_Callback(data):
    global koneksi, id_1,fall_1, task_1, posx_1, posy_1, posw_1, detect_1, ballarea_1, id_2,fall_2, task_2, posx_2, posy_2, posw_2, detect_2, ballarea_2, id_3,fall_3, task_3, posx_3, posy_3, posw_3, detect_3, ballarea_3, goalorient_1, goalorient_2, goalorient_3, id_4,fall_4, task_4, posx_4, posy_4, posw_4, detect_4, ballarea_4, goalorient_4, id_5,fall_5, task_5, posx_5, posy_5, posw_5, detect_5, ballarea_5, goalorient_5, id_mid,fall_mid, task_mid, posx_mid, posy_mid, posw_mid, detect_mid, ballarea_mid, goalorient_mid, id_stkr,fall_stkr, task_stkr, posx_stkr, posy_stkr, posw_stkr, detect_stkr, ballarea_stkr, goalorient_stkr
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
    if detect == "mid mode":
        id_mid = id
        fall_mid = fall
        task_mid = task
        posx_mid = posx
        posy_mid = posy
        posw_mid = posw
        detect_mid = detect
        ballarea_mid = ballarea
        goalorient_mid = goalorient
    elif detect == "stkr mode":
        id_stkr = id
        fall_stkr = fall
        task_stkr = task
        posx_stkr = posx
        posy_stkr = posy
        posw_stkr = posw
        detect_stkt = detect
        ballarea_stkr = ballarea
        goalorient_stkr = goalorient
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
    elif id == 4:
        id_4 = id
        fall_4 = fall
        task_4 = task
        posx_4 = posx
        posy_4 = posy
        posw_4 = posw
        detect_4 = detect
        ballarea_4 = ballarea
        goalorient_4 = goalorient
    elif id == 5:
        id_5 = id
        fall_5 = fall
        task_5 = task
        posx_5 = posx
        posy_5 = posy
        posw_5 = posw
        detect_5 = detect
        ballarea_5 = ballarea
        goalorient_5 = goalorient
    elif id == 100:
        id_1 = id_2 = id_3 = 0
        fall_1 = fall_2 = fall_3 =fall_4 =fall_5 = ""
        task_1 = task_2 = task_3 =task_4 =task_5 = ""
        posx_1 = posx_2 = posx_3 =posx_4 =posx_5 = 0
        posy_1 = posy_2 = posy_3 =posy_4 =posy_5 = 0
        posw_1 = posw_2 = posw_3 =posw_4 =posw_5 =  0
        detect_1 = detect_2 = detect_3 =detect_4 =detect_5 = ""
        ballarea_1 = ballarea_2 = ballarea_3 =ballarea_4 =ballarea_5 = 0
        goalorient_1 = goalorient_2 = goalorient_3 =goalorient_4 =goalorient_5 = 0
        koneksi = "N"

X_global = Y_global = W_global = 0
def location_Callback(data):
    global X_global, Y_global, W_global
    X_global = data.x
    Y_global = data.y
    W_global = data.theta

goal_orientation = 0
def goalangle_Callback(data):
    global goal_orientation, Task
    goal_orientation = data.data
    if Task != 'positioning':
        if Y_global > 200 and Y_global < 400:
            if X_global < 100:
                goal_orientation = 180
            elif X_global > 800:
                goal_orientation = 0
            else:
                pass
        else:
            pass
    else:
        pass
    rospy.logwarn('GOAL ORIENTATION: %d', goal_orientation)

    
def Goalposition(jersey):
    if jersey == 'CYAN':
        X = 900
        Y = 300
    else :
        X = 0
        Y = 300
    Pointposition(X,Y)

def Pointposition(x,y):
    pos = Pose2D()
    pos.x = x
    pos.y = y
    pos.theta = 0
    goalpospub.publish(pos)

def Odom_Command(command):
    data = String()
    data.data = command
    OdomCommand.publish(data)

def OdomStart():
    Odom_Command('start')

def OdomStop():
    Odom_Command('stop')

def MotionMode(mode):
    data = String()
    data.data = mode
    MotionModulePub.publish(data)
    time.sleep(0.1)


def Motion_InitWalking():
    MotionMode('walk')
    time.sleep(0.5)


def Motion_InitAction():
    MotionMode('action')
    time.sleep(0.5)


def Motion_InitHead():
    MotionMode('head')
    time.sleep(0.3)


def MotionCommand(cmd):
    data = String()
    data.data = cmd
    CommandPub.publish(data)
    time.sleep(0.1)


def Motion_InitPose():
    MotionCommand('reset')
    time.sleep(4)


def Motion_Start():
    MotionCommand('start')


def Motion_Stop():
    MotionCommand('stop')
    time.sleep(0.5)


def walking(x, y, o, t):
    walkingParams_data = WalkingParam()
    walkingParams_data.x_move_amplitude = x
    walkingParams_data.y_move_amplitude = y
    walkingParams_data.angle_move_amplitude = o
    walkingParams_data.period_time = t
    WalkingParamsPub.publish(walkingParams_data)
    time.sleep(0.01)


def Motion_ActionNum(num):
    ActionNumPub.publish(num)
    time.sleep(5.0)


def Motion_HeadScan(enable):
    HeadScanPub.publish(enable)
    time.sleep(0.1)


def Motion_HeadControl(pan, tilt):
    HeadPanPub.publish(pan)
    time.sleep(0.1)
    HeadTiltPub.publish(tilt)
    time.sleep(0.1)


def LED_Status(r, g, b):
    data = SyncWriteItem()
    data.joint_name = ['open-cr']
    data.item_name = 'LED'
    data.value = [r + g * 2 + b * 4]
    SyncWritePub.publish(data)
    time.sleep(0.01)


def LED_RGB(r, g, b):
    data = SyncWriteItem()
    data.joint_name = ['open-cr']
    data.item_name = 'LED_RGB'
    generate_color = (b & 0x1F) << 10 | (g & 0x1F) << 5 | (r & 0x1F)
    data.value = [generate_color]
    SyncWritePub.publish(data)
    time.sleep(0.01)


def Buzzer(freq):
    data = SyncWriteItem()
    data.joint_name = ['open-cr']
    data.item_name = 'buzzer'
    data.value = [freq]
    SyncWritePub.publish(data)
    time.sleep(0.01)

buzzerCount = 0
buzzerPeriod = 0
buzzerState = 0

def BuzzerTone(count, period):
    global buzzerCount
    global buzzerPeriod
    buzzerCount = count
    buzzerPeriod = period


def Ticks():
    return int(time.time() * 1000)

rateError = cumError = error = lastError = 0.0
def calculate_pid(val, set_point):
    global lastError, cumError, rateError #, prev_time_
    kp = 8.0
    ki = 5.0
    kd = 15.0

    # curr_time = rospy.get_rostime() #get current time
    # dur = curr_time - prev_time_
    # delta_time = dur.nsecs * 0.000000001 + dur.secs #compute time elapsed from previous computation      
    
    error = set_point - val                          # determine error
    # cumError += error * delta_time                  # compute integral
    rateError = (error - lastError) #/delta_time      # compute derivative

    out = kp*error + kd*rateError #+ ki*cumError      #PID output               

    lastError = error                                 #remember current error
    # prev_time_ = curr_time                          #remember current time
    return out

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

def scale_value(val, src, trg):
    return val / src * trg

def map_value2(source_val, source_min, source_max, target_min, target_max):
    return target_min + scale_value(source_val - source_min, source_max - source_min, target_max - target_min)

def map_value(source_val, source_min, source_max, target_min, target_max):
	source_val = (float) (min_value(source_val, max_value(source_min, source_max)))
	source_val = (float) (max_value(source_val, min_value(source_min, source_max)))
	return (float) (target_min + ((source_val - source_min) * ((target_max - target_min) / (source_max - source_min))))


def filter_value(val, coeff):
    out = 0.0
    c = coeff
    out = ((out * c) + val) / (c + 1)
    return out * 100.0

def limit_value(val, min, max):
    if val < min:
        val = min
    elif val > max:
        val = max
    return val

def start_tracking(state):
    if state is True:
        trackingstatepub.publish('start')
    else:
        trackingstatepub.publish('stop')

def start_positioning(state):
    if state is True:
        positioningstatepub.publish('start')
    else:
        positioningstatepub.publish('stop')

ball_was_lost = 0
def check_ball_lost(threshold):
    global ball_was_lost
    if status_ball == 0:
        ball_was_lost += 1
        if ball_was_lost >= threshold:
            return 1
    else:
        ball_was_lost = 0
        return 0
    # print(ball_was_lost)

goal_was_lost = 0
def check_goal_lost(threshold):
    global goal_was_lost
    if status_goal == 0:
        goal_was_lost += 1
        if goal_was_lost >= threshold:
            return 1
    else:
        goal_was_lost = 0
        return 0
gap = 0
def Heading_Gap(goal, yaw):
    global gap
    if (goal > 0 and yaw < 0) or (goal < 0 and yaw > 0):
        goal_to_0 = np.abs(goal)
        yaw_to_0 = np.abs(yaw)
        goal_to_180 = 180 - goal_to_0
        yaw_to_180 = 180 - yaw_to_0
        gap_to_0 = goal_to_0 + yaw_to_0
        gap_to_180 = goal_to_180 + yaw_to_180
        # print(goal_to_0, yaw_to_0, goal_to_180, yaw_to_180, gap_to_0, gap_to_180)
        if gap_to_180 < gap_to_0:
            if yaw_fixed2 > 0:
                gap = gap_to_180
                print("kiri, 180")
            else:
                gap = -gap_to_180
                print("kanan, 180")
        elif gap_to_180 > gap_to_0:
            if yaw_fixed2 > 0:
                gap = -gap_to_0
                print("kanan, 0")
            else:
                gap = gap_to_0
                print("kiri, 0")
    else:
        gap = (goal - yaw)
    # print(f"point = {gap}")
    return gap


a_move_max = 10.0
x_move_max = 17.0
tilt_min = -61
def walk_follow_head(pan, tilt):
    pan_degree = pan * 180 / np.pi
    tilt_degree = tilt * 180 / np.pi
    a_move = map_value2(pan_degree, -89.0, 89.0, -a_move_max, a_move_max)
    x_move = map_value2(np.abs(a_move), 0.0, a_move_max, x_move_max, 0.)#maksimmal melangkah kedepan
    x_move_tilt = map_value2(tilt_degree - tilt_min, 30.0, 0.0, x_move, -0.008)
    y_move = 0.0
    # coba.publish(x_move_tilt * 0.001)
    # coba.publish(y_move)
    # coba.publish(a_move)
    return x_move_tilt * 0.001, y_move, a_move

in_position = False
determine_equation = 0
m = c = 0
posmod = "move"
def walk_to_position(x, y, o):
    global Status, in_position,determine_equation, m,c, posmod
    distance = math.sqrt((x - X_global)**2 + (y-Y_global)**2 )
    in_position = False
    x_move = y_move_pid = a_move_param = 0
    Pointposition(x,y)
    gap = Heading_Gap(goal_orientation, yaw_fixed2)
    # print (gap, goal_orientation, distance)
    if posmod == "move":
        if X_global < x+6 and X_global > x-6:
            posmod = "netral"
            x_move = -4.0
            y_move_pid = 0.0
            a_move_param = 0.0
        else:
            #xmove
            a_move = map_value(gap, -180, 180, -15.0, 15.0)
            x_move = map_value(np.abs(a_move), 0.0, a_move_max, x_move_max, 0.)
            x_move = map_value(distance, 30.0, 0.0, x_move, -0.008)
            #a_move
            a_move_param = map_value(gap, -180, 180, -60.0, 60.0)
            a_move_param = limit_value(a_move_param, -5.0, 5.0)
            #ymove
            y_move = calculate_pid((float)(gap),  0.0)
            y_move_pid = (float)(y_move) / 30.0
            y_move_pid = limit_value(y_move_pid, -15.0, 15.0)
            # print(x_move, y_move, a_move_param, a_move)
    elif posmod == "netral":
        x_move = -4.0
        y_move_pid = 0.0

        gap_netral = Heading_Gap(o, yaw_fixed2)
        a_move_param = map_value2(gap_netral, -180, 180, -4, 4)
        a_move_param = limit_value(a_move_param, -4, 4)
        # if yaw_fixed2 < o:
        #     a_move_param = 3#map_value(yaw_fixed2 - o, 0, 180, 0, 90) 
        # elif yaw_fixed2 > o:
        #     a_move_param = -3#map_value(yaw_fixed2 - o, 0, -180, 0, -90) 
        # a_move_param = limit_value(a_move_param, -5.0, 5.0)
        if yaw_fixed2 > o - 6 and yaw_fixed2 < o + 6:
            in_position = True
            posmod = "move"
            print ("in posistion")

    return (x_move * 0.001), -(y_move_pid*0.001), a_move_param, in_position

heading_gap2 = delta_tilt = 0.0
def walk_rotate2(heading):
    global heading_gap2, delta_tilt
    x_move = y_move = 0.0
    pan_degree = pan_present * 180.0 / np.pi
    tilt_degree = tilt_present * 180.0 / np.pi
    
    heading_gap2 =Heading_Gap(heading, yaw_fixed2)
    delta_tilt = -53.0 - tilt_degree

    if delta_tilt > 0.0:
        x_move = map_value(delta_tilt, 0.0, 20.0, 0.0, -6.0)
    else:
        x_move = map_value(delta_tilt, -20.0, 0.0, 6.0, 0.)

    y_move = calculate_pid((float)(heading_gap2),  0.0)
    y_move_pid = (float)(y_move) / 30.0
    y_move_pid = limit_value(y_move_pid, -15.0, 15.0)
    # print('heading_gap', heading_gap2, 'y_move_pid', y_move_pid, 'y_move', y_move)

    a_move = map_value2(pan_degree, -10.0, 10.0, -6.0, 6.0)
    a_move = limit_value(a_move, -6, 6)

    return x_move * 0.001, y_move_pid * 0.001, a_move

heading_gap2 = delta_tilt = 0.0
def second_rotate(heading, distance):
    global heading_gap2, delta_tilt
    x_move = y_move = 0.0
    pan_degree = pan_present * 180.0 / np.pi
    tilt_degree = tilt_present * 180.0 / np.pi
    
    heading_gap2 =Heading_Gap(heading, yaw_fixed2)
    delta_tilt = distance - tilt_degree

    if delta_tilt > 0.0:
        x_move = map_value(delta_tilt, 0.0, 20.0, 0.0, -20.0)
    else:
        x_move = map_value(delta_tilt, -20.0, 0.0, 20.0, 0.)

    y_move = calculate_pid((float)(heading_gap2),  0.0)
    y_move_pid = (float)(y_move) / 30.0
    y_move_pid = limit_value(y_move_pid, -15.0, 15.0)
    # print('heading_gap', heading_gap2, 'y_move_pid', y_move_pid, 'y_move', y_move)

    a_move = map_value2(pan_degree, -10.0, 10.0, -6.0, 6.0)
    a_move = limit_value(a_move, -6, 6)

    return x_move * 0.001, y_move_pid * 0.001, a_move

def second_orientation():
    global goal_orientation
    if task_1 == "rt" or task_1 == "posbl" :
        orient_base = goalorient_1
    elif task_2 == "rt" or task_2 == "posbl" :
        orient_base = goalorient_2
    elif task_3 == "rt" or task_3 == "posbl" :
        orient_base = goalorient_3
    else:
        orient_base = 0
    
    if yaw_fixed2 < orient_base:
        goal_orientation = orient_base - 90
    else:
        goal_orientation = orient_base + 90
def takeball_opportunity():
    if task_1 == "stp_sec" or task_2 == "stp_sec" or task_3 == "stp_sec" or task_1 == "rt_sec" or task_2 == "rt_sec" or task_3 == "rt_sec":
        opportunity = 'n'
    else:
        opportunity = 'y'
    rospy.loginfo("Take ball opportunity: %s", opportunity)
    return opportunity

heading_gap22 = delta_tilt2 = 0.0
def walk_rotate22(heading):
    global heading_gap22, delta_tilt2
    x_move = y_move = 0.0
    pan_degree = pan_present * 180.0 / np.pi
    tilt_degree = tilt_present * 180.0 / np.pi
    
    heading_gap2 =Heading_Gap(heading, yaw_fixed2)
    delta_tilt = -50.0 - tilt_degree

    if delta_tilt > 0.0:
        x_move = map_value(delta_tilt2, 0.0, 20.0, 0.0, -6.0)
    else:
        x_move = map_value(delta_tilt2, -20.0, 0.0, 6.0, 0.)

    y_move = calculate_pid((float)(heading_gap2),  0.0)
    y_move_pid = (float)(y_move) / 30.0
    y_move_pid = limit_value(y_move_pid, -15.0, 15.0)
    # print('heading_gap', heading_gap2, 'y_move_pid', y_move_pid, 'y_move', y_move)

    a_move = map_value2(pan_degree, -10.0, 10.0, -6.0, 6.0)
    a_move = limit_value(a_move, -6, 6)

    return x_move * 0.001, y_move_pid * 0.001, a_move

heading_gap3 = delta_tilt = 0.0
def walk_rotate_goal(heading):
    global heading_gap3, delta_tilt
    x_move = y_move = 0.0
    pan_degree = pan_present * 180.0 / np.pi
    tilt_degree = tilt_present * 180.0 / np.pi
    
    heading_gap3 = heading - yaw_fixed2#yaw_fixed
    delta_tilt = -35.0 - tilt_degree# treshold 0 adalah-35.0

    if delta_tilt > 0.0:#<tilt -35, berarti terlalu dkat
        x_move = map_value(delta_tilt, 0.0, 20.0, 0.0, -10.0)
    else:
        x_move = map_value(delta_tilt, -20.0, 0.0, 10.0, 0.)

    y_move = calculate_pid((float)(heading_gap3),  0.0)
    y_move_pid = (float)(y_move) / 30.0
    y_move_pid = limit_value(y_move_pid, -15.0, 15.0)
    # print('heading_gap', heading_gap2, 'y_move_pid', y_move_pid, 'y_move', y_move)


    a_move = map_value(pan_degree, -10.0, 10.0, -6.0, 6.0)

    return x_move * 0.001, y_move_pid * 0.001, a_move

y_move_max = 25.0
heading_gap = 0.0
def walk_rotate(heading, y_move_change):
    global heading_gap
    is_done = False
    # if heading > 0:
    #     heading_gap = heading - yaw_fixed
    # else:
    #     heading_gap = heading + yaw_fixed
    heading_gap = heading - yaw_fixed2#yaw_fixed
    y_move = 0.0

    if y_move_change is True:
        if heading_gap < 0.0:
            y_move = y_move_max
        else:
            y_move = -y_move_max
    
    # a_move = map_value(heading_gap, -180.0, 180.0, -10.0, 10.0)
    a_move = calculate_pid((float)(heading_gap),  0.0)
    a_move_pid = (float)(a_move) / 100.0
    a_move_pid = limit_value(a_move_pid, -10.0, 10.0)

    x_move = -2.0
    if heading_gap == 0.0:
        is_done = True
    else:
        is_done = False
    # print("heading_gap", heading_gap, "yaw_fixed", yaw_fixed, "a_move", a_move_pid)
    return x_move * 0.001, y_move * 0.001, -a_move_pid, is_done

x_move_dribble_max = 30.0
x_move_dribble_min = -10.0
def walk_dribble(heading):
    global heading_gap
    pan_degree = pan_present * 180.0 / np.pi

    x_move = 0
    if np.abs(pan_degree) < 15.0:
        x_move = map_value(np.abs(pan_degree), 0.0, 15.0, x_move_dribble_max, 0.)
    else:
        x_move = map_value(np.abs(pan_degree), 15.0, 45.0, 0.0, x_move_dribble_min)

    y_move = 0
    ry_min = -10
    ry_max = -20
    ly_min = 10
    ly_max = 20
    if pan_degree < -6.0:
        y_move = map_value(pan_degree, -25.0, -6.0, ry_max, ry_min)
    elif pan_degree > 6.0:
        y_move = map_value(pan_degree, 6.0, 25.0, ly_min, ly_max)

    heading_gap = Heading_Gap(heading, yaw_fixed2)#normalize_deg(yaw_fixed2 - heading)
    a_move = map_value(heading_gap, 0.0, 360.0, -8.0, 8.0)

    return x_move * 0.001, y_move * 0.001, a_move

def walk_in_ball_area(target_pan, target_tilt, heading):
    global heading_gap

    x_move = 0
    if np.abs(ball_x_angle) < 10.0:
        x_move = map_value(np.abs(ball_x_angle), 0.0, 10.0, 8.0, 0.)
    else:
        x_move = map_value(np.abs(ball_x_angle), 10.0, 35.0, 0.0, -10.0)

    y_move = 0
    ry_min = -10
    ry_max = -20
    ly_min = 10
    ly_max = 20
    if ball_x_angle < -5.0:
        y_move = map_value(ball_x_angle, -30.0, -5.0, ry_max, ry_min)
    elif ball_x_angle > 5.0:
        y_move = map_value(ball_x_angle, 5.0, 30.0, ly_min, ly_max)

    heading_gap = Heading_Gap(heading, yaw_fixed2) #normalize_deg(yaw_fixed2 - heading)
    a_move = map_value(heading_gap, 0.0, 360.0, -5.0, 5.0)

    return x_move * 0.001, y_move * 0.001, a_move

def walk_in_ball_area2(tilt_target, pan_target, heading):
    global heading_gap

    x_move = 0
    tilt_degree = tilt_present * 180.0 / np.pi
    pan_degree = pan_present * 180.0 / np.pi
    # delta_tilt = -60.0 - tilt_degree
    delta_tilt = tilt_target - tilt_degree

    if delta_tilt > 0.0:
        x_move = map_value(delta_tilt, 0.0, 10.0, 0.0, -10.0)
    else:
        x_move = map_value(delta_tilt, -10.0, 0.0, 10.0, 0.)
    # if np.abs(ball_x_angle) < 10.0:
    #     x_move = map_value(np.abs(ball_x_angle), 0.0, 10.0, 8.0, 0.)
    # else:
    #     x_move = map_value(np.abs(ball_x_angle), 10.0, 35.0, 0.0, -10.0)

    y_move = 0
    ry_min = -10
    ry_max = -20
    ly_min = 10
    ly_max = 20
    if pan_degree < pan_target:
        y_move = map_value(ball_x_angle, -30.0, -8.0, ry_max, ry_min)
    elif pan_degree > 0.0:
        y_move = map_value(ball_x_angle, 8.0, 30.0, ly_min, ly_max)
    # if ball_x_angle < -0.0:
    #     y_move = map_value(ball_x_angle, -30.0, -8.0, ry_max, ry_min)
    # elif ball_x_angle > 6.0:
    #     y_move = map_value(ball_x_angle, 8.0, 30.0, ly_min, ly_max)

    heading_gap = normalize_deg(yaw_fixed - heading)
    a_move = map_value(heading_gap, 0.0, 360.0, -5.0, 5.0)
    if yaw_fixed2 < 2 and tilt_degree <-64:
        located = True
    else:
        located = False

    return x_move * 0.001, y_move * 0.001, a_move, located

ball_located = False
def walk_ball_positioning_to_goal(heading):
    global heading_gap, ball_located
    pan_degree = pan_present * 180.0 / np.pi
    tilt_degree = tilt_present * 180.0 / np.pi
    error_pos_x = pan_degree - 0.0
    error_pos_y = tilt_degree - 10.0
    # print(error_pos_y)

    if error_pos_y < -80:
        ball_located = True
    else:
        ball_located = False

    x_move = 0
    if np.abs(ball_x_angle) < 16.0:
        x_move = map_value(np.abs(ball_x_angle), 0.0, 10.0, 8.0, 0.)
    else:
        x_move = map_value(np.abs(ball_x_angle), 10.0, 35.0, 0.0, -10.0)

    y_move = 0
    ry_min = -10
    ry_max = -20
    ly_min = 10
    ly_max = 20
    if ball_x_angle < -2.0:
        y_move = map_value(ball_x_angle, -30.0, -5.0, ry_max, ry_min)
    elif ball_x_angle > 2.0:
        y_move = map_value(ball_x_angle, 5.0, 30.0, ly_min, ly_max)

    heading_gap =Heading_Gap(heading, yaw_fixed2)# normalize_deg(yaw_fixed2 - heading)
    a_move = map_value(heading_gap, 0.0, 360.0, -5.0, 5.0)

    return x_move * 0.001, y_move * 0.001, a_move, ball_located

goal_located = False
def walk_goal_positioning(heading):
    global heading_gap, goal_located
    pan_degree = pan_present * 180.0 / np.pi
    tilt_degree = tilt_present * 180.0 / np.pi
    error_pos_x = pan_degree - 0.0
    error_pos_y = tilt_degree - 10.0
    print(error_pos_y)

    if error_pos_y < -75:
        goal_located = True
    else:
        goal_located = False


    x_move = 0
    if np.abs(ball_x_angle) < 10.0:
        x_move = map_value(np.abs(ball_x_angle), 0.0, 10.0, 8.0, 0.)
    else:
        x_move = map_value(np.abs(ball_x_angle), 10.0, 35.0, 0.0, -10.0)

    y_move = 0
    ry_min = -10
    ry_max = -20
    ly_min = 10
    ly_max = 20
    if ball_x_angle < -5.0:
        y_move = map_value(ball_x_angle, -30.0, -5.0, ry_max, ry_min)
    elif ball_x_angle > 5.0:
        y_move = map_value(ball_x_angle, 5.0, 30.0, ly_min, ly_max)

    heading_gap = Heading_Gap(heading, yaw_fixed2) #normalize_deg(yaw_fixed2 - heading)
    a_move = map_value(heading_gap, 0.0, 360.0, -5.0, 5.0)

    return x_move * 0.001, y_move * 0.001, a_move, ball_located

heading_gap4 = 0
def walk_ball_position_to_kick(target_pan, target_tilt, heading): #walk_ball_position_to_kick (-8.0, -72, goal_orientation)
    global heading_gap4

    is_located = False
    pan_degree = pan_present * 180.0 / np.pi
    tilt_degree = tilt_present * 180.0 / np.pi
    delta_pan = (target_pan - pan_degree)
    delta_tilt = (target_tilt - tilt_degree)#semakin tunduk, delta semakin besar | param adalah titik 0
    # print("delta_pan: %d   delta_tilt:%d" % (delta_pan, delta_tilt))
    
    #mengambil titik tengah parmeter, dengan nilai toleransi atau batas atas dan bawah delta pan dan tilt adalah 3 
    # if (delta_pan > -2.0 and delta_pan < 2.0 ) and delta_tilt < 13 and ball_size >4000:#(delta_tilt > -8.0 and delta_tilt < 8.0): #(delta_pan < 6 and delta_tilt < 6) or ball_size >3000: #3
    #     is_located = True
    # else:
    #     is_located = False

    abs_delta_pan = np.abs(delta_pan)
    abs_delta_tilt = np.abs(delta_tilt)

    x_move = 0.0
    delta_tilt_pan = delta_tilt + (abs_delta_pan * 0.3)#harus berkorelasi dengan delta tilt
    if (delta_tilt > 1.0):
        x_move = map_value2(delta_tilt, 3.0, 20.0, -15.0 * 0.5, -13.0)
    elif (delta_tilt < -1.0):
        x_move = map_value2(delta_tilt, -20.0, -3.0, 3.0, 5.0 * 0.5)
        
    y_move = 0.0
    if target_pan > 0:
        if (pan_degree < -0.0):
            y_move = map_value(pan_degree - target_pan, -0.0, -20.0, -5.0, -10.0) 
        elif (pan_degree > target_pan):
            y_move = map_value(pan_degree - target_pan, 20.0, 0.0, 10.0, 5.0)
        
    else:
        if (pan_degree < target_pan):
            y_move = map_value(pan_degree - target_pan, -0.0, -20.0, -5.0, -10.0) #kanan
        elif (pan_degree > 0.0):
            y_move = map_value(pan_degree - target_pan, 20.0, 0.0, 10.0, 5.0) # kiri


    heading_gap4 = Heading_Gap(heading, yaw_fixed2)#normalize_deg(yaw_fixed2 - heading)
    a_move = map_value2(heading_gap4, -180, 180, -4.0, 4.0)
    # a_move = map(delta_direction, -15.0, 15.0, position_max_a, -position_max_a);

    print("x_move:%d  y_move:%d  a_move:%d" % (x_move, y_move, a_move))
    print("deg_pan: %d  delta_tilt:%d  deg_tilt:%d " % (pan_degree, delta_tilt, tilt_degree))
    return x_move * 0.001, y_move * 0.001, a_move#, is_located


heading_gap6 = 0
def walk_ball_position_to_kick2(target_pan, target_tilt, heading): #walk_ball_position_to_kick (-8.0, -72, goal_orientation)
    global heading_gap6

    is_located = False
    pan_degree = pan_present * 180.0 / np.pi
    tilt_degree = tilt_present * 180.0 / np.pi
    delta_pan = (target_pan - pan_degree)
    delta_tilt = (target_tilt - tilt_degree)#semakin tunduk, delta semakin besar | param adalah titik 0
    print("delta_pan: %d   delta_tilt:%d" % (delta_pan, delta_tilt))

    abs_delta_pan = np.abs(delta_pan)
    abs_delta_tilt = np.abs(delta_tilt)

    x_move = 0.0
    delta_tilt_pan = delta_tilt + (abs_delta_pan * 0.3)#harus berkorelasi dengan delta tilt
    if (delta_tilt_pan > 2.0):
        x_move = map_value2(delta_tilt_pan, 3.0, 20.0, -15.0 * 0.5, -15.0)
    elif (delta_tilt_pan < -2.0):
        x_move = map_value2(delta_tilt_pan, -20.0, -3.0, 5.0, 5.0 * 0.5)

    y_move = calculate_pid((float)(pan_degree), target_pan)
    y_move = (float)(y_move)/30.0
    y_move = limit_value(y_move, -8.0, 8.0)

    heading_gap6 = Heading_Gap(heading, yaw_fixed2)#normalize_deg(yaw_fixed2 - heading)
    a_move = map_value2(heading_gap4, 0.0, 360.0, -5.0, 5.0)
    # a_move = map(delta_direction, -15.0, 15.0, position_max_a, -position_max_a);

    #mengambil titik tengah parmeter, dengan nilai toleransi atau batas atas dan bawah delta pan dan tilt adalah 3 
    if ((delta_pan > -2.0 and delta_pan < 2.0) and (delta_tilt > -1.0 and delta_tilt < 4.0)) and (Heading_Gap(heading, yaw_fixed2) > -3 and Heading_Gap(heading, yaw_fixed2) < 3): #(delta_pan < 6 and delta_tilt < 6) or ball_size >3000: #3
        is_located = True
    else:
        is_located = False

    print("x_move:%d  delta_pan: %d   delta_tilt:%d" % (x_move, delta_pan, delta_tilt))
    return x_move * 0.001, y_move * 0.001, a_move, is_located



def udp_sendmsg(msg):
    from_ros_to_udp.publish(msg)
    time.sleep(0.1)

def Motion_Kick(num):
    Motion_Stop()
    Motion_InitAction()
    time.sleep(1.0)
    Motion_ActionNum(num)
    Motion_InitWalking()
    walking(0, 0, 0, 0.45)
    Motion_InitHead()
    # Motion_HeadControl(0.0, headTilt)

previous_statement = ""
def print_once(statement):
    global previous_statement
    present_statement = statement
    if present_statement == previous_statement:
        pass
    else:
        rospy.loginfo(statement)
    previous_statement = present_statement
taskTimer = 0
start_timer = 0
count_task = 0
ball_in_position = 0
# rotational_angle = 1
buzzerTimer = 0
pan_degree = 0
goal_fixed = 0
in_position_count = 0
Taskcom = ""
Task_sec = ""
Task_goto = ""
Task_secCF = ""
Task_pos = ""
Task = Task_goto = Task_sec = Task_secCF= Task_kickoff= ''
now_kickoff_time = start_kickoff_time = 0

if __name__ == '__main__':
    try:
        Init()
        # initpoint("center")
        rospy.loginfo("GANDAMANA ROBOT 3 KOMUNIKASI START!!")
        while not rospy.is_shutdown():
            # print (koneksi, id_1, id_2, id_3)
            # Goalposition(JERSEY)
            # print(goal_orientation)
            # walk_to_position(550, 200)
            tilt_degree = tilt_present * 180 / np.pi
            # pan_degree = pan_present * 180 / np.pi
            # print(tilt_degree)
            # print (yaw_fixed2)
            # print(KICKOFF)
            pan_degree = pan_present * 180 / np.pi
            # print(STATE)
            led_status_ball = True
            # print("status ball atas: ", status_ball)
            if check_ball_lost (1) == 0:
                # print("haha")
                pan_degree = pan_present * 180 / np.pi
                # print(pan_degree)
                if pan_degree > 0:
                    rotational_angle = 2
                else:
                    rotational_angle = 1
            else:
                pass
            # print(rotational_angle)
            # print(pan_degree)
            if Status == 'READY':
                # Buzzer(3000)
                # rospy.loginfo('test: %d', 1.7)
                print_once(Status)
                # Com_decision()
                start_tracking(True)
                LED_RGB(0, 0, 0)
                LED_Status(0, 0, 0)
                Motion_InitWalking()
                Motion_InitHead()
                # Motion_Start()
                time.sleep(0.1)
                Motion_HeadControl(0.0, -0.10)
                LED_RGB(255, 255, 255)
                LED_Status(1, 1, 1)
                Status = 'STANDBY'
            elif Status == 'PRE_POSITIONING_KICKOFF':#--------
                print_once('Status: POSITIONING KICKOF!!!')
                Motion_Start()
                start_tracking(False)
                Status = 'POSITIONING_KICKOFF'
                Task_pos = 'pos'
                walking(-0.003, 0, 0.0, 0.63)
            elif Status == "SET":
                print_once('Status: SET STATE RISED!!!')
                Status = 'PRE_STOP'
            elif Status == 'PRE_RUN':
                print_once('Status: READY TO START!!!')
                led_status_ball = True
                Motion_Start()
                walking(-0.003, 0, 0.0, 0.63)
                # Com_decision()
                print (robotstatus)
                # if KICKOFF == True:
                #     Status = 'RUN'
                #     robotstatus = 'kickoff'
                #     Task_kickoff = "initial"
                #     start_kickoff_time = time.time()
                # else:
                    # Com_decision()
                    # Status = 'RUN'
                Task = Task_goto = Task_sec = Task_secCF= Task_kickoff= 'initial'
                print(STATE)
                if STATE == 'STATE_READY':
                    Status = "PRE_POSITIONING_KICKOFF"
                    print("DONE")
                else:
                    Com_decision()
                    Status = 'RUN'
                #Task = 'testing'
                start_tracking(True)
                # rospy.loginfo('Initial executed!')
                taskTimer = Ticks()
            elif Status == 'PRE_STOP':
                print_once('Status: STOP!!!')
                Taskcom = "x"
                TaskcomPub.publish(Taskcom)
                start_tracking(True)
                Motion_InitHead()
                Motion_HeadControl(0.0, 3.0)
                led_status_ball = True
                Motion_Stop()
                Task = Task_goto = Task_sec = Task_secCF= Task_kickoff= 'initial'
                Status = 'STOP'
            elif Status == "POSITIONING_KICKOFF":
                # Motion_Start()
                if Task_pos == "pos":
                    print("goto position")
                    Motion_InitHead()
                    Motion_HeadControl(0.0, 0.0)
                    OdomStart()
                    walking(-0.003, 0, 0.0, 0.63)
                    task_ = "positioning"
                    task_Pub.publish(task_)
                    # print("test positioning")
                    if JERSEY == "CYAN":
                        xmove, ymove, amove, located = walk_to_position(410, 300, 0)
                    else:
                        xmove, ymove, amove, located = walk_to_position(490, 300, 179)

                    walking(xmove,ymove,amove,0.63)
                    if located == True:
                        Status = "PRE_STOP"
                        rospy.loginfo('Positioning Kickoff was Done!!')
                elif Task_pos == 'get_up':
                        OdomStop()
                        rospy.loginfo('KICKOFF -> FALL...')
                        if body_status == 'fallen_forward':
                            # Com_decision()
                            Motion_Stop()
                            Motion_InitAction()
                            Motion_ActionNum(33)
                            time.sleep(3)
                            Motion_InitWalking()
                            Motion_Start()
                            start_tracking(True)
                            time.sleep(1)
                            BodystatePub.publish(body_status)
                            body_status = 'walk_ready'
                        elif body_status == 'fallen_backward':
                            # Com_decision()
                            Motion_Stop()
                            Motion_InitAction()
                            Motion_ActionNum(134)
                            time.sleep(3)
                            Motion_InitWalking()
                            Motion_Start()
                            start_tracking(True)
                            time.sleep(1)
                            BodystatePub.publish(body_status)
                            body_status = 'walk_ready'
                        elif body_status == 'walk_ready':
                            # Com_decision()
                            start_tracking(True)
                            Motion_InitWalking()
                            # walking(0, 0, 0, 0.63)
                            Motion_InitHead()
                            # Motion_HeadControl(0.0, -0.07)
                            Motion_Start()
                            rospy.loginfo('Back to initial executed!')
                            body_status = "walkready"
                            BodystatePub.publish(body_status)
                            Status = 'PRE_POSITIONING_KICKOFF'
                            Task = Task_goto = Task_sec = Task_secCF= Task_kickoff= 'initial'
            elif Status == 'RUN':
                # print_once("Status: RUN EXECUTED!!!")
                if robotstatus == 'kickoff':
                    Task = Task_goto = Task_sec = Task_secCF= 'initial'
                    now_kickoff_time = time.time()
                    delta_kickoff = now_kickoff_time - start_kickoff_time
                    if delta_kickoff > 10:
                        Com_decision()
                    if Task_kickoff == "initial":
                        Motion_InitWalking()
                        Motion_Start()
                        led_status_ball = True
                        start_tracking(True)
                        OdomStart()
                        if JERSEY == "CYAN":
                            goal_orientation = -50
                        elif JERSEY == "MAGENTA":
                            goal_orientation = 130
                        else:
                            Goalposition(JERSEY)
                        Task_kickoff = "positioning to kick"
                    elif Task_kickoff == "positioning to kick":
                        if check_ball_lost(20) == 0:
                            # Goalposition(JERSEY)
                            if yaw_fixed2 > goal_orientation - 6 and yaw_fixed2 < goal_orientation + 6:
                                rospy.loginfo('KICKOFF -> POSITIONING KICK -> YAW IN TARGET...')
                                Taskcom = "posbl_kckoff"
                                TaskcomPub.publish(Taskcom)
                                # Com_decision()
                                pan_degree = pan_present * 180.0 / np.pi
                                tilt_degree = tilt_present * 180.0 / np.pi
                                #---------
                                target_pan = 4.0
                                target_tilt = -69.0
                                delta_pan = (target_pan - pan_degree)
                                delta_tilt = (target_tilt - tilt_degree)#semakin tunduk, delta semakin besar | param adalah titik 0
                                xmove, ymove, amove = walk_ball_position_to_kick (target_pan, target_tilt, goal_orientation)#(8.0, -71, goal_orientation)
                                walking(xmove, ymove, amove,0.63)
                                if (pan_degree > 2.0 and pan_degree <= target_pan + 2) and delta_tilt > -3:# and ball_size > 4000:
                                    ball_in_position +=1
                                else: 
                                    ball_in_position = 0 
                                # rospy.loginfo('Left Kick!!!')

                                task_ = "stop"
                                task_Pub.publish(task_)
                                 #0.64

                                if ball_in_position >2 :#75
                                    Task_kickoff = 'kick_ball'
                                    # Status = 'PRE_STOP'
                                    rospy.loginfo('kickoff -> POSITIONING KICK -> KICK EXECUTED')
                                    # walking(-0.005, -0.015, 0.0, 0.63)
                                    count_goal_found = 0
                                    Motion_Stop()
                                    Motion_InitAction()
                                    time.sleep(1.2)
                            
                            else:
                                rospy.loginfo('KICKOFF -> POSITIONING KICK -> ROTATE...')
                                Taskcom = "rt_kckoff"
                                TaskcomPub.publish(Taskcom)
                                # Com_decision()
                                # rospy.loginfo('Task : Rotate[3]')
                                count_goal_found = 0
                                xmove, ymove, amove = walk_rotate22(goal_orientation)
                                task_ = "rotate"
                                task_Pub.publish(task_)
                                walking(xmove, ymove, amove, 0.63)
                        else:
                            rospy.loginfo('kickoff -> POSITIONING KICK -> SCAN AROUND...')
                            Taskcom = "sc"
                            TaskcomPub.publish(Taskcom)
                            task_ = "scan"
                            task_Pub.publish(task_)
                            walking(-0.005, 0.0, 0.0, 0.63)
                            count_goal_found = 0
                    elif Task_kickoff == 'kick_ball':
                        OdomStop()
                        pan_degree = pan_present * 180 / np.pi
                        if  ball_x_angle < 0:
                            rospy.logwarn('FOOT DECISION: RIGHT KICK!!!')
                            # Motion_Stop()
                            # Motion_InitAction()
                            # time.sleep(0.5)
                            Taskcom = "kckoff"
                            TaskcomPub.publish(Taskcom)
                            Motion_ActionNum(230)
                            # Motion_ActionNum(181)
                            # Motion_ActionNum(182)
                            # Motion_ActionNum(183)
                            Taskcom = "kckoff"
                            Com_decision()
                            Motion_InitWalking()
                            
                            Motion_HeadControl(0.0, -0.20)
                            # walking(0, 0, 0, 0.63)
                            Motion_InitHead()
                            Motion_Start()
                            start_tracking(True)
                            Task_kickoff = 'check_takeball_opportunity'
                        else:
                            rospy.logwarn('FOOT DECISION: LEFT KICK!!!')
                            # Motion_Stop()
                            # Motion_InitAction()
                            # time.sleep(0.5)
                            Taskcom = "kckoff"
                            TaskcomPub.publish(Taskcom)
                            Motion_ActionNum(225)
                            # Motion_ActionNum(177)
                            # Motion_ActionNum(178)
                            # Motion_ActionNum(179)
                            Taskcom = "kckoff"
                            Com_decision()
                            Motion_InitWalking()
                            Motion_HeadControl(0.0, -0.20)
                            # walking(0, 0, 0, 0.63)
                            Motion_InitHead()
                            Motion_Start()
                            start_tracking(True)
                            Task_kickoff = 'check_takeball_opportunity'
                    elif Task_kickoff == 'check_takeball_opportunity':
                        oppotunity = takeball_opportunity()
                        if oppotunity == 'n':
                            Motion_Start()
                            start_tracking(True)
                            walking(-0.004, 0, 0, 0.63)
                            rospy.logerr("mengizinkan robot second")
                            time.sleep (3)
                            Com_decision()
                            # robotstatus = "SECOND"
                            Task = Task_goto = Task_sec = Task_secCF= Task_kickoff= 'initial'
                        else:
                            Task = Task_goto = Task_sec = Task_secCF= Task_kickoff= 'initial'
                            rospy.logerr("robot second tidak menguntungkan")

                    elif Task_kickoff == 'get_up':
                        OdomStop()
                        rospy.loginfo('KICKOFF -> FALL...')
                        if body_status == 'fallen_forward':
                            Com_decision()
                            Motion_Stop()
                            Motion_InitAction()
                            Motion_ActionNum(33)
                            time.sleep(3)
                            Motion_InitWalking()
                            Motion_Start()
                            start_tracking(True)
                            time.sleep(1)
                            BodystatePub.publish(body_status)
                            body_status = 'walk_ready'
                        elif body_status == 'fallen_backward':
                            Com_decision()
                            Motion_Stop()
                            Motion_InitAction()
                            Motion_ActionNum(134)
                            time.sleep(3)
                            Motion_InitWalking()
                            Motion_Start()
                            start_tracking(True)
                            time.sleep(1)
                            BodystatePub.publish(body_status)
                            body_status = 'walk_ready'
                        elif body_status == 'walk_ready':
                            Com_decision()
                            start_tracking(True)
                            Motion_InitWalking()
                            # walking(0, 0, 0, 0.63)
                            Motion_InitHead()
                            # Motion_HeadControl(0.0, -0.07)
                            Motion_Start()
                            rospy.loginfo('Back to initial executed!')
                            body_status = "walkready"
                            BodystatePub.publish(body_status)
                            Task = Task_goto = Task_sec = Task_secCF= Task_kickoff= 'initial'
#-----------------------------------------------------------------------------------------------------------
                elif robotstatus == "SECOND_CF":
                    Task = Task_goto = Task_sec = Task_kickoff= 'initial'
                    com_to_cb = "stkr mode"
                    comcb.publish(com_to_cb)
                    if Task_secCF == "initial":
                        Motion_Start()
                        led_status_ball = True
                        Motion_InitWalking()
                        start_tracking(True)
                        OdomStart()
                        Task_secCF = "CFpositioning"
                    elif Task_secCF == "CFpositioning":
                        if JERSEY == "CYAN":
                            mygoalarea = "MAGENTA"
                            CFx = 525
                        else:
                            mygoalarea = "CYAN"
                            CFx = 375
                        Motion_InitHead()
                        Motion_HeadControl(0.0, 0.0)
                        OdomStart()
                        walking(-0.003, 0, 0.0, 0.63)
                        task_ = "positioning"
                        task_Pub.publish(task_)
                        # print("test positioning")
                        xmove, ymove, amove, located = walk_to_position(CFx, 300, Goalposition(mygoalarea))
                        walking(xmove,ymove,amove,0.63)
                        # Com_decision()
                        if located == True:
                            Motion_Stop()
                            Motion_InitHead()
                            Motion_InitWalking()
                            rospy.loginfo('Positioning Kickoff was Done!!')
                        
                        # if ball_size > 2000:
                        #     robotstatus = "UTAMA"
                        #     Taskcom = "stkr take ball"
                        #     TaskcomPub.publish(Taskcom)
                    elif Task_secCF == "CFdone":
                        start_tracking(True)
                        led_status_ball = True
                        if check_ball_lost(100):
                            if np.abs(pan_degree) > 60:
                                robotstatus = "UTAMA"
                                Taskcom = "stkr take ball"
                                TaskcomPub.publish(Taskcom)

                    elif Task_secCF == 'get_up':
                        OdomStop()
                        rospy.loginfo('UTAMA -> FALL...')
                        if body_status == 'fallen_forward':
                            # Com_decision()
                            Motion_Stop()
                            Motion_InitAction()
                            Motion_ActionNum(33)
                            time.sleep(3)
                            BodystatePub.publish(body_status)
                            body_status = 'walk_ready'
                        elif body_status == 'fallen_backward':
                            # Com_decision()
                            Motion_Stop()
                            Motion_InitAction()
                            Motion_ActionNum(134)
                            time.sleep(3)
                            BodystatePub.publish(body_status)
                            body_status = 'walk_ready'
                        elif body_status == 'walk_ready':
                            # Com_decision()
                            start_tracking(True)
                            Motion_InitWalking()
                            # walking(0, 0, 0, 0.63)
                            Motion_InitHead()
                            # Motion_HeadControl(0.0, -0.07)
                            Motion_Start()
                            rospy.loginfo('Back to initial executed!')
                            body_status = "walkready"
                            BodystatePub.publish(body_status)
                            Task_secCF= 'initial'
#-----------------------------------------------------------------------------------------------------------
                elif robotstatus == "GOTO":
                    Task = Task_sec = Task_secCF= Task_kickoff= 'initial'
                    print_once('RobotStatus: GOTO...')
                    if Task_goto == 'initial':
                        # Motion_InitWalking()
                        Motion_Start()
                        led_status_ball = True
                        start_tracking(True)
                        OdomStart()
                        Task_goto = "follow_ball"
                    elif Task_goto == "follow_ball":
                        if check_ball_lost(20) == 0:
                            rospy.loginfo('GOTO -> FOLLOW BALL...')
                            Taskcom = "fb_goto"
                            TaskcomPub.publish(Taskcom)
                            start_tracking(True)
                            led_status_ball = True
                            Motion_Start()
                            xmove, ymove, amove = walk_follow_head(pan_present, tilt_present)
                            # print(xmove, ymove, amove) 
                            task_ = "move"
                            task_Pub.publish(task_)
                            walking(xmove, ymove, amove, 0.63)
                            tilt_degree = tilt_present * 180 / np.pi
                            if ball_size >= ballsize_treshold:
                                Com_decision()
                                rospy.loginfo('GOTO -> MAKE DECISION...')
                                walking(0.006, 0.0, amove, 0.63)
                        else:
                            # Motion_Start()
                            # led_status_ball = True
                            # start_tracking(True)
                            Com_decision()
                            rospy.loginfo('GOTO -> SCAN AROUND...')
                            Taskcom = "sc_goto"
                            TaskcomPub.publish(Taskcom)
                            # rospy.loginfo(rotational_angle)
                            task_ = "stay"
                            task_Pub.publish(task_) 
                            if rotational_angle == 1:
                                walking(-0.005, 0.0, -3.0, 0.63)
                            elif rotational_angle == 2:
                                walking(-0.005, 0.0, 3.0, 0.63)
                            else:
                                walking(-0.005, 0.0, 0.0, 0.63)
                    elif Task_goto == 'get_up':
                        OdomStop()
                        rospy.loginfo('GOTO -> FALL...')
                        if body_status == 'fallen_forward':
                            Com_decision()
                            Motion_Stop()
                            Motion_InitAction()
                            Motion_ActionNum(33)
                            time.sleep(3)
                            Motion_InitWalking()
                            Motion_Start()
                            start_tracking(True)
                            time.sleep(1)
                            BodystatePub.publish(body_status)
                            body_status = 'walk_ready'
                        elif body_status == 'fallen_backward':
                            Com_decision()
                            Motion_Stop()
                            Motion_InitAction()
                            Motion_ActionNum(134)
                            time.sleep(3)
                            Motion_InitWalking()
                            Motion_Start()
                            start_tracking(True)
                            time.sleep(1)
                            BodystatePub.publish(body_status)
                            body_status = 'walk_ready'
                        elif body_status == 'walk_ready':
                            Motion_InitHead()
                            Motion_HeadControl(0.0, -50)
                            led_status_ball = True
                            start_tracking(True)
                            Motion_InitHead()
                            Motion_HeadControl(0.0, -50)
                            Motion_InitWalking()
                            # walking(0, 0, 0, 0.63)
                            # Motion_InitHead()
                            # Motion_HeadControl(0.0, -0.07)
                            Motion_Start()
                            rospy.loginfo('Back to initial executed!')
                            body_status = "walkready"
                            BodystatePub.publish(body_status)
                            Task = Task_goto = Task_sec = Task_secCF= Task_kickoff= 'initial'
                    else:
                        Task_goto = 'initial'
#------------------------------------------------------------------------------------------------------------
                elif robotstatus == "SECOND":
                    Task = Task_goto  = Task_secCF= Task_kickoff= 'initial'
                    print_once('RobotStatus: SECOND...')
                    if Task_sec == 'initial':
                        # Motion_InitWalking()
                        Motion_Start()
                        led_status_ball = True
                        start_tracking(True)
                        OdomStart()
                        Task_sec = "follow_ball"
                    elif Task_sec == "follow_ball":
                        if check_ball_lost(20) == 0:
                            rospy.loginfo('SECOND -> FOLLOW BALL...')
                            Taskcom = "fb_sec"
                            TaskcomPub.publish(Taskcom)
                            start_tracking(True)
                            xmove, ymove, amove = walk_follow_head(pan_present, tilt_present)
                            # print(xmove, ymove, amove) 
                            task_ = "move"
                            task_Pub.publish(task_)
                            walking(xmove, ymove, amove, 0.63)
                            Com_decision()
                            tilt_degree = tilt_present * 180 / np.pi
                            if tilt_degree < -30:
                                Task_sec = 'second_positioning'
                                if yaw_fixed2 < 0:
                                    goal_orientation = -90
                                else:
                                    goal_orientation = 90
                                rospy.loginfo('SECOND POSITIONING START!!!!')
                            # else:
                            #     Task = 'follow_ball'
                            #     rospy.loginfo('Ball follow executed! [2]')
                        else:
                            rospy.loginfo('SECOND -> SCAN AROUND...')
                            Taskcom = "sc_sec"
                            TaskcomPub.publish(Taskcom)
                            # rospy.loginfo(rotational_angle)
                            task_ = "stay"
                            task_Pub.publish(task_) 
                            if rotational_angle == 1:
                                walking(-0.005, 0.0, -3.0, 0.63)
                            elif rotational_angle == 2:
                                walking(-0.005, 0.0, 3.0, 0.63)
                            else:
                                walking(-0.005, 0.0, 0.0, 0.63)
                            Com_decision()
                    elif Task_sec == 'second_positioning':
                        if check_ball_lost(20) == 0:
                            if yaw_fixed2 > goal_orientation - 6 and yaw_fixed2 < goal_orientation + 6:
                                #xmove, ymove, amove, is_located = walk_ball_positioning_to_goal(goal_orientation
                                # Com_decision()
                                led_status_ball = True
                                start_tracking(True)
                                Motion_Stop()
                                Taskcom = "stp_sec"
                                TaskcomPub.publish(Taskcom)
                                rospy.loginfo('SECOND -> MAKE DECISION...')
                                if task_3 == "rt" or task_3 == "posbal":
                                    pass
                                else:
                                    Com_decision()
                                
                            else:
                                rospy.loginfo('SECOND -> ROTATE POSITIONING...') 
                                second_orientation()                               
                                Taskcom = "rt_sec"
                                # rospy.loginfo('Task: Rotate[1]')
                                task_ = "rotate" 
                                task_Pub.publish(task_)
                                xmove, ymove, amove = second_rotate(goal_orientation, -30 )
                                walking(xmove, ymove, amove, 0.63)
                                Com_decision()
                        else:
                            # Motion_HeadControl(0.0, -0.40)
                            Task_sec = 'initial'
                            walking(-0.005, 0.0, 0.0, 0.63)
                            rospy.loginfo('Initial second executed!! [from positioning]')
                    elif Task_sec == 'get_up':
                        OdomStop()
                        rospy.loginfo('SECOND -> FALL...')
                        if body_status == 'fallen_forward':
                            Com_decision()
                            Motion_Stop()
                            Motion_InitAction()
                            Motion_ActionNum(33)
                            time.sleep(3)
                            Motion_InitWalking()
                            Motion_Start()
                            start_tracking(True)
                            time.sleep(1)
                            BodystatePub.publish(body_status)
                            body_status = 'walk_ready'
                        elif body_status == 'fallen_backward':
                            Com_decision()
                            Motion_Stop()
                            Motion_InitAction()
                            Motion_ActionNum(134)
                            time.sleep(3)
                            Motion_InitWalking()
                            Motion_Start()
                            start_tracking(True)
                            time.sleep(1)
                            BodystatePub.publish(body_status)
                            body_status = 'walk_ready'
                        elif body_status == 'walk_ready':
                            Com_decision()
                            Motion_InitHead()
                            Motion_HeadControl(0.0, -50)
                            start_tracking(True)
                            Motion_InitWalking()
                            # walking(0, 0, 0, 0.63)
                            Motion_InitHead()
                            # Motion_HeadControl(0.0, -0.07)
                            Motion_Start()
                            rospy.loginfo('Back to initial executed!')
                            body_status = "walkready"
                            BodystatePub.publish(body_status)
                            Task = Task_goto = Task_sec = Task_secCF= Task_kickoff= 'initial'
                    else:
                        Task = Task_goto = Task_sec = Task_secCF= Task_kickoff= 'initial'
                    Com_decision()
#------------------------------------------------------------------------------------------------------------
                elif robotstatus == "UTAMA":
                    Task_goto = Task_sec = Task_secCF= Task_kickoff= 'initial'
                    # print_once('RobotStatus: UTAMA...')
                    # if detect_4 == "ACTIVE":
                    #     com_to_cb = "mid mode"
                    #     comcb.publish(com_to_cb)
                    # else:
                    #     com_to_cb = "stkr mode"
                    #     comcb.publish(com_to_cb)
                    
                    if Task == 'testing':
                        start_tracking(True)
                    if Task == 'initial':
                        Com_decision()
                        if Ticks() -  start_timer >= 1000:
                            rospy.loginfo('UTAMA -> INITIAL...')
                            count_task += 1
                            # Motion_InitWalking()
                            Motion_Start()
                            start_tracking(True)#command untuk tracking bola
                            led_status_ball = True
                            OdomStart()
                            if check_ball_lost(10):
                                xmove, ymove, amove = walk_follow_head(pan_present, tilt_present)
                                walking(-0.004, 0.0, amove, 0.63)
                            else:
                                walking(-0.004, 0.0, 0.0, 0.63)
                            task_ = "initial"
                            task_Pub.publish(task_)
                            # print(count_task)
                            start_timer = Ticks()
                            if count_task > 2:
                                count_task = 0
                                Task = 'follow_ball'
                                # rotational_angle = r.randint(1, 2)
                                rospy.loginfo('Ball follow executed!')
                    
                    elif Task == "Transition":
                        if check_ball_lost(20) == 0:
                            print("transition")
                            pan_degree = pan_present * 180.0 / np.pi
                            a_move = map_value2(pan_degree, -89.0, 89.0, -10, 10)
                            a_move = limit_value(a_move, -7.0, 7.0)
                            walking(-0.01, 0.0, a_move, 0.63)
                            if np.abs(pan_degree) < 30 and tilt_degree < -50:
                                Task = "heading_to_goal_first"
                            elif np.abs(pan_degree) < 30:
                                Task = "follow_ball"
                            
                            if pan_degree < 0:
                                rotational_angle = 1
                            else:
                                rotational_angle = 0
                        else:
                            if rotational_angle == 1:
                                walking(-0.004, 0.0, -5.0, 0.63)
                            elif rotational_angle == 0:
                                walking(-0.004, 0.0, 5.0, 0.63)
                            else:
                                walking(-0.004, 0.0, 0.0, 0.63)


                    elif Task == 'follow_ball':
                        # print(check_ball_lost(20))
                        Com_decision()
                        print("status ball: ", status_ball)
                        if check_ball_lost(10) == 0:
                            # pan_degree = pan_present * 180.0 / np.pi
                            # if pan_degree > 0:
                            #     rotational_angle = 2
                            # else:
                            #     rotational_angle = 1
                            rospy.loginfo('UTAMA -> FOLLOW BALL...')
                            Taskcom = "fb"
                            TaskcomPub.publish(Taskcom)
                            start_tracking(True)
                            xmove, ymove, amove = walk_follow_head(pan_present, tilt_present)
                            # print(xmove, ymove, amove) 
                            task_ = "move"
                            task_Pub.publish(task_)
                            walking(xmove, ymove, amove, 0.63)
                            tilt_degree = tilt_present * 180 / np.pi
                            # rospy.loginfo('Task: Follow Ball!')
                            if tilt_degree < -45:
                                Task = 'heading_to_goal_first'
                                Goalposition(JERSEY)
                                # print(goal_orientation)
                                rospy.loginfo('Checking goal executed!')
                            # else:
                            #     Task = 'follow_ball'
                            #     rospy.loginfo('Ball follow executed! [2]')
                        else:
                            rospy.loginfo('UTAMA -> SCAN AROUND...')
                            Taskcom = "sc"
                            TaskcomPub.publish(Taskcom)
                            # rospy.loginfo(rotational_angle)
                            task_ = "stay"
                            task_Pub.publish(task_) 
                            if rotational_angle == 1:
                                walking(-0.004, 0.0, -3.0, 0.63)
                            elif rotational_angle == 2:
                                walking(-0.004, 0.0, 3.0, 0.63)
                            else:
                                walking(-0.004, 0.0, 0.0, 0.63)

                    elif Task == 'heading_to_goal_first':
                        print("status ball: ", status_ball)
                        if check_ball_lost(10) == 0:
                            # pan_degree = pan_present * 180.0 / np.pi
                            # if pan_degree > 0:
                            #     rotational_angle = 2
                            # else:
                            #     rotational_angle = 1
                            if yaw_fixed2 > goal_orientation - 8 and yaw_fixed2 < goal_orientation + 8:
                                #xmove, ymove, amove, is_located = walk_ball_positioning_to_goal(goal_orientation)
                                rospy.loginfo('UTAMA -> HEADING TO GOAL 1 -> YAW IN TARGET....')
                                Goalposition(JERSEY)
                                # print(goal_orientation)
                                Task = 'positioning_kick'#'heading_to_goal_second'

                            else:
                                
                                rospy.loginfo('UTAMA -> HEADING TO GOAL 1 -> ROTATE -> %d', goal_orientation)
                                Taskcom = "rt"
                                TaskcomPub.publish(Taskcom)
                                Com_decision()
                                # rospy.loginfo('Task: Rotate[1]')
                                count_goal_found = 0
                                task_ = "rotate"
                                task_Pub.publish(task_)
                                xmove, ymove, amove = walk_rotate2(goal_orientation)
                                walking(xmove, ymove, amove, 0.63)
                                
                        else:
                            # Motion_HeadControl(0.0, -0.40)
                            Task = 'follow_ball'
                            if rotational_angle == 1:
                                walking(-0.004, 0.0, -3.0, 0.63)
                            elif rotational_angle == 2:
                                walking(-0.004, 0.0, 3.0, 0.63)
                            else:
                                walking(-0.004, 0.0, 0.0, 0.63)
                            rospy.loginfo('Ball follow executed! [from positioning]')

                    elif Task =='positioning_kick':
                        print("status ball: ", status_ball)
                        if check_ball_lost(10) == 0:
                            # pan_degree = pan_present * 180.0 / np.pi
                            # if pan_degree > 0:
                            #     rotational_angle = 2
                            # else:
                            #     rotational_angle = 1
                            # Goalposition(JERSEY)
                            if yaw_fixed2 > goal_orientation - 20 and yaw_fixed2 < goal_orientation + 20:
                                rospy.loginfo('UTAMA -> POSITIONING KICK -> YAW IN TARGET...')
                                Taskcom = "posbl"
                                TaskcomPub.publish(Taskcom)
                                Com_decision()
                                pan_degree = pan_present * 180.0 / np.pi
                                tilt_degree = tilt_present * 180.0 / np.pi
                                
                                if goal_orientation > 0:#kiri kir
                                    target_pan = 4.0
                                    target_tilt = -68.0
                                    delta_pan = (target_pan - pan_degree)
                                    delta_tilt = (target_tilt - tilt_degree)#semakin tunduk, delta semakin besar | param adalah titik 0
                                    xmove, ymove, amove = walk_ball_position_to_kick (target_pan, target_tilt, goal_orientation)#(8.0, -71, goal_orientation)
                                    walking(xmove, ymove, amove,0.63)
                                    if (pan_degree > 2.0 and pan_degree <= target_pan + 2) and delta_tilt > -3:# and ball_size > 4000:
                                        ball_in_position +=1
                                    else: 
                                        ball_in_position = 0 
                                    # rospy.loginfo('Left Kick!!!')

                                else:#kanan kanan
                                    target_pan = -4.0
                                    target_tilt = -68.0
                                    delta_pan = (target_pan - pan_degree)
                                    delta_tilt = (target_tilt - tilt_degree)#semakin tunduk, delta semakin besar | param adalah titik 0
                                    xmove, ymove, amove = walk_ball_position_to_kick (target_pan, target_tilt, goal_orientation)#(-8.0, -71, goal_orientation)
                                    walking(xmove, ymove, amove,0.63)
                                    if (pan_degree < -2.0 and pan_degree >= target_pan - 2) and delta_tilt > -2:# and ball_size > 4000:
                                        ball_in_position +=1
                                    else: 
                                        ball_in_position = 0 
                                
                                task_ = "stop"
                                task_Pub.publish(task_)
                                 #0.64

                                if ball_in_position >2 :#75
                                    Task = 'kick_ball'
                                    # Status = 'PRE_STOP'
                                    rospy.loginfo('UTAMA -> POSITIONING KICK -> KICK EXECUTED')
                                    # walking(-0.005, -0.015, 0.0, 0.63)
                                    count_goal_found = 0
                                    Motion_Stop()
                                    Motion_InitAction()
                                    time.sleep(1)
                                
                                pan_degree = pan_present * 180.0 / np.pi
                                if np.abs(pan_degree) > 40 :
                                    Task = "Transition"
                                if tilt_degree > -36:
                                    Task = 'follow_ball'
                            else:
                                rospy.loginfo('UTAMA -> POSITIONING KICK -> ROTATE...')
                                Taskcom = "rt"
                                TaskcomPub.publish(Taskcom)
                                Com_decision()
                                # rospy.loginfo('Task : Rotate[3]')
                                count_goal_found = 0
                                xmove, ymove, amove = walk_rotate22(goal_orientation)
                                task_ = "rotate"
                                task_Pub.publish(task_)
                                walking(xmove, ymove, amove, 0.63)
                        else:
                            rospy.loginfo('UTAMA -> POSITIONING KICK -> SCAN AROUND...')
                            Taskcom = "sc"
                            TaskcomPub.publish(Taskcom)
                            task_ = "scan"
                            task_Pub.publish(task_)
                            if rotational_angle == 1:
                                walking(-0.004, 0.0, -3.0, 0.63)
                            elif rotational_angle == 2:
                                walking(-0.004, 0.0, 3.0, 0.63)
                            else:
                                walking(-0.004, 0.0, 0.0, 0.63)
                            count_goal_found = 0
                            Task = 'follow_ball'

                    elif Task == 'kick_ball':
                        OdomStop()
                        led_status_ball = False
                        start_tracking(False)
                        pan_degree = pan_present * 180 / np.pi
                        if  ball_x_angle < 0:
                            rospy.logwarn('FOOT DECISION: RIGHT KICK!!!')
                            # Motion_Stop()
                            # Motion_InitAction()
                            # time.sleep(0.5)
                            Taskcom = "kck"
                            TaskcomPub.publish(Taskcom)
                            Motion_ActionNum(230)
                            # Motion_ActionNum(181)
                            # Motion_ActionNum(182)
                            # Motion_ActionNum(183)
                            Taskcom = "kck"
                            Com_decision()
                            led_status_ball = True
                            start_tracking(True)
                            Motion_InitWalking()
                            
                            Motion_HeadControl(0.0, -0.20)
                            # walking(0, 0, 0, 0.63)
                            Motion_InitHead()
                            Motion_InitWalking()
                            Motion_Start()
                            start_tracking(True)
                            time.sleep(1)
                            Task = Task_goto = Task_sec = Task_secCF= Task_kickoff= 'initial'
                            Task = 'check_takeball_opportunity'
                        else:
                            rospy.logwarn('FOOT DECISION: LEFT KICK!!!')
                            # Motion_Stop()
                            # Motion_InitAction()
                            # time.sleep(0.5)
                            Taskcom = "kck"
                            TaskcomPub.publish(Taskcom)
                            Motion_ActionNum(225)
                            # Motion_ActionNum(177)
                            # Motion_ActionNum(178)
                            # Motion_ActionNum(179)
                            Taskcom = "kck"
                            Com_decision()
                            led_status_ball = True
                            start_tracking(True)
                            Motion_InitWalking()
                            Motion_HeadControl(0.0, -0.20)
                            # walking(0, 0, 0, 0.63)
                            Motion_InitHead()
                            Motion_Start()
                            start_tracking(True)
                            time.sleep(1)
                            Task = Task_goto = Task_sec = Task_secCF= Task_kickoff= 'initial'
                            Task = 'check_takeball_opportunity'
                    elif Task == 'check_takeball_opportunity':
                        oppotunity = takeball_opportunity()
                        if oppotunity == 'n':
                            Motion_InitWalking()
                            Motion_Start()
                            start_tracking(True)
                            walking(-0.004, 0, 0, 0.63)
                            rospy.logerr("mengizinkan robot second")
                            time.sleep (3)
                            Com_decision()
                            # robotstatus = "SECOND"
                            # Com_decision()
                            Task = Task_goto = Task_sec = Task_secCF= Task_kickoff= 'initial'
                        else:
                            Task = Task_goto = Task_sec = Task_secCF= Task_kickoff= 'initial'
                            rospy.logerr("robot second tidak menguntungkan")

                    elif Task == 'get_up':
                        OdomStop()
                        rospy.loginfo('UTAMA -> FALL...')
                        if body_status == 'fallen_forward':
                            Com_decision()
                            Motion_Stop()
                            Motion_InitAction()
                            Motion_ActionNum(33)
                            time.sleep(1)
                            Motion_InitWalking()
                            Motion_Start()
                            time.sleep(1)
                            BodystatePub.publish(body_status)
                            body_status = 'walk_ready'
                        elif body_status == 'fallen_backward':
                            Com_decision()
                            Motion_Stop()
                            Motion_InitAction()
                            Motion_ActionNum(134)
                            time.sleep(1)
                            Motion_InitWalking()
                            Motion_Start()
                            time.sleep(1)
                            BodystatePub.publish(body_status)
                            body_status = 'walk_ready'
                        elif body_status == 'walk_ready':
                            # Com_decision()
                            start_tracking(True)
                            Motion_InitWalking()
                            # walking(0, 0, 0, 0.63)
                            Motion_InitHead()
                            # Motion_HeadControl(0.0, -0.07)
                            Motion_Start()
                            rospy.loginfo('Back to initial executed!')
                            body_status = "walkready"
                            BodystatePub.publish(body_status)
                            Task = Task_goto = Task_sec = Task_secCF= Task_kickoff= 'initial'

                    elif Task == 'walk_inarea':
                        walking(-0.003, 0.0, 0.0, 0.63)

                TaskcomPub.publish(Taskcom)
                BodystatePub.publish(body_status)
            elif Status == 'PRE_IDLE':
                led_status_ball = False
                Motion_InitPose()
                BuzzerTone(3, 60)
                # Buzzer(3000)
                # time.sleep(0.1)
                # Buzzer(0)
                # time.sleep(0.02)
                # Buzzer(3000)
                # time.sleep(0.1)
                # Buzzer(0)
                # time.sleep(0.02)
                # Buzzer(3000)
                # time.sleep(0.05)
                # Buzzer(0)
                Status = 'IDLE'
            elif Status == 'IDLE':
                if Ticks() - taskTimer < 250:
                    LED_RGB(255, 0, 0)
                    LED_Status(0, 0, 1)
                elif Ticks() - taskTimer < 500:
                    LED_RGB(0, 0, 255)
                    LED_Status(1, 0, 0)
                else:
                    taskTimer = Ticks()
            elif Status == 'STANDBY':
                Status = 'STOP'
            elif Status == 'STOP':
                ()

            if buzzerCount > 0:
                if Ticks() - buzzerTimer >= buzzerPeriod:
                    buzzerTimer = Ticks()
                    if not buzzerState:
                        Buzzer(3000)
                        buzzerState = 1
                    else:
                        Buzzer(0)
                        buzzerState = 0
                        buzzerCount -= 1
            else:
                buzzerTimer = Ticks()

    except rospy.ROSInterruptException():
        pass
