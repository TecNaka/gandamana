#!/usr/bin/python

import rospy
import socket
import json
import time
from std_msgs.msg import (String)
from communication.msg import RobotInfo

shootPub = rospy.Publisher('/DEWO/Communication/shooter', String, queue_size = 1) 
pub = rospy.Publisher("/DEWO/Communication/robotinfo", RobotInfo, queue_size=1)   
playerID = ballsize = 0
detect = fall = task = ""
position = [0,0,0]
goalorient = 0
if __name__ == '__main__':
    rospy.init_node('receivernasional')
    try:
        rospy.loginfo("GANDAMANA RECEIVER REDIII !!!")
        myIP = "192.168.1.32"
        port = 6604

        Socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        Socket.settimeout(1)
        Socket.bind((myIP, port))

        print("Server Started")
        while not rospy.is_shutdown():
                # print("as")
            try:
                data, addr = Socket.recvfrom(1024)
                print("a")
                # time_ = time.time()
                data = json.loads(data.decode())
                # print("Message from: " + str(addr))

                #pecah data
                playerID = data[0]
                fall = data[1]
                task = data[2]
                position = [data[3], data[4], data[5]]
                detect = data[6]
                ballsize = data[7]
                goalorient = data[8]

                #monitoring
                print ("player: ",playerID)
                print ("fall: ",fall)
                print ("task: ",task)
                print ("position: ",position)
                print ("detect: ",detect)
                print ("ballsize: ",ballsize)
                print ("goalorient: ",goalorient)
                print("--------------------------")

                # #reset
                # playerID = 0
                # fall = ""
                # task = ""
                # position = [0,0,0]
                # detect = ""
                # ballsize = 0

                # shootPub.publish(data)
            except socket.timeout:
                playerID = ballsize = 100
                detect = fall = task = "o"
                position = [0,0,0]
                goalorient = 0
                print("tidak terhubung!!!")
            #publish ke process
            msg = RobotInfo()
            msg.playerID = playerID
            msg.fall = fall
            msg.task = task
            msg.posx,msg.posy, msg.posw = position
            msg.detect = detect
            msg.ballsize = ballsize
            msg.goalorient = goalorient
            pub.publish(msg)
            # print(playerID)
    except rospy.ROSInterruptException:
        Socket.close()
        # print("a")
