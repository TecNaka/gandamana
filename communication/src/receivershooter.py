#!/usr/bin/python

import rospy
import socket
import json
from std_msgs.msg import (String)

shootPub = rospy.Publisher('/DEWO/Communication/shooter', String, queue_size = 1)        
if __name__ == '__main__':
    rospy.init_node('receivershooter')
    try:
        rospy.loginfo("GANDAMANA RECEIVER SHOOTER WILAYAH REDIII !!!")
        myIP = "192.168.1.32"
        port =  5504 #6604

        Socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        Socket.bind((myIP, port))

        print("Server Started")
        while not rospy.is_shutdown():
            data, addr = Socket.recvfrom(1024)
            data = json.loads(data.decode())
            # print("Message from: " + str(addr))
            print("RECEIVE: ", data)
            shootPub.publish(data)
    except rospy.ROSInterruptException:
        Socket.close()