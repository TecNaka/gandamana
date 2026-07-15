#!/usr/bin/python
#-*- coding:utf-8 -*-
import rospy
import socket
import time
import json
from std_msgs.msg import String
enabel = ""
def send_Callback(data):
    global enabel
    enabel = data.data

if __name__ == '__main__':
    rospy.init_node("robotkickoff")
    rospy.Subscriber("/DEWO/Communication/kickoff", String, send_Callback)
    try:
        rospy.loginfo("GANDAMANA SENDER KICKOFF WILAYAH REDIII !!!")
        host='192.168.1.32' #IP KITA (ROBOT2)
        port = 6604

        robot2 = ('192.168.1.33', 6604)#IP, PORT TUJUAN(ROBOT3)
        # robot1 = ('192.168.1.35', 6604)#IP, PORT TUJUAN(ROBOT5)

        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        while not rospy.is_shutdown():
            msg = json.dumps(enabel)
            s.sendto(msg.encode(), robot2)#robot2 = formasi3, robot1 = formasi2
            print("SEND: ", enabel)
            time.sleep(0.1) 
    except rospy.ROSInterruptException:
        s.close()
