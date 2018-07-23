#!/usr/bin/python
# -*- coding: utf-8 -*-

import time
import socket
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy

######################tcp begining
HOST='192.168.13.13'

PORT=7998

BUFFER=4096

sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)

sock.connect((HOST, PORT))


################ros begining
rospy.init_node('tcptalker',anonymous=0)
pub=rospy.Publisher('tcptopic',String,queue_size=10)

print 'i am listening'
i=0
while not rospy.is_shutdown():
    sock.send('#,TWIST,+500,+500,-100,-1000,!')
    time.sleep(0.1)
    print i
    i=i+1


