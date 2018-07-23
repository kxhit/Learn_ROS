#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import socket
from std_msgs.msg import String
from sensor_msgs.msg import Joy

scale = 300

pub=rospy.Publisher('tcptopic',String,queue_size=10)
	######################tcp begining
HOST='192.168.13.13'
PORT=7998
BUFFER=4096
sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
sock.connect((HOST, PORT))


def callback(data):
    wheel1=-1*int(data.axes[3] * scale) - int(data.axes[2] * scale)
    wheel2=-1*int(data.axes[3] * scale) - int(data.axes[2] * scale)
    wheel3=int(data.axes[3] * scale) - int(data.axes[2] * scale)
    wheel4=int(data.axes[3] * scale) - int(data.axes[2] * scale)	

    string_send = "#,TWIST," + str(wheel1) + "," + str(wheel2) + "," + str(wheel3) + "," + str(wheel4) + ","
    string_send = string_send + "CAMERA," + "0" + "," + "0" + "," + "0" + "," + "0" + "," + "0" + "," + "0" + ","
    string_send = string_send + "POSITION," + "0" + "," + "0" + ","+ "0" + "," + "0" + "," + "0" + ","
    string_send = string_send + "STATE," + "0" + "," + "0" + ","+ "0" + "," + "0" + "," + "0" + "," + "0" + "," + "0" + ","
    string_send = string_send + "DEBUG," + "0" + ","
    
    string_send = string_send + "!"
    print string_send
    print "length = " + str(len(string_send))
	
    sock.send(string_send)
    time.sleep(0.05)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    #rospy.Subscriber("chatter", String, callback)
    rospy.Subscriber("joy", Joy, callback, queue_size=1)
    print "start listeren "



	

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
