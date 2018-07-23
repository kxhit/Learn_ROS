#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import socket
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import ChannelFloat32
from sensor_msgs.msg import LaserScan

scale = 300

pub=rospy.Publisher('tcptopic',String,queue_size=10)
	######################tcp begining
HOST='192.168.13.13'
PORT=7998
BUFFER=4096
sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
sock.connect((HOST, PORT))

a=0.367
b=0.352

camera_roll=0.0
camera_pitch=0.0
camera_yaw=0.0
camera_x=0.0
camera_y=0.0
camera_z=0.0

x=0.0
y=0.0
theta=0.0

sonar_x=0.0
sonar_y=0.0

is_start_camera=0
is_gps_vaild_flag=0
is_sonar=0
is_amcl=0
is_infrared=0
current_target=0
finish_target=0
debug_cmd=0

wheel1=0
wheel2=0
wheel3=0
wheel4=0

def play_callback(msg):
    global is_infrared
    global current_target
    if msg.name == "play_msg":
        if msg.values[0] == 1:
            is_infrared=2
            current_target=msg.values[1]
        elif msg.values[0] == 0:
            is_infrared=0
            current_target=0

def record_callback(msg):
    global is_infrared
    global current_target
    if msg.name == "record_msg":
        if msg.values[0] == 1:
            is_infrared=1
            current_target=msg.values[1]
        elif msg.values[0] == 0:
            is_infrared=0
            current_target=0

def callback1(data):
    global is_amcl
    if len(data.ranges)>0:
        is_amcl=1
    
def callback(cmd):
    global wheel1
    global wheel2
    global wheel3
    global wheel4

    delta_x=cmd.linear.x
    delta_y=0
    delta_th=cmd.angular.z

    wheel1=-1*int((delta_x+delta_y+delta_th*(a+b))*scale)
    wheel2=-1*int((delta_x+delta_y+delta_th*(a+b))*scale)
    wheel3=int((delta_x+delta_y-delta_th*(a+b))*scale)
    wheel4=int((delta_x+delta_y-delta_th*(a+b))*scale)

    string_send = "#,TWIST," + str(wheel1) + "," + str(wheel2) + "," + str(wheel3) + "," + str(wheel4) + ","
    string_send = string_send + "CAMERA," + str(camera_roll) + "," + str(camera_pitch) + "," + str(camera_yaw) + "," + str(camera_x) + "," + str(camera_y) + "," + str(camera_z) + ","
    string_send = string_send + "POSITION," + str(x) + "," + str(y) + "," + str(theta) + "," + str(sonar_x) + "," + str(sonar_y) + ","
    string_send = string_send + "STATE," + str(is_start_camera) + "," + str(is_gps_vaild_flag) + ","+ str(is_sonar) + "," + str(is_amcl) + "," + str(is_infrared) + "," +str(current_target) +  "," + str(finish_target) + ","
    string_send = string_send + "DEBUG," + str(debug_cmd) + ","
    
    string_send = string_send + "!"
    print string_send
    #print "length = " + str(len(string_send))
	
    sock.send(string_send)
    time.sleep(0.05)
    
def listener():
    global debug_cmd
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    #rospy.Subscriber("chatter", String, callback)
    rospy.Subscriber("cmd_vel", Twist, callback, queue_size=1)
    rospy.Subscriber("/play_msg", ChannelFloat32, play_callback, queue_size=1)
    rospy.Subscriber("/record_msg", ChannelFloat32, record_callback, queue_size=1)
    rospy.Subscriber("base_scan", LaserScan, callback1)
    print "start listeren "

    debug_cmd=1
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
