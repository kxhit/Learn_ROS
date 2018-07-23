#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import time
import roslib
import rospy
import tf
from std_msgs.msg import String  
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf import TransformBroadcaster 
from geometry_msgs.msg import Twist
from tcp2ros.msg import readDataAll
import rosbag
import subprocess

num=3
start=1
i=0
last_data=rospy.Time()
def callback1(msg):
    if msg.is_start_camera==2 and msg.nest_target_num!=0:
        start=1
        num=msg.next_target_num
    else:
        start=0

if __name__ =="__main__":
    rospy.init_node('car_play')
    
    rospy.Subscriber("/tcpData", readDataAll, callback1)
    print 'start record'
    
    while not rospy.is_shutdown():
        if start==1:
	    process=subprocess.Popen('rosbag record /home/exbot/test'+str(num)+'.bag',shell=True,stdout=subprocess.PIPE)
            output,error=process.communicate()
            process.wait()
	    print 'record test'+str(num)+'.bag finish'
            start=0

