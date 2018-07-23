#!/usr/bin/env python
# -*- coding:utf-8 -*-

import math
import numpy as np
from numpy import *

import logging
import rospy
from tcp2ros.msg import rtkGPSmessage
from tcp2ros.msg import readDataAll

gps_length_x=0.00
gps_length_y=0.00

if __name__ == "__main__":
    pub = rospy.Publisher('rtkGPS', rtkGPSmessage, queue_size=10)
    pub1 = rospy.Publisher('tcpData', readDataAll, queue_size=10)
    rospy.init_node('testFloat', anonymous=False)
    rate = rospy.Rate(30) # 10hz

    N_POSITION = float(3349250.5000)
    E_POSITION = float(511131.6562)
    YAW_POSITION = float(0.3751)
    N_POSITION = N_POSITION + gps_length_y * math.sin(YAW_POSITION) + gps_length_x * math.cos(YAW_POSITION)

    while not rospy.is_shutdown():
        pub_Data=readDataAll()
        pub_Data.tank_id=1
        pub_Data.track_point_id=5
        pub_Data.distance_alignment=1.0
	pub_DaTa.Pillar_distance=1.0

        pub_temp = rtkGPSmessage()
        pub_temp.ROS_time = rospy.get_rostime()
        pub_temp.GPS_time = "None"
        pub_temp.vaild_flag = True
        pub_temp.flash_state = 'POSITION'
        pub_temp.north_meter = N_POSITION+gps_length_y * math.sin(YAW_POSITION) + gps_length_x * math.cos(YAW_POSITION)
        pub_temp.east_meter = E_POSITION - gps_length_y * math.cos(YAW_POSITION) + gps_length_x * math.sin(YAW_POSITION)
        pub_temp.yaw_rad = YAW_POSITION
        #pub.publish(pub_temp)
        pub1.publish(pub_Data)
        #print pub_temp.north_meter,pub_temp.east_meter

        rate.sleep()
