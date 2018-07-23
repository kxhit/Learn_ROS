#!/usr/bin/python
# -*- coding: utf-8 -*-

import time
import socket
import rospy
import tf
import math
from std_msgs.msg import String
from tcp2ros.msg import readDataAll
from nav_msgs.msg import Odometry
from tf import *
from geometry_msgs.msg import Quaternion,TransformStamped


other_car_x=0.0
other_car_y=0.0
other_car_theta=0.0

infrared_right=0.0
infrared_left=0.0

target_tank_id = 0

mode = 0

track_point_id = 0

first_alignment_mode = 0
laser_alignment_mode = 0
distance_laser = 1.0

pause = 0
stop = 0

Pillar_distance = 1.0

is_start_camera=0

next_target_num=0

control1=0
control2=0
control3=0
control4=0
back_home = 0
def talker():
    pub=rospy.Publisher('tcpData',readDataAll,queue_size=10)
    odom_pub=rospy.Publisher('odom',Odometry,queue_size=10)
    odom_broadcaster=tf.TransformBroadcaster()
    rospy.init_node('tcpReader',anonymous=0)


    x=0.0
    y=0.0
    th=0.0
    updated_=0.0
    
    rate = rospy.Rate(40) # 40hz
    while not rospy.is_shutdown():
        #receviedData = sock.recv(200)
        #print receviedData
        #receviedDataSplit = receviedData.split(',')
        #print receviedDataSplit


	    # other_car_x = float(receviedDataSplit[7])
	    # other_car_y = float(receviedDataSplit[8])
         #    other_car_theta = float(receviedDataSplit[9])
        #
         #    infrared_right = float(receviedDataSplit[10])
	    # infrared_left = float(receviedDataSplit[11])
        #
         #    is_start_camera= int(receviedDataSplit[13])
         #
         #    next_target_num = int(receviedDataSplit[14])
         #
         #    control1 = int(receviedDataSplit[15])
         #    control2 = int(receviedDataSplit[16])
         #    control3 = int(receviedDataSplit[17])
         #    control4 = int(receviedDataSplit[18])

        pubData = readDataAll()
        pubData.time = rospy.Time.now()
        pubData.odom1 = 0
        pubData.odom2 = 0
        pubData.odom3 = 0
        pubData.odom4 = 0
        pubData.mode = 1
        pubData.tank_id = 1
        pubData.track_point_id = 1
        pubData.first_alignment = first_alignment_mode
        pubData.laser_alignment = 3
        pubData.distance_alignment = 5.0
        pubData.Pillar_distance = Pillar_distance
        pubData.back_home = 0
        pubData.pause = pause
        pubData.stop = stop
        pubData.is_start_camera = 1
        # pubData.auto = auto
        # pubData.other_car_x=other_car_x
        # pubData.other_car_x=other_car_x
        # pubData.other_car_y=other_car_y
        # pubData.other_car_theta=other_car_theta
        # pubData.infrared_right=infrared_right
        # pubData.infrared_left=infrared_left
        # pubData.is_start_camera=is_start_camera
        # pubData.next_target_num = next_target_num
        # pubData.control1 = control1
        # pubData.control2 = control2
        # pubData.control3 = control3
        # pubData.control4 = control4

        #print odom1,odom2,odom3,odom4,other_car_x,other_car_y,other_car_theta,infrared_right,infrared_left,is_start_camera,next_target_num,control1,control2,control3,control4
        pub.publish(pubData)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
