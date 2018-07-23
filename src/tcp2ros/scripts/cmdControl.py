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
from tcp2ros.msg import rtkGPSmessage
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu

scale = 300

pub = rospy.Publisher('tcptopic', String, queue_size=10)
#HOST = '192.168.137.1'
HOST = '192.168.13.13'
PORT = 7998
BUFFER = 4096
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

a = 0.367
b = 0.352

camera_roll = 0.0
camera_pitch = 0.0
camera_yaw = 0.0
camera_x = 0.0
camera_y = 0.0
camera_z = 0.0

x = 0.0
y = 0.0
theta = 0.0

map_x = 0.0
map_y = 0.0

sonar_x = 0.0
sonar_y = 0.0

is_start_program = 0
is_start_imu = 0
is_start_laser = 0
is_start_camera = 0
is_gps_vaild_flag = 0
is_sonar = 0
is_amcl = 0
is_infrared = 0
current_target = 0
finish_target = 0
debug_cmd = 0

wheel1 = 0
wheel2 = 0
wheel3 = 0
wheel4 = 0

Position_Now_rtkGPS = []

roll = 0
pitch = 0
yaw = 0

is_odom_work = 0
# state
GPS_Position_State = 0
# Laser_Position_state = 0
Coarse_Alignment_state = 0
Laser_Alignment_State = 0
Current_MapPoint_id = 0
Target_MapPoint_id = 0
Pause_State = 0
Barrier = 0

is_FunctionAll_work = 0
is_laser_search_work = 0
is_laserprocess_work = 0
is_odom_imu_gps_start = 0
is_path_publish_start = 0
is_pub_map_pose_start = 1


def play_callback(msg):
    global is_infrared
    global current_target
    if msg.name == "play_msg":
        if msg.values[0] == 1:
            is_infrared = 2
            current_target = int(msg.values[1])
        elif msg.values[0] == 0:
            is_infrared = 2
            current_target = 0


def record_callback(msg):
    global is_infrared
    global current_target
    if msg.name == "record_msg":
        if msg.values[0] == 1:
            is_infrared = 1
            current_target = int(msg.values[1])
        elif msg.values[0] == 0:
            is_infrared = 1
            current_target = 0


def callback1(data):
    global is_start_laser
    if len(data.ranges) > 0:
        is_start_laser = 1


def callback(cmd):
    global wheel1
    global wheel2
    global wheel3
    global wheel4

    delta_x = cmd.linear.x
    delta_y = 0
    delta_th = cmd.angular.z
    if delta_x == 0 and delta_th != 0:
	if delta_th < 0:
	    wheel1 = 130
	    wheel2 = 30
	    wheel3 = 130
	    wheel4 = 30
	else:
	    wheel1 = -130
	    wheel2 = -30
	    wheel3 = -130
	    wheel4 = -30	    
        #wheel1 = -1 * int((delta_x + delta_y + delta_th * (a + b)) * scale)*3.0
        #wheel2 = -1 * int((delta_x + delta_y + delta_th * (a + b)) * scale)
        #wheel3 = int((delta_x + delta_y - delta_th * (a + b)) * scale)*3.0
        #wheel4 = int((delta_x + delta_y - delta_th * (a + b)) * scale)
    elif delta_x == 0 and delta_th == 0 :
        wheel1 = 0
	wheel2 = 0
	wheel3 = 0
	wheel4 = 0       
    else:    
        wheel1 = -1 * int((delta_x + delta_y + delta_th * (a + b)) * scale)
        wheel2 = -1 * int((delta_x + delta_y + delta_th * (a + b)) * scale)
        wheel3 = int((delta_x + delta_y - delta_th * (a + b)) * scale)
        wheel4 = int((delta_x + delta_y - delta_th * (a + b)) * scale)

    print "wheel1:       "
    print wheel1
    print "wheel2:       "
    print wheel2
    print "wheel3:       "
    print wheel3
    print "wheel4:       "
    print wheel4


def image_callback(data):
    global camera_yaw
    global camera_y
    global camera_pitch
    global camera_roll
    global camera_x
    global camera_z
    camera_x = round(data.values[0],3)
    camera_y = round(data.values[1],3)
    camera_z = round(data.values[2],3)
    camera_roll = round(data.values[3],3)
    camera_pitch = round(data.values[4],3)
    camera_yaw = round(data.values[5],3)


def rtkGPS_callback(data):
    global theta
    global x
    global y
    global is_gps_vaild_flag
    global is_odom_work
    if data.flash_state == 'YAW':
        theta = data.yaw_rad
    if data.flash_state == 'POSITION':
        x = data.north_meter
        y = data.east_meter
    if data.vaild_flag == True:
        is_gps_vaild_flag = 1
        is_odom_work = 0
    else:
        if data.north_meter == 0 and data.east_meter == 0:
            is_gps_vaild_flag = 0
            is_odom_work = 0
        else:
            is_gps_vaild_flag = 0
            is_odom_work = 1


def camera_state_callback(data):
    global is_start_camera
    if data.name == "camera_state":
        is_start_camera = data.values[0]


def state_callback(data):
    global GPS_Position_State
    global Laser_Alignment_State
    global Coarse_Alignment_state
    global Current_MapPoint_id
    global Target_MapPoint_id
    global Pause_State
    global Barrier
    if data.name == "Position_State_GPS":
        GPS_Position_State = data.values[0]
    if data.name == "Coarse_Alignment_State":
        Coarse_Alignment_state = data.values[0]
    if data.name == "Laser_Alignment_State":
        Laser_Alignment_State = data.values[0]
    if data.name == "Point_Id":
        Current_MapPoint_id = data.values[0]
        Target_MapPoint_id = data.values[1]
    if data.name == "Pause_State":
        Pause_State = data.values[0]
    if data.name == "BarrierBarrier":
        Barrier = data.values[0]


def map_pose_callback(data):
    global map_x
    global map_y
    map_x = data.pose.position.x
    map_y = data.pose.position.y


def imu_callback(data):
    global is_start_imu
    is_start_imu = 1


def start_odom_imu_callback(data):
    global is_odom_imu_gps_start
    is_odom_imu_gps_start = 1


def start_path_publish_callback(data):
    global is_path_publish_start
    is_path_publish_start = 1





def laserprocess_callback(data):
    global is_laserprocess_work
    is_laserprocess_work = 1


def laser_search_callback(data):
    global is_laser_search_work
    is_laser_search_work = 1


def FunctionAll_callback(data):
    global is_FunctionAll_work
    is_FunctionAll_work = 1


def listener():
    global debug_cmd
    global is_start_program
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    # rospy.Subscriber("chatter", String, callback)
    rospy.Subscriber("cmd_vel", Twist, callback, queue_size=5)
    rospy.Subscriber("/play_msg", ChannelFloat32, play_callback, queue_size=5)
    rospy.Subscriber("/record_msg", ChannelFloat32, record_callback, queue_size=5)
    rospy.Subscriber("base_scan", LaserScan, callback1)
    rospy.Subscriber("/rtkGPS_filter", rtkGPSmessage, rtkGPS_callback)
    # rospy.Subscriber("/imu_angle", ChannelFloat32, imu_callback, queue_size=1)
    rospy.Subscriber("image_result", ChannelFloat32, image_callback, queue_size=5)
    rospy.Subscriber("camera_state", ChannelFloat32, camera_state_callback, queue_size=5)
    rospy.Subscriber("Working_state", ChannelFloat32, state_callback, queue_size=5)
    # rospy.Subscriber("rtkGPS_filter",rtkGPSmessage,gps_state_callback,queue_size=5)

    rospy.Subscriber("/map_pose", PoseStamped, map_pose_callback, queue_size=5)
    rospy.Subscriber("/imu/data", Imu, imu_callback, queue_size=5)

    rospy.Subscriber("/odom_imu_gps_start", ChannelFloat32, start_odom_imu_callback, queue_size=5)
    rospy.Subscriber("/path_publish_start", ChannelFloat32, start_path_publish_callback, queue_size=5)
    #rospy.Subscriber("/pub_map_pose", ChannelFloat32, start_pub_map_pose_callback, queue_size=5)
    rospy.Subscriber("is_laserprocess_work", ChannelFloat32, laserprocess_callback, queue_size=5)
    rospy.Subscriber("is_laser_search_work", ChannelFloat32, laser_search_callback, queue_size=5)
    rospy.Subscriber("is_FunctionAll_work", ChannelFloat32, FunctionAll_callback, queue_size=5)

    print "start listeren "

    debug_cmd = 1

    while not rospy.is_shutdown():

        #print "is_odom_imu_gps_start"
        #print is_odom_imu_gps_start
        #print "is_path_publish_start"
        #print is_path_publish_start
        #print "is_pub_map_pose_start"
        #print is_pub_map_pose_start
        #print "is_laserprocess_work"
        #print is_laserprocess_work
        #print "is_laser_search_work"
        #print is_laser_search_work
        #print "is_FunctionAll_work"
        #print is_FunctionAll_work
	#print "is_start_program"
	#print is_start_program
	#print "map_x"
	#print map_x
	#print "map_y"
	#print map_y
	#print "********************************"
	#print is_gps_vaild_flag
        if is_odom_imu_gps_start == 1 and is_path_publish_start == 1 and is_pub_map_pose_start == 1 and is_laserprocess_work == 1 and is_laser_search_work == 1 and is_FunctionAll_work == 1:
            is_start_program = 1

        string_send = "#,TWIST," + str(wheel1) + "," + str(wheel2) + "," + str(wheel3) + "," + str(wheel4) + ","
        string_send = string_send + "CAMERA," + str(camera_roll) + "," + str(camera_pitch) + "," + str(
            camera_yaw) + "," + str(camera_x) + "," + str(camera_y) + "," + str(camera_z) + ","
        string_send = string_send + "POSITION," + str(x) + "," + str(y) + "," + str(theta) + "," + str(
            map_x) + "," + str(map_y) + ","
        string_send = string_send + "STATE," + str(is_start_program) + "," + str(is_start_imu) + "," + str(
            is_start_laser) + "," + str(is_start_camera) + "," + str(is_gps_vaild_flag) + "," + str(
            GPS_Position_State) + "," + str(Coarse_Alignment_state) + "," + str(Laser_Alignment_State) + "," + str(
            is_odom_work) + "," + str(Pause_State) + "," + str(Current_MapPoint_id) + "," + str(
            Target_MapPoint_id) + "," + str(Barrier) + ","

        string_send = string_send + "!"
        print string_send
        # print "length = " + str(len(string_send))
        sock.send(string_send)

        time.sleep(0.05)
        # spin() simply keeps python from exiting until this node is stopped
        # rospy.spin()


if __name__ == '__main__':
    listener()
