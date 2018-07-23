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
import keylogger
from sensor_msgs.msg import ChannelFloat32
import virtkey
import thread

num=0
start=0
i=0
last_data=rospy.Time()
cmd_vel_pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
play_pub=rospy.Publisher('/play_msg',ChannelFloat32,queue_size=10)
stop_count=0
stop_flag=False
pause=False
key_pause=False

KeyEmulator=virtkey.virtkey()
def press(keyname):
        # 'press_unicode()' emulates the key being pressed down
        # ord() converts the character to it's unicode value
	KeyEmulator.press_unicode(ord(keyname))

        # The key stays pressed down until you tell virtkey to release
        # it with the 'release_unicode()' function. note: you have to specify
        # which key to release with the unicode value again
	KeyEmulator.release_unicode(ord(keyname))


def print_keys(t, modifiers, keys): 
    global pause
    global key_pause
    if keys==" ":
	if key_pause==False:
            key_pause=True
	    cmd=Twist()
            for i in range(3):
	        cmd.linear.x=0.0
	        cmd.angular.z=0.0
	        cmd_vel_pub.publish(cmd)
        else:
            key_pause=False


def pause_thread1(threadName,delay):
    global key_pause
    global stop_flag
    while 1:
        if start==1:
	    #if key_pause==True:
            #   print 'key pause'
            #    cmd=Twist()
	    #    #cmd.linear.x=0.0
	    #    cmd.angular.z=0.0
	    #    cmd_vel_pub.publish(cmd)
	    if stop_flag==True:
                print 'laser pause'
                cmd=Twist() 
	        cmd.linear.x=0.0
	        cmd.angular.z=0.0
	        cmd_vel_pub.publish(cmd)
        time.sleep(delay)


def pause_thread(threadName,delay):
    while 1:
        if start==1:
            now = time.time()
            done = lambda: time.time() > now + 999999
            #print 'keylogger'
            #keylogger.log(done, print_keys)


def callback1(msg):
    global start
    global num
    if msg.is_start_camera==2 and msg.next_target_num!=0:
        start=1
        num=msg.next_target_num
    #elif msg.is_start_camera==0 and msg.next_target_num==0:
        #start=0

def callback2(data):
    global stop_count
    global stop_flag
    global pause
    
    if data.name == "stop_flag" and start==1:
       if data.values[0] == 0.0:
           stop_count = stop_count+1
           if stop_count>50:
               stop_flag = False
               stop_count = 51
               #print "Not Stop"
       else:
           stop_flag = True
           stop_count = 0
           #print "STOP"
    if stop_flag and pause==False and start==1:
        press(" ")
        cmd=Twist()
        cmd.linear.x = 0
        cmd.angular.z = 0		     	     
        cmd_vel_pub.publish(cmd)
	pause=True
	
    elif stop_flag==False and pause==True and start==1:
        press(" ")
	pause=False

    return

if __name__ =="__main__":
    rospy.init_node('car_play')
    rospy.Subscriber("/tcpData", readDataAll, callback1)
    rospy.Subscriber("laser_target_position", ChannelFloat32, callback2, queue_size=1)
    try:
        #thread.start_new_thread(pause_thread, ("pause",2,))
        thread.start_new_thread(pause_thread1, ("pause1",1,))
    except:
        print "Error: unable to start thread"

    print 'start play'
    
    while not rospy.is_shutdown():
        if start==1:
            print "start play ",num
            play_data = ChannelFloat32()
            play_data.name = "play_msg"
            play_data.values.append(1.0)
            play_data.values.append(num)
            play_pub.publish(play_data)
	    process=subprocess.Popen('rosbag play /home/exbot/test'+str(num)+'.bag',shell=True,stdout=subprocess.PIPE)
            output,error=process.communicate()
            process.wait()
            play_data1 = ChannelFloat32()
            play_data1.name = "play_msg"
            play_data1.values.append(0.0)
            play_pub.publish(play_data1)
	    print 'play '+str(num)+' finish'
            num=0
            start=0

