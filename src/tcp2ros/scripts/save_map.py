#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import time
import roslib
import rospy
import tf
from tcp2ros.msg import rtkGPSmessage
import rosbag
import subprocess
import time


class MapSaver():
    def __init__(self):
        rospy.init_node('save_map')
        rospy.Subscriber("/rtkGPS_filter", rtkGPSmessage, self.callback1)

        self.start=0
        self.end=0
        self.num=0
        self.last_num=0
        self.count=0

        while not rospy.is_shutdown():
            if self.start==1:
                if self.num != self.last_num:
                    self.last_num=self.num

                elif self.num == self.last_num:
                    self.count = self.count+1

            if self.count == 4:
                print "map_saver"
                process=subprocess.Popen('rosrun map_server map_saver -f /home/exbot/catkin_ws/src/tcp2ros/NodeMap/map/mymap',shell=True,stdout=subprocess.PIPE)
                output,error=process.communicate()
                process.wait()

            time.sleep(1)


    def callback1(self,data):
        if self.start == 0:
            self.start = 1

        self.num = self.num+1

if __name__ == '__main__':
    try:
        MapSaver()
        rospy.spin()
    except:
        rospy.loginfo("map save terminated.")
