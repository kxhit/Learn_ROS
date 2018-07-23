#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity
import rospy
import numpy as np
import Motion_Planning_go
import Motion_Planning_rotate
import Motion_Planning_laser
import Map_Point
import LaserProcess_new_filter_new
import TF
import tf
import time
import math
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from tcp2ros.msg import rtkGPSmessage
from tcp2ros.msg import readDataAll
from sensor_msgs.msg import ChannelFloat32
from nav_msgs.msg import Path
from geometry_msgs.msg import Quaternion
from tf import transformations
from tcp2ros.msg import reach
from std_msgs.msg import Float32
def loadNodeMapPoint():
    fr=open('/home/exbot/catkin_ws/src/tcp2ros/NodeMap/NodeMap.txt')
    i=0
    numNode=0
    NodeMapPoint=[]
    for line in fr.readlines():
        lineArr=line.strip().split()
        if i==0:
            numNode=int(lineArr[0])
        if i<=numNode:
            if i==1 or i==2 or i==3 or i==4:
                NodeMapPoint.append([int(lineArr[0]),float(lineArr[1]),float(lineArr[2]),float(lineArr[3])])
        i+=1
    return NodeMapPoint

def caculateYaw(y0,x0):
    x = np.array([y0,x0])
    y = np.array([0,1])
    Lx = np.sqrt(x.dot(x))
    Ly = np.sqrt(y.dot(y))
    cos_angle = x.dot(y)/(Lx*Ly)
    angle = np.arccos(cos_angle)
    if(y0 < 0):
        angle = -angle
    return angle


class functionAll:
     def __init__(self):
         rospy.init_node('FunctionAll', anonymous=False)
         self.FunctionAll_workstate_pub = rospy.Publisher("is_FunctionAll_work",ChannelFloat32,queue_size=5)
         #self.drive_pub = rospy.Publisher("is_drive_work",ChannelFloat32,queue_size=5)

         #self.camera_pub = rospy.Publisher('camera_start',ChannelFloat32, queue_size=1)
         self.rate = rospy.Rate(30)
         self.cmd = Twist()
         self.count = 0
         self.num = 0
         self.last_point_update = False
         self.last_point_updated = False
         self.add_coarse_point = False
         self.last_point_search = 0
         self.cmd.linear.x = 0.0
         self.cmd.linear.y = 0.0
         self.cmd.linear.z = 0
         self.cmd.angular.z = 0.0
         self.run_state = 0#0 is going to map point. 1 is going to angle
         self.run_laser = False # false is use gps information to go  else use laser
         self.stop_flag = False#if laser find there is something before it
         self.goto_real_target = False #--[flag]go to the real target
         self.add_true_target = False #--[flag] add the GPS of target
         self.stop_count = 0
         self.target_distance = 1.0 #--save the distance from car to the middle point
         self.back_count = False #-- publish back message when it's true

         self.add_last_point = False
         self.initial_yaw = 0
         self.err_angle = 1
         self.imu_nav = False
         self.imu_count = 0
         self.target_disappear = False
         self.disappear_count = 0
         self.search_target = False
         self.control_mode = 1
         #self.control_mode = 0
         self.laser_mode = 1
         self.Pause = 0
         self.Calculate_distance = 0
         self.back_home = 0
         self.detect = 0
         self.search_flag = False
         self.search_second = False
         self.done = 0
         self.refresh = 0
         self.get_yaw = 0
         self.get_position = 0
         self.search_count = 0
         self.wait = 0
         self.second_search_count = 0
         self.first_search_count = 0
         self.True_count = 0
         self.add_new = 0
         self.last_updated = 0
         self.start = False
         self.update_ban = False
         self.last_workstate = 0

         self.BackHomeNodeMapPoint=loadNodeMapPoint()

         #callback6 message filter START
         self.messageFilter_tank_id = 0
         self.messageFilter_track_point_id = 0
         self.messageFilter_first_alignment = 0
         self.messageFilter_laser_alignment = 0
         self.messageFilter_stop = 0
         self.messageFilter_back_home = 0
         self.messageFilter_is_start_camera = 0
         self.messageFilter_alignment_distance = 0

         self.messageFilter_tank_id_before = 0
         self.messageFilter_track_point_id_before = 0
         self.messageFilter_first_alignment_before = 0
         self.messageFilter_laser_alignment_before = 0
         self.messageFilter_stop_before = 0
         self.messageFilter_back_home_before = 0
         self.messageFilter_is_start_camera_before = 0
         self.messageFilter_alignment_distance_before = 0

         self.messageFilter_workingState = 0#0 is free ,1 is working ,2 is camera working



         #callback6 message filter END

         self.GPS_Position_State = ChannelFloat32()
         self.GPS_Position_State.name = "Position_State_GPS"
         self.GPS_Position_State.values.append(0)


         self.Laser_Position_state = ChannelFloat32()
         self.Laser_Position_state.name = "Position_State_Laser"
         self.Laser_Position_state.values.append(0)

         self.Coarse_Alignment_state = ChannelFloat32()
         self.Coarse_Alignment_state.name = "Coarse_Alignment_State"
         self.Coarse_Alignment_state.values.append(0)

         self.Imu_Nav_State = ChannelFloat32()
         self.Imu_Nav_State.name = "Imu_Nav_State"
         self.Imu_Nav_State.values.append(0)

         self.Laser_Alignment_State = ChannelFloat32()
         self.Laser_Alignment_State.name = "Laser_Alignment_State"
         self.Laser_Alignment_State.values.append(0)

         self.MapPoint_id = ChannelFloat32()
         self.MapPoint_id.name = "Point_Id"
         self.MapPoint_id.values.append(0)
         self.MapPoint_id.values.append(0)

         self.Camera_start = ChannelFloat32()
         self.Camera_start.name = "Camera_State"
         self.Camera_start.values.append(0)

         # self.GPS_Navigation = ChannelFloat32()
         # self.GPS_Navigation.name = "GPS_Navigation_State"
         # self.GPS_Navigation.values[0] = 0

         self.laser_alignment_workstate = ChannelFloat32()
         self.laser_alignment_workstate.name = "laser_alignment"

         self.FunctionAll_workstate = ChannelFloat32()
         self.FunctionAll_workstate.name = "working"

         #self.laser_alignment_workstate.values.append(0)

         # self.drive_workstate = ChannelFloat32()
         # self.drive_workstate.name = "driving"

         self.Pause_State = ChannelFloat32()
         self.Pause_State.name = "Pause_State"
         self.Pause_State.values.append(0)

         self.Barrier_State = ChannelFloat32()
         self.Barrier_State.name = "Barrier"
         self.Barrier_State.values.append(0)

         self.debug_print_count=0
         self.MapPoint_Num_Update = 10000 #-- set a big initial value
         self.yaw_new = 0
         self.Map_select = 0 #-- mark as the option of MapPoint list
         self.MapPoint_Temp = [] #-- save the
         self.last_point_temp_x = 0
         self.last_point_temp_y = 0
         self.last_point_temp_yaw = 0
         self.Motion_rotate_temp = []
         self.Motion_temp = []
         self.laser_count = 0 #-- laser counter,when the car reach the target,laser work
         self.angle_temp = 0
         # self.MapPointNumLaserIndex = [4,11,30]#id is laser search point
         self.MapPointNumLaserIndex = []#id is laser search point
         self.MapPointNum_Temp = 0
         self.init_camera = 1

         self.MotionPlaner_speedXY = float(rospy.get_param("MotionPlaner_speedXY",'0.3'))
         self.MotionPlaner_speedYaw = float(rospy.get_param("MotionPlaner_speedYaw",'0.2'))
         self.MotionPlaner_errorMeter = float(rospy.get_param("MotionPlaner_errorMeter",'0.3'))#m
         self.MotionPlaner_errorAngle = float(rospy.get_param("MotionPlaner_errorAngle",'0.2'))#rad

         self.MotionPlaner = Motion_Planning_go.Motion_Planning(0.8, self.MotionPlaner_speedYaw, self.MotionPlaner_errorMeter, 0.15)
         self.MotionPlaner_laser = Motion_Planning_laser.Motion_Planning(0.2,0.15,0.08,0.075)
         self.Motion_Planer_rotate = Motion_Planning_rotate.Motion_Planning(0.8, self.MotionPlaner_speedYaw, self.MotionPlaner_errorMeter, self.MotionPlaner_errorAngle)
         #self.MotionPlaner.setValues(0.2,0.2,0.05,0.042)
         self.MapPoint = Map_Point.Map_Point()
         self.MapPoint_New = Map_Point.Map_Point()
         self.TF_body = TF.TF()
         #MapPoint is in rtkGPS coordinate system

         self.North_base = float(rospy.get_param("North_base",'0.2'))
         self.East_base = float(rospy.get_param("East_base",'0.2'))
         self.Yaw_base = float(rospy.get_param("Yaw_base",'0.0'))
         self.MapPointNum_Now = 0
         #TEST
         # self.MapPoint.add_point(3349243.9373, 511128.9789, 0.4196,1)
         # self.MapPoint.add_point(3349247.9593, 511130.5813, 0.3891,2)
         # self.MapPoint.add_point(3349250.2322, 511131.4639, 0.3641,3)
         #右侧 0
         #self.MapPoint.add_point(3349261.0467, 511135.5801, 1.9044,4)
         #左侧 1
         #self.MapPoint.add_point(3349261.1558, 511135.5988, 5.1007,4)

         #真实MapPoint
         #self.MapPoint.add_point(3349258.9099, 511134.7793, 0.3613,4)

         #tmp1代表的是加入的倒数第二个MapPoint
         #tmp1 = self.MapPoint.get_point(2)
         #tmp2代表的是加入的最后一个MapPoint
         #tmp2 = self.MapPoint.get_point(3)
         #atan2(y,x)
         # a = math.atan2(1,2)
         # print "a: "
         # print a

         #print tmp2[2]
         # yaw_new = math.atan2((tmp2[1] - tmp1[1]),(tmp2[0] - tmp1[0]))
         # print math.pi
         # if tmp2[2] > math.pi:
         #     tmp2[2] = -(math.pi - (tmp2[2] - math.pi))
         #     print tmp2[2]
         # elif tmp2[2] < -math.pi:
         #     tmp2[2] = math.pi + (math.pi + tmp2[2])
         # else:
         #     tmp2[2] = tmp2[2]
         # print tmp2[2]
         # if yaw_new - tmp2[2] < 0:
         #     self.direction = 0
         # else:
         #     self.direction = 1
         #
         # print self.direction

         #self.MapPoint.add_point(4630839.0000, 786262.0625, 1.5672,3)


         # self.MapPoint.add_point(4630832.0000, 786267.4375, 1.7700)
         # self.MapPoint.add_point(4630833.0000, 786267.6250, 3.1416)
         # #laser5
         # self.MapPoint.add_point(4630832.5000, 786268.3750, 4.6559)
         # self.MapPoint.add_point(4630832.0000, 786263.6250, 4.6295)
         # self.MapPoint.add_point(4630831.5000, 786263.5000, 6.2556)
         # self.MapPoint.add_point(4630838.0000, 786263.1250, 6.2230)
         # self.MapPoint.add_point(4630838.0000, 786262.3125, 1.5548)
         # self.MapPoint.add_point(4630838.5000, 786267.7500, 1.5243)
         # self.MapPoint.add_point(4630839.0000, 786267.9375, 3.1604)
         # #laser12
         # self.MapPoint.add_point(4630839.0000, 786268.6875, 4.6752)
         # self.MapPoint.add_point(4630838.5000, 786263.0625, 4.6457)
         # self.MapPoint.add_point(4630838.5000, 786262.3750, 3.1260)
         # self.MapPoint.add_point(4630823.0000, 786263.1250, 3.1930)
         # self.MapPoint.add_point(4630822.0000, 786263.5625, 0.0486)

         #self.MapPoint.add_point(3349259.83,  511222.59,3.47215795517)


         self.MapPointNum_All = len(self.MapPoint.pointlist_x)

         #到时候要删掉
         # self.MapPointNum_Temp = self.MapPointNum_All - 1
         # self.MapPointNumLaserIndex.append(self.MapPointNum_All)
         #print self.MapPointNum_All 


         self.Position_now_rtkGPS = [self.North_base+0, self.East_base+0,self.Yaw_base]
         self.EnableFlag = False
         self.EnableCount = [0,0]
         self.Position_BaseLink_Target=[0,0,0]

         self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
         self.reach_pub = rospy.Publisher('reach', reach, queue_size=5)
         self.back_pub = rospy.Publisher('back',ChannelFloat32,queue_size=5) #--publish back messages
         self.laser_find_none_pub = rospy.Publisher('laser_find_none', ChannelFloat32, queue_size=5)
         self.working_state_pub = rospy.Publisher('Working_state', ChannelFloat32, queue_size=5)
         self.distance_alignment_pub = rospy.Publisher('laser_alignment_distance', Float32, queue_size=5)
         self.change_radius_pub = rospy.Publisher('change_radius', Float32, queue_size=5)

         time.sleep(10)
         rospy.Subscriber("joy", Joy, self.callback1)
         rospy.Subscriber("rtkGPS_filter", rtkGPSmessage, self.callback2, queue_size=1)
         rospy.Subscriber("laser_target_position", ChannelFloat32, self.callback3, queue_size=1)
         rospy.Subscriber("path_node", Path, self.callback4, queue_size=1)
         #rospy.Subscriber("/imu_angle", ChannelFloat32, self.callback5, queue_size=1)
         rospy.Subscriber("/tcpData", readDataAll, self.callback6, queue_size=1)
         rospy.Subscriber("SearchTargetPoint",ChannelFloat32,self.callback7,queue_size=5)
         #rospy.Subscriber("pause", ChannelFloat32, self.callback7, queue_size=1) #receive
         while not rospy.is_shutdown():
             self.FunctionAll_workstate_pub.publish(self.FunctionAll_workstate)
             self.rate.sleep()

     def callback1(self, data):
         print data

     def callback2(self, data):
         #print "back home"
         #print self.back_home
         #更新gps坐标
         if self.last_point_updated == False and self.start == True:
             updated = rospy.get_time()
             if (updated - self.last_updated) > 15:
                 self.Pause = 0
                 self.start = False
                 print "overtime ------- "
         if data.flash_state == 'YAW':
             if data.north_meter != 0 and data.east_meter !=0 :
                 self.Position_now_rtkGPS[2] = data.yaw_rad
                 self.EnableCount[0] = self.EnableCount[0] + 1
                 self.get_yaw = 1
                 #print "self.EnableCount[0]"
                 #print self.EnableCount[0]
                 #print "get yaw"
         if data.flash_state == 'POSITION':
             if data.north_meter != 0 and data.east_meter !=0 :
                 self.Position_now_rtkGPS[0] = data.north_meter
                 self.Position_now_rtkGPS[1] = data.east_meter
                 self.EnableCount[1] = self.EnableCount[1] + 1
                 self.get_position = 1
                 #print "self.EnableCount[1]"
                 #print self.EnableCount[1]
                 #print "get position"
         #当同时有效时才更新gps坐标
         if self.get_yaw == 1 and self.get_position == 1:
             self.refresh = 1
         if data.flash_state == 'Data_Valid_Fault':
             self.EnableFlag = False
             self.EnableCount[0] = 0
             self.EnableCount[1] = 0
         #print "pause"
         #print self.Pause
         if self.Pause == 0:
             #self.control_mode = 0
             self.Pause_State.values[0] = 0
             self.working_state_pub.publish(self.Pause_State)
             #print "ok?"
             #print self.MapPointNum_Now
             #print self.MapPointNum_All
             #当前点的序号小于点的总数时才继续走点程序
             if self.MapPointNum_Now < self.MapPointNum_All:
                 if self.MapPointNum_Now != 0 and self.MapPointNum_Now <=len(self.MapPoint.pointlist_x) :
                     if self.MapPoint.get_id(self.MapPointNum_Now - 1) == 3 and self.back_home == 1 and self.add_new == 0 :
                         print "reach 3"
                         print self.search_flag
                         if self.search_flag == False:
                             self.True_count = 0
                             self.first_search_count = self.first_search_count + 1
                             if self.first_search_count > 100:
                                 self.first_search_count = 0
                                 self.MapPoint.pointlist_x = []
                                 self.MapPoint.pointlist_y = []
                                 self.MapPoint.pointlist_yaw = []
                                 self.MapPoint.id = []

                                 self.Map_select = 0
                                 self.done = 0
                                 self.add_true_target = False
                                 self.run_laser = False
                                 #temp = self.MapPoint.select_MapPoint(4)
                                 #self.MapPoint.add_point(4630832.3913, 786262.4277, 3.1400,4)
                                 self.MapPoint.add_point(self.BackHomeNodeMapPoint[0][1],self.BackHomeNodeMapPoint[0][2],self.BackHomeNodeMapPoint[0][3],self.BackHomeNodeMapPoint[0][0])
                                 #self.MapPoint.add_point(self.BackHomeNodeMapPoint[1][1],self.BackHomeNodeMapPoint[1][2],self.BackHomeNodeMapPoint[1][3],self.BackHomeNodeMapPoint[1][0])

                                 #self.MapPoint.add_point(4630832.3915, 786260.9810, 0.0672,3)
                                 #self.MapPoint.add_point(4630830.3973, 786261.0010, 6.2703,2)

                                 #temp = self.MapPoint.select_MapPoint(3)
                                 #self.MapPoint.add_point(temp[0], temp[1], temp[2], temp[3])

                                 #temp = self.MapPoint.select_MapPoint(2)
                                 #self.MapPoint.add_point(temp[0], temp[1], temp[2], temp[3])

                                 self.MapPointNumLaserIndex = []
                                 self.Map_select = 0

                                 self.MapPointNum_All = len(self.MapPoint.pointlist_x)
                                 self.MapPointNumLaserIndex.append(self.MapPointNum_All)
                                 self.MapPointNum_Now = 0
                                 self.add_new = 1
                                 self.wait = 1
                                 print "no cc"
                                 print "add_new_point"
                                 print self.MapPointNum_All
                                 return
                             else:
                                 return
                         else:
                             self.first_search_count = 0
                             self.True_count = self.True_count + 1
                             if self.True_count < 100:
                                 print "wait"
                                 return
                             else:
                                 self.True_count = 0

                                 self.Map_select = 0
                                 self.done = 0
                                 self.add_true_target = False
                                 self.run_laser = False

                                 self.MapPoint.pointlist_x = []
                                 self.MapPoint.pointlist_y = []
                                 self.MapPoint.pointlist_yaw = []
                                 self.MapPoint.id = []

                                 #temp = self.MapPoint.select_MapPoint(2)
                                 #self.MapPoint.add_point(temp[0], temp[1], temp[2], temp[3])
                                 #self.MapPoint.add_point(4630830.3973, 786261.0010, 6.2703,2)
                                 self.MapPoint.add_point(self.BackHomeNodeMapPoint[1][1],self.BackHomeNodeMapPoint[1][2],self.BackHomeNodeMapPoint[1][3],self.BackHomeNodeMapPoint[1][0])
                                 self.Map_select = 0
                                 self.MapPointNumLaserIndex = []

                                 self.MapPointNum_All = len(self.MapPoint.pointlist_x)
                                 self.MapPointNumLaserIndex.append(self.MapPointNum_All)
                                 self.MapPointNum_Now = 0
                                 print "--------------------------------------------------"
                                 print "cc exit gogogo"
                                 print "add_new_point"
                                 print self.MapPointNum_All
                                 return

                 if self.EnableCount[0] > 10 and self.EnableCount[1] > 10 :
                     self.EnableFlag = True
                     self.EnableCount[0] = 11
                     self.EnableCount[1] = 11
                     #print self.EnableFlag
                 if self.EnableFlag and self.run_laser == False:
                     #选择原来的map列表
                     if self.Map_select == 0:
                         #print "Map_select 0"
                         #print self.MapPointNum_Now
                         #print self.MapPoint.get_point(self.MapPointNum_Now)
                         #将gps坐标转换为车体坐标系下
                         self.Position_BaseLink_Target=self.TF_body.rtkGPStoBaseLink(self.MapPoint.get_point(self.MapPointNum_Now), self.Position_now_rtkGPS)
                         self.debug_print_count =  self.debug_print_count+1
                     # 选择更新之后的map列表
                     if self.Map_select ==1 :
                         #print "Map_select 1"
                         self.Position_BaseLink_Target=self.TF_body.rtkGPStoBaseLink(self.MapPoint_New.get_point(self.MapPointNum_Now), self.Position_now_rtkGPS)
                         self.debug_print_count =  self.debug_print_count+1
                     if self.debug_print_count >=10:
                         #print self.Position_BaseLink_Target
                         self.debug_print_count=0
                     #设置车体当前坐标为（0,0,0）
                     self.MotionPlaner.setNowPosition([0,0,0])
                     #设置设置下一个点的坐标
                     self.MotionPlaner.setNextTarget(self.Position_BaseLink_Target)

                     #另外一个Planner
                     self.Motion_Planer_rotate.setNowPosition([0,0,0])
                     self.Motion_Planer_rotate.setNextTarget(self.Position_BaseLink_Target)
                     #print self.Position_BaseLink_Target
                     self.Motion_temp = self.MotionPlaner.MotionPlan()#get speed wanted
                     self.Motion_rotate_temp = self.Motion_Planer_rotate.MotionPlan()

                     #发布工作状态
                     if self.MapPoint_Num_Update == 10000:
                         self.GPS_Position_State.values[0] = 1
                         self.working_state_pub.publish(self.GPS_Position_State)
                     else:
                         self.GPS_Position_State.values[0] = 0
                         self.working_state_pub.publish(self.GPS_Position_State)
                     self.Laser_Alignment_State.values[0] = 0
                     self.working_state_pub.publish(self.Laser_Alignment_State)

                     if self.MapPointNum_All == self.MapPoint_Num_Update and self.run_laser == False:
                         self.Coarse_Alignment_state.values[0] = 1
                         self.working_state_pub.publish(self.Coarse_Alignment_state)
                     else:
                         self.Coarse_Alignment_state.values[0] = 0
                         self.working_state_pub.publish(self.Coarse_Alignment_state)
                     print "run_state"
                     print self.run_state
                     if self.run_state is 0:#go to point
                         self.cmd.linear.x = self.Motion_temp[0]
                         self.cmd.linear.y = 0
                         self.cmd.linear.z = 0
                         self.cmd.angular.z = self.Motion_temp[1]
                         if self.stop_flag:
                             #if self.MapPointNum_Now >= 1 or self.start == True:
                             #if self.MapPointNum_Now >=1 or (self.MapPointNum_Now == 0 and (self.last_workstate != 1 or self.last_workstate !=  2)) or self.start == True:
                             if (not ((self.MapPointNum_Now == 0 and (self.last_workstate == 2 or self.last_workstate == 1)) or self.MapPoint_Num_Update != 10000)):
                                 self.cmd.linear.x = 0
                                 self.cmd.angular.z = 0
                                 print "*************************************"
                         self.cmd_vel_pub.publish(self.cmd)
                         #print self.cmd.linear.x
                         #print self.cmd.linear.y
                         #print self.cmd.linear.z
                         #print self.cmd.angular.z
                         #print "pub_0"
                     if self.run_state is 1:#go to angle
                         self.cmd.linear.x = 0
                         self.cmd.linear.y = 0
                         self.cmd.linear.z = 0
                         self.cmd.angular.z = self.Motion_rotate_temp[2]
                         if self.stop_flag:
                             #if self.MapPointNum_Now >= 1 or (self.MapPointNum_Now == 0 and (self.last_workstate != 1 or self.last_workstate != 2)) or self.start == True:
                             if (not ((self.MapPointNum_Now == 0 and (self.last_workstate == 2 or self.last_workstate == 1)) or self.MapPoint_Num_Update != 10000 or (self.MapPointNum_Now == self.MapPointNum_Temp and self.back_home == 0))):
                                 self.cmd.linear.x = 0
                                 self.cmd.angular.z = 0
                             #self.cmd.angular.z = 0
                         self.cmd_vel_pub.publish(self.cmd)
                         print "pub_1"
                 if self.EnableFlag == False:
                     self.cmd.linear.x = 0
                     self.cmd.linear.y = 0
                     self.cmd.linear.z = 0
                     self.cmd.angular.z = 0
                     self.cmd_vel_pub.publish(self.cmd)

                 print "position now: "
                 print self.MapPointNum_Now
                 if self.MapPointNum_Now == self.MapPointNum_All - 1:
                     if self.last_point_update == True and self.add_last_point == False and self.add_coarse_point == False and self.update_ban == False:
                         self.MapPoint.update_point((self.MapPointNum_All - 1), self.last_point_temp_x,
                                                    self.last_point_temp_y, self.last_point_temp_yaw)
                         print "update the last point-------"
                         #temp_point = self.MapPoint.get_point(self.MapPointNum_All - 1)
                         #print temp_point
                         self.last_point_update = False
                         self.add_last_point = True

                 if self.run_state is 0 and self.run_laser == False:
                 #if self.run_laser == False:
                 #先到点
                     if self.MotionPlaner.reach_position():
                         self.run_state = 1
                         print "state_0______________________"
                     # else:
                     #     self.run_state = 0

                 if self.run_state is 1 and self.run_laser == False:
                     #再判断是否到点，此时的self.run_laser为false
                     if self.Motion_Planer_rotate.reach_angle():
                         self.run_state = 0
                         print "state_1_______________________"
                     # else:
                     #     self.run_state = 1
                         #到点后更新自身点位序号
                         self.MapPointNum_Now = self.MapPointNum_Now + 1
                         self.last_workstate = 0
                         print "MapPointNum_Now" + str(self.MapPointNum_Now)
                         if self.done == 0 and len(self.MapPoint.pointlist_x)!=0:
                             #发布航迹点的序号
                             self.MapPoint_id.values[1] = self.MapPoint.get_id(self.MapPointNum_Now - 1)
                             #print self.MapPoint_id.values[1]
                             self.working_state_pub.publish(self.MapPoint_id)
                         #判断此时是否到达最终点
                         for ii in range (len(self.MapPointNumLaserIndex)):
                             if self.MapPointNumLaserIndex[ii] == self.MapPointNum_Now:
                                 self.done = 1
                                 self.messageFilter_workingState = 0
                                 self.GPS_Position_State.values[0] = 0
                                 self.working_state_pub.publish(self.GPS_Position_State)
                                 if self.control_mode == 1:
                                     self.add_true_target = True
                                 else:
                                     self.add_true_target = False
                                 #self.Map_select = 1

                                 self.num = self.num+1
                                 #print self.num
                                 #print self.add_true_target
                             # if self.num == 0:
                             #     self.working_state.values[0] = 0
                             #     self.working_state_pub.publish(self.working_state)

                         #在原来粗对准之后才会满足这个条件
                         if self.MapPointNum_Now == self.MapPoint_Num_Update :
                             #self.run_laser = True #-- self.MapPoint_Num_Update is the amount of MapPoint in the New MapPoint list,when the car reach the destination ,run_laser = true
                             #self.MotionPlaner.setValues(0.2,0.2,0.05,0.042)
                             #print self.run_laser #non-auto model
                             self.messageFilter_workingState = 0
                             self.Coarse_Alignment_state.values[0] = 0
                             self.working_state_pub.publish(self.Coarse_Alignment_state)
                             #print "reach the target MapPoint"
                             if self.control_mode == 1:
                                 self.run_laser = True
                             else:
                                 self.run_laser = False
                             #print self.run_laser
         else:
             self.cmd.linear.x = 0
             self.cmd.linear.y = 0
             self.cmd.linear.z = 0
             self.cmd.angular.z = 0
             self.cmd_vel_pub.publish(self.cmd)
             self.Pause_State.values[0] = 1
             self.working_state_pub.publish(self.Pause_State)




     def callback3(self, data):
         #激光扫描没有发现目标时的策略
         #self.laser_alignment_workstate.values[0] = 1
         if data.name == "find_none" and self.run_laser == True:
             self.disappear_count = self.disappear_count+1
             if(self.disappear_count > 5):
                 self.target_disappear = True
                 self.disappear_count = 0
                 self.cmd.linear.x = 0
                 self.cmd.linear.y = 0
                 self.cmd.linear.z = 0
                 self.cmd.angular.z = 0
                 self.cmd_vel_pub.publish(self.cmd)
                 self.Laser_Alignment_State.values[0] = 2
                 self.working_state_pub.publish(self.Laser_Alignment_State)
             else:
                 self.target_disappear = False

         #if self.control_mode == "Auto":
         #进行粗对准之前，接收激光节点发布的数据，加入粗对准所需要的gps航迹点
         if self.add_true_target == True and data.name == "target_position_rtk" and self.refresh == 1:
            #print "first alignment"
            self.add_true_target = False
            self.refresh = 0
            #print "add"
            self.search_target = False
            #self.yaw_new = math.atan((data.values[1]-self.Position_now_rtkGPS[1])/(data.values[0]-self.Position_now_rtkGPS[0])) #-- calculate the yaw between the current MapPoint with the next MapPoint
            #publish
            self.GPS_Position_State.values[0] = 0
            self.working_state_pub.publish(self.GPS_Position_State)

            self.Coarse_Alignment_state.values[0] = 1
            self.working_state_pub.publish(self.Coarse_Alignment_state)

            self.Laser_Alignment_State.values[0] = 0
            self.working_state_pub.publish(self.Laser_Alignment_State)


            for ii in range (len(self.MapPoint.pointlist_x)):
                self.MapPoint_Temp = self.MapPoint.get_point(ii) #-- self.MapPoint_New.add_point(self.MapPoint.get_point(ii)) will generate error because of the incorrect arg number
                self.MapPoint_New.add_point(self.MapPoint_Temp[0],self.MapPoint_Temp[1],self.MapPoint_Temp[2],100)
            #self.MapPoint_New = self.MapPoint
                #print "Test"
                #print self.MapPoint_Temp
            #向map列表中添加新的航迹点坐标
            #self.MapPoint_New.add_point(self.Position_now_rtkGPS[0],self.Position_now_rtkGPS[1],self.yaw_new,100) #-- add the MapPoint which point to the target MapPoint
            self.MapPoint_New.add_point(data.values[0],data.values[1],data.values[2],101) #-- add the target point
            self.add_coarse_point = True
            self.target_distance = data.values[3] #-- save the distance from the car to the middle point
            #选择新的map列表
            self.Map_select = 1
            #print "target_distance: "
            #print self.target_distance
            self.count = self.count+1
            #self.MapPoint_Num_Update = self.MapPointNum_Now + 2

            #刷新map列表长度
            self.MapPointNum_All = len(self.MapPoint_New.pointlist_x)

            #赋值为新的map列表长度
            self.MapPoint_Num_Update = self.MapPointNum_All #-- save the amounts of MapPoint in the New MapPoint list
            self.last_workstate = 1
            for ii in range (self.MapPointNum_All):
                print self.MapPoint_New.get_point(ii)
#dedect cc
         if data.name == "first_search":
             if data.values[0] == 0.0:
                 self.search_flag = False

             else:
                 self.search_flag = True
         #避障
         if data.name == "stop_flag":
             if data.values[0] == 0.0:
                  self.stop_count = self.stop_count+1
                  self.Barrier_State.values[0] = 0
                  self.working_state_pub.publish(self.Barrier_State)
                  if self.stop_count>50:
                      self.stop_flag = False
                      self.stop_count = 51

                      #print "Not Stop"
             else:
                  self.stop_flag = True
                  self.Barrier_State.values[0] = 1
                  self.working_state_pub.publish(self.Barrier_State)
                  self.stop_count = 0
             return
         if self.Pause == 0:
             self.Pause_State.values[0] = 0
             self.working_state_pub.publish(self.Pause_State)
             #接收反光柱连线中点的坐标，用于后面的细对准
             if self.run_laser and data.name == "laser_target_position_now" and self.imu_nav == False:
                 self.GPS_Position_State.values[0] = 0
                 self.working_state_pub.publish(self.GPS_Position_State)

                 self.Coarse_Alignment_state.values[0] = 0
                 self.working_state_pub.publish(self.Coarse_Alignment_state)

                 self.Laser_Alignment_State.values[0] = 1
                 self.working_state_pub.publish(self.Laser_Alignment_State)

                 self.debug_print_count = self.debug_print_count + 1
                 if self.debug_print_count >= 10:
                     print data.values[0], data.values[1], data.values[2]/math.pi*180,data.values[3]
                     self.debug_print_count=0
                 self.MotionPlaner_laser.setNowPosition([-0.60, 0, 0])#maybe exist some mistake
                 self.MotionPlaner_laser.setNextTarget((data.values[0], data.values[1], data.values[2]))
                 self.Motion_temp = self.MotionPlaner_laser.MotionPlan()# get speed wanted
                 self.last_workstate = 2
                 if self.run_state is 0:  # go to point
                     self.cmd.linear.x = self.Motion_temp[0]
                     self.cmd.linear.y = 0
                     self.cmd.linear.z = 0
                     self.cmd.angular.z = self.Motion_temp[1]
                     self.cmd_vel_pub.publish(self.cmd)
                 if self.run_state is 1:  # go to angle
                     self.cmd.linear.x = 0
                     self.cmd.linear.y = 0
                     self.cmd.linear.z = 0
                     self.cmd.angular.z = self.Motion_temp[2]
                     self.cmd_vel_pub.publish(self.cmd)

                 if self.run_state is 0:
                     if self.MotionPlaner_laser.reach_position():
                         self.run_state = 1
                         print "reach position"

                 if self.run_state is 1:
                     if self.MotionPlaner_laser.reach_angle():
                         self.run_state = 0
                         print "reach angle"
                         if self.MotionPlaner_laser.reach_position():#may be loop forever
                             self.laser_count = self.laser_count+1
                             self.messageFilter_workingState = 0

                             #发布激光对准状态，并且车体静止
                             self.Laser_Alignment_State.values[0] = 0
                             self.working_state_pub.publish(self.Laser_Alignment_State)
                             self.run_laser = False #-- set the flag to false ,laser end
                             self.Camera_start.values[0] = 1
                             self.working_state_pub.publish(self.Camera_start)
			     time.sleep(0.1)
                             self.working_state_pub.publish(self.Camera_start)
			     time.sleep(0.1)
                             self.working_state_pub.publish(self.Camera_start)
                             self.cmd.linear.x = 0
                             self.cmd.linear.y = 0
                             self.cmd.linear.z = 0
                             self.cmd.angular.z = 0
                             self.cmd_vel_pub.publish(self.cmd)
                             print "Laser End"
         else:
             self.cmd.linear.x = 0
             self.cmd.linear.y = 0
             self.cmd.linear.z = 0
             self.cmd.angular.z = 0
             self.cmd_vel_pub.publish(self.cmd)

             self.Pause_State.values[0] = 1
             self.working_state_pub.publish(self.Pause_State)


     #接收航迹点,并且做一些初始化
     def callback4(self, data):
         self.start = False
         self.last_point_search = 0
         self.last_point_update = False
         self.last_point_updated = False
         self.add_last_point = False
         self.add_coarse_point = False
         self.laser_count = 0

         self.MapPointNum_Now = 0
         self.MapPoint_Num_Update = 10000
         self.Map_select = 0
         self.done = 0
         self.add_true_target = False
         self.run_laser = False
         self.MapPoint.pointlist_x = []
         self.MapPoint.pointlist_y = []
         self.MapPoint.pointlist_yaw = []
         self.MapPoint.id = []

         self.MapPoint_New.pointlist_x = []
         self.MapPoint_New.pointlist_y = []
         self.MapPoint_New.pointlist_yaw = []
         self.MapPoint_New.id = []
         self.MapPointNumLaserIndex = []
         ii = 0
         for ii in range(len(data.poses)):
             quaternion = (
                 data.poses[ii].pose.orientation.x,
                 data.poses[ii].pose.orientation.y,
                 data.poses[ii].pose.orientation.z,
                 data.poses[ii].pose.orientation.w)
             (roll,pitch,yaw) =  transformations.euler_from_quaternion(quaternion)
             if yaw>2*math.pi:
                 yaw=yaw-2*math.pi
             elif yaw<0:
                 yaw=yaw+2*math.pi
             self.MapPoint.add_point(data.poses[ii].pose.position.x,data.poses[ii].pose.position.y,yaw,data.poses[ii].pose.position.z)
             if ii == (len(data.poses) - 1):
                 self.MapPoint_id.values[0] = data.poses[ii].pose.position.z
                 self.working_state_pub.publish(self.MapPoint_id)
             print data.poses[ii].pose.position.x,data.poses[ii].pose.position.y,yaw,data.poses[ii].pose.position.z
             print '\n'
         print "receive_________"
         print self.MapPointNum_Now

         self.MapPointNum_All = len(self.MapPoint.pointlist_x)
         self.MapPointNum_Temp = self.MapPointNum_All - 1
         #self.messageFilter_workingState = 1
         self.GPS_Position_State.values[0] = 1
         self.working_state_pub.publish(self.GPS_Position_State)
         print self.MapPointNum_All
         if data.poses[ii].pose.position.z!=0:
             self.MapPointNumLaserIndex.append(self.MapPointNum_All)

     #imu的回调函数
     def callback5(self,data):
         if data.name == "roll_pitch_yaw" and self.run_laser == True and self.target_disappear == True and self.Pause % 2 == 0:
             if(self.imu_count == 0):
                 self.initial_yaw = data.values[2]
                 self.imu_count = 1
             if abs(data.values[1]) > 3:
                 self.imu_nav = True
                 self.Imu_Nav_State.values[0] = 1
                 self.working_state_pub.publish(self.Imu_Nav_State)
                 if(abs(data.values[2] - self.initial_yaw)>self.err_angle):
                     if((data.values[2] - self.initial_yaw)>0):
                         self.cmd.linear.x = 0.1
                         self.cmd.linear.y = 0
                         self.cmd.linear.z = 0
                         self.cmd.angular.z = -0.1
                         self.cmd_vel_pub.publish(self.cmd)
                     else:
                         self.cmd.linear.x = 0.1
                         self.cmd.linear.y = 0
                         self.cmd.linear.z = 0
                         self.cmd.angular.z = 0.1
                         self.cmd_vel_pub.publish(self.cmd)
                 else:
                     self.cmd.linear.x = 0.1
                     self.cmd.linear.y = 0
                     self.cmd.linear.z = 0
                     self.cmd.angular.z = 0
                     self.cmd_vel_pub.publish(self.cmd)

             else:
                 self.imu_nav = False
                 self.Imu_Nav_State.values[0] = 0
                 self.working_state_pub.publish(self.Imu_Nav_State)


     #数据通讯的回调函数
     def callback6(self,data):
         #first data filter
         self.messageFilter_tank_id = 0
         self.messageFilter_track_point_id = 0
         self.messageFilter_first_alignment = 0
         self.messageFilter_laser_alignment = 0
         self.messageFilter_stop = 0
         self.messageFilter_back_home = 0
         self.messageFilter_is_start_camera = 0
         self.messageFilter_alignment_distance = 0

         # if data.tank_id == 100000 or data.back_home == 1:
         #     self.update_ban = True
         #
         # else:
         #     self.update_ban = False


         if self.messageFilter_tank_id_before != data.tank_id and data.tank_id != 0:
             self.messageFilter_tank_id = 1
         if self.messageFilter_track_point_id_before != data.track_point_id and data.track_point_id != 0:
             self.messageFilter_track_point_id = 1
         if self.messageFilter_first_alignment_before != data.first_alignment and data.first_alignment==1:
             self.messageFilter_first_alignment = 1
         if self.messageFilter_laser_alignment_before != data.laser_alignment and (data.laser_alignment==1 or data.laser_alignment==3):
             self.messageFilter_laser_alignment = 1
         if self.messageFilter_stop_before != data.stop and data.stop==1:
             self.messageFilter_stop = 1
         if self.messageFilter_back_home_before != data.back_home and data.back_home == 1:
             self.messageFilter_back_home = 1
         if self.messageFilter_is_start_camera_before != data.is_start_camera:
             self.messageFilter_is_start_camera = 1
         if self.messageFilter_alignment_distance_before != data.distance_alignment:
             self.messageFilter_alignment_distance = 1

         if data.tank_id == 0 and data.track_point_id != 0:
             self.update_ban = True

         if data.tank_id != 0:
             self.update_ban = False

         #if self.messageFilter_back_home == 1:
             #self.update_ban = True

         self.messageFilter_tank_id_before = data.tank_id
         self.messageFilter_track_point_id_before = data.track_point_id
         self.messageFilter_first_alignment_before = data.first_alignment
         self.messageFilter_laser_alignment_before = data.laser_alignment
         self.messageFilter_stop_before = data.stop
         self.messageFilter_back_home_before = data.back_home
         self.messageFilter_is_start_camera_before = data.is_start_camera
         self.messageFilter_alignment_distance_before = data.distance_alignment
         self.control_mode = data.mode
         #second state judge
         if data.stop == 1:
             self.messageFilter_workingState = 0

             self.MapPointNum_Now = 0
             self.MapPointNum_All = 0
             self.MapPoint_Num_Update = 10000
             self.add_true_target = False
             self.run_laser = False
             self.Map_select = 0
             #self.run_state = 0
             self.wait = 0
             self.done = 0
             self.Calculate_distance = 0
             self.Pause = 0
             self.add_new = 0
             self.detect = 0
             self.back_home = 0
             #另外添加的变量初始化
             self.last_point_updated = False
             self.last_point_update = False
             self.last_point_search = 0
             self.start = False
             self.add_last_point = False
             self.add_coarse_point = False
             self.laser_count = 0




             self.MapPoint.pointlist_x = []
             self.MapPoint.pointlist_y = []
             self.MapPoint.pointlist_yaw = []
             self.MapPoint.id = []

             self.MapPoint_New.pointlist_x = []
             self.MapPoint_New.pointlist_y = []
             self.MapPoint_New.pointlist_yaw = []
             self.MapPoint_New.id = []
             # stop
             self.cmd.linear.x = 0
             self.cmd.linear.y = 0
             self.cmd.linear.z = 0
             self.cmd.angular.z = 0
             self.cmd_vel_pub.publish(self.cmd)

             self.GPS_Position_State.values[0] = 0
             self.working_state_pub.publish(self.GPS_Position_State)

             self.Coarse_Alignment_state.values[0] = 0
             self.working_state_pub.publish(self.Coarse_Alignment_state)

             self.Laser_Alignment_State.values[0] = 0
             self.working_state_pub.publish(self.Laser_Alignment_State)
             return



         if data.pause == 1:
             self.Pause = 1
         else:
             self.Pause = 0

         #任意开启相机
	 #print "camera***********888"
	 #print self.messageFilter_is_start_camera
         if self.messageFilter_is_start_camera != 0:#use camera
             #self.messageFilter_workingState = 2
             #do some camera init
             if data.is_start_camera == 1:
                 self.Camera_start.values[0] = 1
                 self.working_state_pub.publish(self.Camera_start)
                 time.sleep(0.1)
                 self.working_state_pub.publish(self.Camera_start)
                 time.sleep(0.1)
                 self.working_state_pub.publish(self.Camera_start)
                 time.sleep(0.1)
                 self.working_state_pub.publish(self.Camera_start)
                 print "---------------------------------------on"
             elif data.is_start_camera == 0:
                 self.Camera_start.values[0] = 0
                 self.working_state_pub.publish(self.Camera_start)
                 time.sleep(0.2)
                 self.working_state_pub.publish(self.Camera_start)
                 time.sleep(0.2)
                 self.working_state_pub.publish(self.Camera_start)
                 time.sleep(0.2)
                 self.working_state_pub.publish(self.Camera_start)
                 time.sleep(0.2)
                 self.working_state_pub.publish(self.Camera_start)
                 time.sleep(0.2)
                 self.working_state_pub.publish(self.Camera_start)
                 time.sleep(0.2)
                 self.working_state_pub.publish(self.Camera_start)
                 time.sleep(0.2)
                 self.working_state_pub.publish(self.Camera_start)
                 time.sleep(0.2)
                 self.working_state_pub.publish(self.Camera_start)
                 time.sleep(0.2)
                 self.working_state_pub.publish(self.Camera_start)
                 time.sleep(0.2)
                 self.working_state_pub.publish(self.Camera_start)
                 print "---------------------------------------off off"



         if self.messageFilter_workingState == 0 or self.messageFilter_workingState == 2:#now is free or using camera
             if self.messageFilter_back_home != 0:
                 self.back_home = 1
                 self.update_ban = True
                 self.add_new = 0
                 self.detect = 0
                 self.wait = 0
                 self.change_radius_pub.publish(0.8)
                 time.sleep(0.1)
                 self.change_radius_pub.publish(0.8)
                 time.sleep(0.1)
                 self.change_radius_pub.publish(0.8)
                 time.sleep(0.1)
                 self.change_radius_pub.publish(0.8)
                 time.sleep(0.1)		 
                 self.messageFilter_workingState = 1
                 # stop camera
                 #
                 self.Camera_start.values[0] = 0
                 time.sleep(0.1)
                 self.working_state_pub.publish(self.Camera_start)
                 time.sleep(0.1)
                 self.working_state_pub.publish(self.Camera_start)
                 time.sleep(0.1)
                 self.working_state_pub.publish(self.Camera_start)
                 time.sleep(0.1)
                 self.working_state_pub.publish(self.Camera_start)

                 # do some laser init
                 self.add_true_target = False
                 self.run_laser = False
                 self.Map_select = 0

                 return
             else:
                 self.back_home = 0
                 self.change_radius_pub.publish(0.6)
                 #time.sleep(0.1)
                 self.change_radius_pub.publish(0.6)
                 #time.sleep(0.1)
                 self.change_radius_pub.publish(0.6)
                 #time.sleep(0.1)
                 #return
                 #print "------------------"
             if data.mode == 1:
                 # print "*******************************"
                 # print self.messageFilter_tank_id
                 # print self.messageFilter_track_point_id
                 # print self.messageFilter_track_point_id
                 if (self.messageFilter_tank_id != 0 and self.messageFilter_track_point_id != 0) or self.messageFilter_track_point_id != 0:
                     #print "*******************"
		     self.messageFilter_workingState = 1
                     if data.distance_alignment >= 0.7 and data.Pillar_distance >=0.5 :#check
                         #self.messageFilter_workingState = 1#now start working
                         self.distance_alignment_pub.publish(data.distance_alignment)
                         time.sleep(0.1)
                         #print "publish the laser alignment distance: "
                         #print data.distance_alignment
                         self.distance_alignment_pub.publish(data.distance_alignment)
                         time.sleep(0.1)
                         #print "publish the laser alignment distance: "
                         #print data.distance_alignment
                         self.distance_alignment_pub.publish(data.distance_alignment)
                         time.sleep(0.1)
                         #print "publish the laser alignment distance: "
                         #print data.distance_alignment
                         self.distance_alignment_pub.publish(data.distance_alignment)
                         time.sleep(0.1)
                         #stop camera
                         self.Camera_start.values[0] = 0
                         time.sleep(0.1)
                         self.working_state_pub.publish(self.Camera_start)
                         time.sleep(0.1)
                         self.working_state_pub.publish(self.Camera_start)
                         time.sleep(0.1)
                         self.working_state_pub.publish(self.Camera_start)
                         time.sleep(0.1)
                         self.working_state_pub.publish(self.Camera_start)

                         #do some laser init
                         self.add_true_target = False

                         self.Map_select = 0

                         return
             if data.mode == 0:
		 if self.messageFilter_track_point_id != 0:
		     self.messageFilter_workingState = 1
		     

                 if self.messageFilter_first_alignment != 0:
                     self.messageFilter_workingState = 1  # now start working
                     # stop camera
                     self.Camera_start.values[0] = 0
                     time.sleep(0.1)
                     self.working_state_pub.publish(self.Camera_start)
                     time.sleep(0.1)
                     self.working_state_pub.publish(self.Camera_start)
                     time.sleep(0.1)
                     self.working_state_pub.publish(self.Camera_start)
                     time.sleep(0.1)
                     self.working_state_pub.publish(self.Camera_start)
                     time.sleep(0.1)
                     self.working_state_pub.publish(self.Camera_start)
                     time.sleep(0.1)
                     self.working_state_pub.publish(self.Camera_start)
                     # do some laser init
                     self.add_true_target = True
                     self.run_laser = False



                     return
                 if self.messageFilter_laser_alignment != 0:
                     self.messageFilter_workingState = 1  # now start working
                     # stop camera
                     self.Camera_start.values[0] = 0
                     #time.sleep(0.1)
                     self.working_state_pub.publish(self.Camera_start)
                     time.sleep(0.1)
                     self.working_state_pub.publish(self.Camera_start)
                     time.sleep(0.1)
                     self.working_state_pub.publish(self.Camera_start)
                     time.sleep(0.1)
                     self.working_state_pub.publish(self.Camera_start)
                     time.sleep(0.1)
                     self.working_state_pub.publish(self.Camera_start)
                     time.sleep(0.1)
                     self.working_state_pub.publish(self.Camera_start)
                     # do some laser init
                     if data.laser_alignment == 1:
                         if data.distance_alignment >= 0.7 and data.Pillar_distance >= 0.5:
                            self.distance_alignment_pub.publish(data.distance_alignment)
                            time.sleep(0.1)
                            self.distance_alignment_pub.publish(data.distance_alignment)
                            time.sleep(0.1)
                            self.distance_alignment_pub.publish(data.distance_alignment)
                            #time.sleep(0.1)
                            #print "publish the alignment distance 2: "
                            #print data.distance_alignment
                            self.add_true_target = False
                            self.run_laser = True
                     if data.laser_alignment == 3:
                         if data.Pillar_distance >= 0.5:
                            self.distance_alignment_pub.publish(data.target_distance)
                            time.sleep(0.1)
                            self.distance_alignment_pub.publish(data.target_distance)
                            time.sleep(0.1)
                            self.distance_alignment_pub.publish(data.target_distance)

                            self.add_true_target = False
                            self.run_laser = True
                     print self.run_laser
                     #if data.distance_alignment >= 0.7 and data.Pillar_distance >= 0.5:




                     return

             # if self.messageFilter_is_start_camera != 0:#use camera
             #     self.messageFilter_workingState = 2
             #     #do some camera init
             #     if data.is_start_camera == 1:
             #         self.Camera_start.values[0] = 1
             #         self.working_state_pub.publish(self.Camera_start)
             #         time.sleep(0.1)
             #         self.working_state_pub.publish(self.Camera_start)
             #         time.sleep(0.1)
             #         self.working_state_pub.publish(self.Camera_start)
             #         time.sleep(0.1)
             #         self.working_state_pub.publish(self.Camera_start)
             #         #print "---------------------------------------"
             #     elif data.is_start_camera == 0:
             #         self.Camera_start.values[0] = 0
             #         self.working_state_pub.publish(self.Camera_start)
             #         time.sleep(0.2)
             #         self.working_state_pub.publish(self.Camera_start)
             #         time.sleep(0.2)
             #         self.working_state_pub.publish(self.Camera_start)
             #         time.sleep(0.2)
             #         self.working_state_pub.publish(self.Camera_start)
             #         time.sleep(0.2)
             #         self.working_state_pub.publish(self.Camera_start)
             #         time.sleep(0.2)
             #         self.working_state_pub.publish(self.Camera_start)
             #         time.sleep(0.2)
             #         self.working_state_pub.publish(self.Camera_start)
             #         time.sleep(0.2)
             #         self.working_state_pub.publish(self.Camera_start)
             #         time.sleep(0.2)
             #         self.working_state_pub.publish(self.Camera_start)
             #         time.sleep(0.2)
             #         self.working_state_pub.publish(self.Camera_start)
             #         time.sleep(0.2)
             #         self.working_state_pub.publish(self.Camera_start)
             #         #print "---------------------------------------"
             #     return

#在到达最后一个航迹点之前，检测对应的反光柱，接收反光柱连线中心的GPS坐标，替换掉原来最后一个航迹点
     def callback7(self,data):
         if self.update_ban == False:
             if self.MapPointNum_Now == self.MapPointNum_Temp and self.MapPointNum_Temp!= 0:
                 if self.last_point_updated:
                     print "already update"
                     return
                 #print "self.MapPointNum_Temp:   " + str(self.MapPointNum_Temp)
                 #print "self.MapPointNum_All:   " + str(self.MapPointNum_All)
                 #Temp_Point1 = self.MapPoint.get_point(self.MapPointNum_Temp - 1)
                 #print "Temp_Point1:   " + str(Temp_Point1)
                 #Temp_Point2 = self.MapPoint.get_point(self.MapPointNum_All - 1)
                 #print "Temp_Point2:   " + str(Temp_Point2)
                 Temp_yaw = self.Position_now_rtkGPS[2]
                 #print "Temp_yaw:   " + str(Temp_yaw)
                 if Temp_yaw > math.pi:
                     Temp_yaw = Temp_yaw - 2 * math.pi
                 elif Temp_yaw < -math.pi:
                     Temp_yaw = Temp_yaw + 2 * math.pi
                 else:
                     Temp_yaw = Temp_yaw
                 #---zmf
                 # angle = caculateYaw((Temp_Point2[1]-Temp_Point1[1]),(Temp_Point2[0]-Temp_Point1[0]))
                 # print "angle:   " + str(angle)
                 # temp_angle = Temp_yaw - angle
                 # print "temp_angle before:   " + str(temp_angle)
                 # #temp_angle = abs(Temp_yaw - math.atan2((Temp_Point[1] - self.Position_now_rtkGPS[1]),(Temp_Point[0] - self.Position_now_rtkGPS[0])))
                 # if temp_angle > math.pi:
                 #     temp_angle = 2*math.pi - temp_angle
                 # elif temp_angle < -math.pi:
                 #     temp_angle = temp_angle + 2*math.pi
                 # else:
                 #     temp_angle = temp_angle
                 # print "temp_angle after:   " + str(temp_angle)
                 #
                 # print "abs(temp_angle):   " + str(abs(temp_angle))
                 #if abs(temp_angle) < 0.18:
                 #---zmf
                 self.last_point_search = self.last_point_search + 1
                 if self.last_point_search == 2 and self.last_point_updated == False:
                     self.Pause = 1
                     self.start = True
                     self.cmd.linear.x = 0
                     self.cmd.angular.z = 0
                     self.cmd_vel_pub.publish(self.cmd)
                     print "wait for recheck---------"
                     self.last_updated = rospy.get_time()

                 if self.last_point_search == 4 and self.last_point_updated == False:
                     self.last_point_update = True
                     self.last_point_search = 0
                     self.last_point_temp_x = data.values[0]
                     self.last_point_temp_y = data.values[1]
                     self.last_point_temp_yaw = data.values[2]
                     self.last_point_updated = True
                     self.Pause = 0
                     self.start = False
                     print "last point:  " + str(self.last_point_temp_x) + " " + str(self.last_point_temp_y) + " " + str(self.last_point_temp_yaw)
                     print "get the last point ---------"
                     print "self.last_point_update: "
                     print self.last_point_update
         else:
             self.last_point_search = 0











if __name__ == "__main__":
    try:
        body = functionAll()
    except:
        rospy.logwarn("functionAll closed!")

