#!/usr/bin/env python
# license removed for brevity
import rospy
import Motion_Planning
import Map_Point
import LaserProcess_new_filter_new
import TF
import tf
import math
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from tcp2ros.msg import rtkGPSmessage
from sensor_msgs.msg import ChannelFloat32
from nav_msgs.msg import Path
from geometry_msgs.msg import Quaternion
from tf import transformations
from tcp2ros.msg import reach

class functionAll:
     def __init__(self):
         rospy.init_node('FunctionAll', anonymous=False)
         self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
         self.reach_pub = rospy.Publisher('reach', reach, queue_size=5)
         self.back_pub = rospy.Publisher('back',ChannelFloat32,queue_size=1) #--publish back messages
         self.rate = rospy.Rate(30)
         self.cmd = Twist()
         self.count = 0
         self.num = 0
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
         self.target_distance = 0 #--save the distance from car to the middle point
         self.back_count = False #-- publish back message when it's true

         self.initial_yaw = 0
         self.err_angle = 1
         self.imu_nav = False
         self.Position_Adjustment = False
         self.imu_count = 0
         self.target_disappear = False
         self.disappear_count = 0

         self.debug_print_count=0
         self.MapPoint_Num_Update = 10000 #-- set a big initial value
         self.yaw_new = 0
         self.Map_select = 0 #-- mark as the option of MapPoint list
         self.MapPoint_Temp = [] #-- save the
         self.New_Point = [] #give values before use the empty array
         self.laser_count = 0 #-- laser counter,when the car reach the target,laser work
         self.angle_temp = 0
         # self.MapPointNumLaserIndex = [4,11,30]#id is laser search point
         self.MapPointNumLaserIndex = [2]#id is laser search point

         self.MotionPlaner_speedXY = float(rospy.get_param("MotionPlaner_speedXY",'0.3'))
         self.MotionPlaner_speedYaw = float(rospy.get_param("MotionPlaner_speedYaw",'0.2'))
         self.MotionPlaner_errorMeter = float(rospy.get_param("MotionPlaner_errorMeter",'0.3'))#m
         self.MotionPlaner_errorAngle = float(rospy.get_param("MotionPlaner_errorAngle",'0.2'))#rad

         self.MotionPlaner = Motion_Planning.Motion_Planning(self.MotionPlaner_speedXY, self.MotionPlaner_speedYaw, self.MotionPlaner_errorMeter, self.MotionPlaner_errorAngle)
         self.MapPoint = Map_Point.Map_Point()
         self.MapPoint_New = Map_Point.Map_Point()
         self.TF_body = TF.TF()
         #MapPoint is in rtkGPS coordinate system

         self.North_base = float(rospy.get_param("North_base",'0.2'))
         self.East_base = float(rospy.get_param("East_base",'0.2'))
         self.Yaw_base = float(rospy.get_param("Yaw_base",'0.0'))
         #print self.North_base ,self.East_base

         self.MapPoint.add_point(4630830.0000, 786262.1250, 0.3905)
         self.MapPoint.add_point(4630835.5000, 786264.6250, 0.0692)

         self.MapPoint.add_point(4630843.5000, 786262.1875, 1.5587)

         #self.MapPoint.add_point(3349295.7500, 511144.2500, 5.1409)


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
      
         self.MapPointNum_Now = 0
         self.MapPointNum_All = len(self.MapPoint.pointlist_x)
         #print self.MapPointNum_All 


         self.Position_now_rtkGPS = [self.North_base+0, self.East_base+0,self.Yaw_base]
         self.EnableFlag = False
         self.EnableCount = [0,0]
         self.Position_BaseLink_Target=[0,0,0]

         
         rospy.Subscriber("joy", Joy, self.callback1)
         rospy.Subscriber("rtkGPS", rtkGPSmessage, self.callback2, queue_size=1)
         rospy.Subscriber("laser_target_position", ChannelFloat32, self.callback3, queue_size=1)
         #rospy.Subscriber("path_node", Path, self.callback4, queue_size=1)
         rospy.Subscriber("/imu_angle", ChannelFloat32, self.callback5, queue_size=1)


         while not rospy.is_shutdown():
             self.rate.sleep()

     def callback1(self, data):
         print data

     def callback2(self, data):
         if self.MapPointNum_Now < self.MapPointNum_All:
             if data.flash_state == 'YAW':
                 self.Position_now_rtkGPS[2] = data.yaw_rad
                 self.EnableCount[1] = self.EnableCount[1]+1
             if data.flash_state == 'POSITION':
                 self.Position_now_rtkGPS[0] = data.north_meter
                 self.Position_now_rtkGPS[1] = data.east_meter
                 #print "Update Gps Position"
                 #print self.Position_now_rtkGPS
                 self.EnableCount[0] = self.EnableCount[0]+1
             if data.flash_state == 'Data_Valid_Fault':
                 self.EnableFlag = False
                 self.EnableCount[0] = 0
                 self.EnableCount[1] = 0
             if self.EnableCount[0] > 10 and self.EnableCount[1] > 10 :
                 self.EnableFlag = True
                 self.EnableCount[0] = 11
                 self.EnableCount[1] = 11

             if self.EnableFlag and self.run_laser == False:
                 self.Point_temp = self.MapPoint.get_point(self.MapPointNum_Now)
                 if (abs(self.Point_temp[0] - self.Position_now_rtkGPS[0]) > 0.4):
                    self.angle_temp = math.atan((self.Point_temp[1] - self.Position_now_rtkGPS[1]) / (self.Point_temp[0] - self.Position_now_rtkGPS[0]))


                 if self.Map_select == 0:
                     print "Map_select 0"
                     print self.MapPoint.get_point(self.MapPointNum_Now)
                     if (abs(self.Position_now_rtkGPS[2] - self.angle_temp) > ((30 * math.pi) / 180)):
                         self.Position_Adjustment = True
                         self.New_Point.append(self.Position_now_rtkGPS[0])
                         self.New_Point.append(self.Position_now_rtkGPS[1])
                         self.New_Point.append(self.angle_temp)
                         self.Position_BaseLink_Target=self.TF_body.rtkGPStoBaseLink(self.New_Point, self.Position_now_rtkGPS)
                     else:
                         self.Position_Adjustment = False
                         self.Position_BaseLink_Target = self.TF_body.rtkGPStoBaseLink(
                             self.MapPoint.get_point(self.MapPointNum_Now), self.Position_now_rtkGPS)
                     self.debug_print_count =  self.debug_print_count+1
                 if self.Map_select ==1 :
                     print "Map_select 1"
                     if (abs(self.Position_now_rtkGPS[2] - self.angle_temp) > ((30 * math.pi) / 180)):
                         self.Position_Adjustment = True
                         self.New_Point.append(self.Position_now_rtkGPS[0])
                         self.New_Point.append(self.Position_now_rtkGPS[1])
                         self.New_Point.append(self.angle_temp)
                         self.Position_BaseLink_Target=self.TF_body.rtkGPStoBaseLink(self.New_Point, self.Position_now_rtkGPS)
                     else:
                         self.Position_Adjustment = False
                         self.Position_BaseLink_Target = self.TF_body.rtkGPStoBaseLink(
                             self.MapPoint_New.get_point(self.MapPointNum_Now), self.Position_now_rtkGPS)
                     #self.Position_BaseLink_Target=self.TF_body.rtkGPStoBaseLink(self.MapPoint_New.get_point(self.MapPointNum_Now), self.Position_now_rtkGPS)
                     self.debug_print_count =  self.debug_print_count+1
                 if self.debug_print_count >=10:
                     #print self.Position_BaseLink_Target
                     self.debug_print_count=0
                 self.MotionPlaner.setNowPosition([0,0,0])
                 self.MotionPlaner.setNextTarget(self.Position_BaseLink_Target)
                 #print self.Position_BaseLink_Target
                 self.Motion_temp = self.MotionPlaner.MotionPlan()#get speed wanted
		 print self.Motion_temp
                 # self.Point_temp = self.MapPoint.get_point(self.MapPointNum_Now)
                 # self.angle_temp = math.atan((self.Point_temp[1] - self.Position_BaseLink_Target[1])/(self.Point_temp[0] - self.Position_BaseLink_Target[0]))
                 # #print "In gps"
                 # #if abs(self.angle_temp)>(45*math.pi/180):
                 # print self.Position_now_rtkGPS
                 # print self.Position_now_rtkGPS[2] - self.angle_temp
                 # print "run_state"
                 # print self.run_state
                 # if( self.run_state == 0 and abs(self.Position_now_rtkGPS[2]-self.angle_temp)>(10*math.pi/180)):
                 # #while(self.run_state == 0 and abs(self.Position_now_rtkGPS[2]-self.Point_temp[2])>(45*math.pi/180)):
                 #     #print "while"
                 #     print "TTT"
                 #
                 #     self.cmd.linear.x = 0
                 #     self.cmd.linear.y = 0
                 #     self.cmd.linear.z = 0
                 #     self.cmd.angular.z = self.Motion_temp[2]/3
                 #     print "angle_adjust"
                 #     print self.cmd.angular.z
                 #     if self.stop_flag:
                 #         self.cmd.angular.z = 0
                 #     self.cmd_vel_pub.publish(self.cmd)
                 # else:
                 # self.cmd.linear.x = 0
                 # self.cmd.linear.y = 0
                 # self.cmd.linear.z = 0
                 # self.cmd.angular.z = 0
                 # self.cmd_vel_pub.publish(self.cmd)
                 if self.run_state is 0:#go to point
                     self.cmd.linear.x = self.Motion_temp[0]
                     self.cmd.linear.y = 0
                     self.cmd.linear.z = 0
                     self.cmd.angular.z = self.Motion_temp[1]
                     if self.stop_flag:
                         self.cmd.linear.x = 0
                         self.cmd.angular.z = 0
                     self.cmd_vel_pub.publish(self.cmd)
                 if self.run_state is 1:#go to angle
                     self.cmd.linear.x = 0
                     self.cmd.linear.y = 0
                     self.cmd.linear.z = 0
                     self.cmd.angular.z = self.Motion_temp[2]
                     if self.stop_flag:
                         self.cmd.angular.z = 0
                     self.cmd_vel_pub.publish(self.cmd)
             if self.EnableFlag == False:
                 self.cmd.linear.x = 0
                 self.cmd.linear.y = 0
                 self.cmd.linear.z = 0
                 self.cmd.angular.z = 0
                 self.cmd_vel_pub.publish(self.cmd)

             if self.run_state is 0 and self.run_laser == False and self.Position_Adjustment == False:
                 if self.MotionPlaner.reach_position():
                     self.run_state = 1

             if self.run_state is 1 and self.run_laser == False and self.Position_Adjustment == False:
                 if self.MotionPlaner.reach_angle():
                     self.run_state = 0
                     print "MapPointNum_Now" + str(self.MapPointNum_Now)
                     print self.MapPointNum_Now
#                     if self.goto_real_target == True:
#                        self.goto_real_target = False
#                        self.run_laser = True
#                        #self.MotionPlaner.setValues(0.2,0.2,0.05,0.042)
#                        print "Laser start"

                     for ii in range (len(self.MapPointNumLaserIndex)):
                         if self.MapPointNumLaserIndex[ii] == self.MapPointNum_Now:
                             self.add_true_target = True #when the car reach the last map point which set before,set the self.add_true_target as True
                             self.Map_select = 1 #when the car reach the last map point which set before,select another Map
                             #self.num = self.num+1
                             #print self.num
                     if self.MapPointNum_Now == (self.MapPoint_Num_Update - 1):
                         self.run_laser = True #-- self.MapPoint_Num_Update is the amount of MapPoint in the New MapPoint list,when the car reach the destination ,run_laser = true
                         self.MotionPlaner.setValues(0.2,0.2,0.05,0.042)
                         print self.run_laser
                         print "Laser start"
                         #self.run_laser = True
                         #self.MotionPlaner.setValues(0.2,0.2,0.05,0.042)
                         #print "Laser start"
                     self.MapPointNum_Now = self.MapPointNum_Now+1
                     #if self.MapPointNum_Now == self.MapPointNum_All:
                          #self.MapPointNum_Now = self.MapPointNum_All-1

     def callback3(self, data):
         if data.name == "find_none" and self.run_laser == True:
             self.disappear_count = self.disappear_count+1
             if(self.disappear_count > 5):
                 self.target_disappear = True
                 self.disappear_count = 0
             else:
                 self.target_disappear = False
             self.cmd.linear.x = 0
             self.cmd.linear.y = 0
             self.cmd.linear.z = 0
             self.cmd.angular.z = 0
             self.cmd_vel_pub.publish(self.cmd)

         if self.add_true_target == True and data.name == "target_position_rtk":
            self.add_true_target = False            
            self.yaw_new = math.atan((data.values[1]-self.Position_now_rtkGPS[1])/(data.values[0]-self.Position_now_rtkGPS[0])) #-- calculate the yaw between the current MapPoint with the next MapPoint
            for ii in range (len(self.MapPoint.pointlist_x)):
                self.MapPoint_Temp = self.MapPoint.get_point(ii) #-- self.MapPoint_New.add_point(self.MapPoint.get_point(ii)) will generate error because of the incorrect arg number
                self.MapPoint_New.add_point(self.MapPoint_Temp[0],self.MapPoint_Temp[1],self.MapPoint_Temp[2])
            #self.MapPoint_New = self.MapPoint
                print "Test"
                print self.MapPoint_Temp
            self.MapPoint_New.add_point(self.Position_now_rtkGPS[0],self.Position_now_rtkGPS[1],self.yaw_new) #-- add the MapPoint which point to the target MapPoint
            self.MapPoint_New.add_point(data.values[0],data.values[1],data.values[2]) #-- add the target point
            self.target_distance = data.values[3] #-- save the distance from the car to the middle point
            print self.target_distance
            self.count = self.count+1
            #self.MapPoint_Num_Update = self.MapPointNum_Now + 2
            self.MapPointNum_All = len(self.MapPoint_New.pointlist_x)
            self.MapPoint_Num_Update = self.MapPointNum_All #-- save the amounts of MapPoint in the New MapPoint list

            print "New MapPoint:"
            print data.values[0],data.values[1],data.values[2]
            print self.count
            print self.MapPointNum_Now
            print self.add_true_target
            print self.MapPointNumLaserIndex
            for ii in range (len(self.MapPoint.pointlist_x)): #-- test the valuation
                print self.MapPoint.get_point(ii)
            for ii in range (self.MapPointNum_All):
                print self.MapPoint_New.get_point(ii)

            #self.goto_real_target = 1
         if data.name == "stop_flag":
             if data.values[0] == 0.0:
                  self.stop_count = self.stop_count+1
                  if self.stop_count>50:
                      self.stop_flag = False
                      self.stop_count = 51
                      #print "Not Stop"
             else:
                  self.stop_flag = True
                  self.stop_count = 0
                  #print "STOP"
             return
         if self.run_laser and data.name == "laser_target_position_now" and self.imu_nav == False:
             print self.run_laser
             self.debug_print_count = self.debug_print_count + 1
             if self.debug_print_count >= 10:
                 print data.values[0], data.values[1], data.values[2]/math.pi*180,data.values[3]
                 self.debug_print_count=0
             print "self.laser_count"
             print self.laser_count
             if self.laser_count == 1:
                 if data.values[3] != 1:
                     print "don't accept"
                     return
             self.MotionPlaner.setNowPosition([-0.60, 0, 0])#maybe exist some mistake
             self.MotionPlaner.setNextTarget((data.values[0], data.values[1], data.values[2]))
             self.Motion_temp = self.MotionPlaner.MotionPlan()# get speed wanted
             if self.run_state is 0:  # go to point
                 self.cmd.linear.x = self.Motion_temp[0]/2.0
                 self.cmd.linear.y = 0
                 self.cmd.linear.z = 0
                 self.cmd.angular.z = self.Motion_temp[1]/2.0
                 self.cmd_vel_pub.publish(self.cmd)
             if self.run_state is 1:  # go to angle
                 self.cmd.linear.x = 0
                 self.cmd.linear.y = 0
                 self.cmd.linear.z = 0
                 self.cmd.angular.z = self.Motion_temp[2]/2.0
                 self.cmd_vel_pub.publish(self.cmd)

             if self.run_state is 0:
                 if self.MotionPlaner.reach_position():
                     self.run_state = 1
                     print "reach position"

             if self.run_state is 1:
                 if self.MotionPlaner.reach_angle():
                     self.run_state = 0
                     print "reach angle"
                     if self.MotionPlaner.reach_position():#may be loop forever
#                         reach_msg=reach()
#                         reach_msg.time=rospy.Time.now()
#                         reach_msg.reach=1
#                         reach_pub.publish(reach_msg)
                         self.laser_count = self.laser_count+1
                         if self.laser_count == 1:
                             self.back_count = True
                         if self.back_count: #-- reach the point (distance = 1m),publish the back message, then the car will back to the target point
                             print "pub back"
                             self.back_count = False
                             back_msg = ChannelFloat32()
                             back_msg.name = "goback"
                             back_msg.values.append(self.target_distance)
                             #back_msg.values.append(2.0)
                             self.back_pub.publish(back_msg)

                         #self.MotionPlaner.setValues(self.MotionPlaner_speedXY, self.MotionPlaner_speedYaw, self.MotionPlaner_errorMeter, self.MotionPlaner_errorAngle)
                         if self.laser_count == 2: #-- now the car reach the target point only by laser
                             self.run_laser = False #-- set the flag to false ,laser end
                         self.cmd.linear.x = 0
                         self.cmd.linear.y = 0
                         self.cmd.linear.z = 0
                         self.cmd.angular.z = 0
                         self.cmd_vel_pub.publish(self.cmd)
#                         self.MotionPlaner.add_point(self.Position_now_rtkGPS[0], self.Position_now_rtkGPS[1], self.Position_now_rtkGPS[2])
#                         self.MapPointNum_All = len(self.MapPoint.pointlist_x)
#                         self.MapPointNum_Now = self.MapPointNum_Now+1
#                         if self.MapPointNum_Now == self.MapPointNum_All:
#                             self.MapPointNum_Now = self.MapPointNum_All-1
                         print "Laser End"

     def callback4(self, data):
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
             self.MapPoint.add_point(data.poses[ii].pose.position.x,data.poses[ii].pose.position.y,yaw)
             print data.poses[ii].pose.position.x,data.poses[ii].pose.position.y,yaw
             print '\n'

         self.MapPointNum_All = len(self.MapPoint.pointlist_x)
         if data.poses[ii].pose.position.z!=0:
             self.MapPointNumLaserIndex.append(self.MapPointNum_All-1)

     # def callback5(self,data):
     #     if data.name == "roll_pitch_yaw":
     #         #print data.values[1]
     #         if abs(data.values[1]) > 3:
     #             self.imu_nav = True
     #             self.cmd.linear.x = 0.1
     #             self.cmd.linear.y = 0
     #             self.cmd.linear.z = 0
     #             self.cmd.angular.z = 0
     #             self.cmd_vel_pub.publish(self.cmd)
     #             #print "navigation by imu"
     #         else:
     #             self.imu_nav = False

     def callback5(self,data):
         if data.name == "roll_pitch_yaw" and self.run_laser == True and self.target_disappear == True:
             if(self.imu_count == 0):
                 self.initial_yaw = data.values[2]
                 self.imu_count = 1
             if abs(data.values[1]) > 3:
                 self.imu_nav = True
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

if __name__ == "__main__":
    try:
        body = functionAll()
    except:
        rospy.logwarn("functionAll closed!")

