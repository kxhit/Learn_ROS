#!/usr/bin/env python
# license removed for brevity
import rospy
import Motion_Planning
import Map_Point
import TF
import math
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from tcp2ros.msg import rtkGPSmessage
from sensor_msgs.msg import ChannelFloat32

class functionAll:
     def __init__(self):
         rospy.init_node('FunctionAll', anonymous=False)
         rospy.Subscriber("joy", Joy, self.callback1)
         rospy.Subscriber("rtkGPS", rtkGPSmessage, self.callback2, queue_size=1)
         rospy.Subscriber("laser_target_position", ChannelFloat32, self.callback3, queue_size=1)

         self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
         self.rate = rospy.Rate(30)
         self.cmd = Twist()
         self.cmd.linear.x = 0.0
         self.cmd.linear.y = 0.0
         self.cmd.linear.z = 0
         self.cmd.angular.z = 0.0
         self.run_state = 0#0 is going to map point. 1 is going to angle
         self.run_laser = False # false is use gps information to go  else use laser
         self.stop_flag = False#if laser find there is something before it
         self.stop_count = 0

         self.debug_print_count=0

         self.MapPointNumLaserIndex = [4,11,30]#id is laser search point

         self.MotionPlaner_speedXY = float(rospy.get_param("MotionPlaner_speedXY",'1.0'))
         self.MotionPlaner_speedYaw = float(rospy.get_param("MotionPlaner_speedYaw",'0.2'))
         self.MotionPlaner_errorMeter = float(rospy.get_param("MotionPlaner_errorMeter",'0.3'))#m
         self.MotionPlaner_errorAngle = float(rospy.get_param("MotionPlaner_errorAngle",'0.1'))#rad

         self.MotionPlaner = Motion_Planning.Motion_Planning(self.MotionPlaner_speedXY, self.MotionPlaner_speedYaw, self.MotionPlaner_errorMeter, self.MotionPlaner_errorAngle)
         self.MapPoint = Map_Point.Map_Point()
         self.TF_body = TF.TF()
         #MapPoint is in rtkGPS coordinate system

         self.North_base = float(rospy.get_param("North_base",'0.2'))
         self.East_base = float(rospy.get_param("East_base",'0.2'))
         self.Yaw_base = float(rospy.get_param("Yaw_base",'0.0'))
         #print self.North_base ,self.East_base

         self.MapPoint.add_point(4630822.0000, 786263.1875, 6.2712)
         self.MapPoint.add_point(4630831.0000, 786262.7500, 0.0498)
         self.MapPoint.add_point(4630831.5000, 786262.0625, 1.4956)
         self.MapPoint.add_point(4630832.0000, 786267.4375, 1.7700)
         self.MapPoint.add_point(4630833.0000, 786267.6250, 3.1416)
         #laser4
         self.MapPoint.add_point(4630832.5000, 786268.3750, 4.6559)
         self.MapPoint.add_point(4630832.0000, 786263.6250, 4.6295)
         self.MapPoint.add_point(4630831.5000, 786263.5000, 6.2556)
         self.MapPoint.add_point(4630838.0000, 786263.1250, 6.2230)
         self.MapPoint.add_point(4630838.0000, 786262.3125, 1.5548)
         self.MapPoint.add_point(4630838.5000, 786267.7500, 1.5243)
         self.MapPoint.add_point(4630839.0000, 786267.9375, 3.1604)
         #laser11
         self.MapPoint.add_point(4630839.0000, 786268.6875, 4.6752)
         self.MapPoint.add_point(4630838.5000, 786263.0625, 4.6457)
         self.MapPoint.add_point(4630838.5000, 786262.3750, 3.1260)
         self.MapPoint.add_point(4630823.0000, 786263.1250, 3.1930)
         self.MapPoint.add_point(4630822.0000, 786263.5625, 0.0486)


         
         #self.MapPoint.add_point(3349259.83,  511222.59,3.47215795517)
      
         
         self.MapPointNum_Now = 0
         self.MapPointNum_All = len(self.MapPoint.pointlist_x)

         self.Position_now_rtkGPS = [self.North_base+0, self.East_base+0,self.Yaw_base]
         self.EnableFlag = False
         self.EnableCount = [0,0]
         self.Position_BaseLink_Target=[0,0,0]

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
                 self.Position_BaseLink_Target=self.TF_body.rtkGPStoBaseLink(self.MapPoint.get_point(self.MapPointNum_Now), self.Position_now_rtkGPS)
                 self.debug_print_count =  self.debug_print_count+1
                 if  self.debug_print_count >=10:
                     print self.Position_BaseLink_Target
                     self.debug_print_count=0
                 self.MotionPlaner.setNowPosition([0,0,0])
                 self.MotionPlaner.setNextTarget(self.Position_BaseLink_Target)
                 self.Motion_temp = self.MotionPlaner.MotionPlan()#get speed wanted
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

             if self.run_state is 0 and self.run_laser == False:
                 if self.MotionPlaner.reach_position():
                     self.run_state = 1

             if self.run_state is 1 and self.run_laser == False:
                 if self.MotionPlaner.reach_angle():
                     self.run_state = 0
                     print "MapPointNum_Now" + str(self.MapPointNum_Now)

                     for ii in range (len(self.MapPointNumLaserIndex)):
                         if self.MapPointNumLaserIndex[ii] == self.MapPointNum_Now:
                             self.run_laser = True
                             self.MotionPlaner.setValues(0.2,0.2,0.05,0.042)
                             print "Laser start"
                     self.MapPointNum_Now = self.MapPointNum_Now+1
                     if self.MapPointNum_Now == self.MapPointNum_All:
                          self.MapPointNum_Now = self.MapPointNum_All-1

     def callback3(self, data):
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
         if self.run_laser and data.name == "laser_target_position_now":
             self.debug_print_count = self.debug_print_count + 1
             if self.debug_print_count >= 10:
                 print data.values[0], data.values[1], data.values[2]/math.pi*180
                 self.debug_print_count=0
             self.MotionPlaner.setNowPosition([-0.6, 0, 0])
             self.MotionPlaner.setNextTarget((data.values[0], data.values[1], data.values[2]))
             self.Motion_temp = self.MotionPlaner.MotionPlan()# get speed wanted
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
                 if self.MotionPlaner.reach_position():
                     self.run_state = 1
                     print "reach position"

             if self.run_state is 1:
                 if self.MotionPlaner.reach_angle():
                     self.run_state = 0
                     print "reach angle"
                     if self.MotionPlaner.reach_position():#may be loop forever
                         self.MotionPlaner.setValues(self.MotionPlaner_speedXY, self.MotionPlaner_speedYaw, self.MotionPlaner_errorMeter, self.MotionPlaner_errorAngle)
                         self.run_laser = False
                         self.cmd.linear.x = 0
                         self.cmd.linear.y = 0
                         self.cmd.linear.z = 0
                         self.cmd.angular.z = 0
                         self.cmd_vel_pub.publish(self.cmd)
                         print "Laser End"





if __name__ == "__main__":
    try:
        body = functionAll()
    except:
        rospy.logwarn("functionAll closed!")

