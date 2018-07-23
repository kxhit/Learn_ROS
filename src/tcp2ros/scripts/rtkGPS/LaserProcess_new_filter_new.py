#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from nav_msgs.msg import Path
import numpy as np
import Map_Point
import TF
from tcp2ros.msg import rtkGPSmessage
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import ChannelFloat32
from tcp2ros.msg import readDataAll
from std_msgs.msg import Float32

#dela_t = 1.0
def caculateYaw(y0,x0):
    # y0 = float(y0)
    # x0 = float(x0)
    x = np.array([y0,x0])
    #print "x: " + str(x)
    y = np.array([0,1])
    #print "y: " + str(y)
    Lx = np.sqrt(x.dot(x))
    #print "Lx: " + str(Lx)
    Ly = np.sqrt(y.dot(y))
    #print "Ly: " + str(Ly)
    cos_angle = float(x.dot(y))/(float(Lx*Ly))
    #print "cos_angle: " + str(cos_angle)
    angle = np.arccos(cos_angle)
    #print "angle: " + str(angle)
    if(y0 < 0):
        angle = -angle
    #print "angle_after: " + str(angle)
    if(angle > math.pi):
        angle = angle - 2*math.pi
    elif(angle < -math.pi):
        angle = angle + 2*math.pi
    else:
        angle = angle
    return angle

def quadratic(a,b,c):
    p=b*b-4*a*c
    #if p>=0 and a!=0:#一元二次方程有解的条件
    y1=(-b+math.sqrt(p))/(2*a)
    y2=(-b-math.sqrt(p))/(2*a)
    solution = [y1,y2]
    #print "result:   " + str(x1) + " " + str(x2)
    return solution
    #     return x1,x2
    # elif a==0:#a=0的情况下为一元一次方程
    #     x1=x2=-c/b
    #     return x1
    # else:
    #     return('Wrong Number！')

def Calculate2(y1, x1, y2, x2, dela_t):
    y1 = float(y1)
    x1 = float(x1)
    y2 = float(y2)
    x2 = float(x2)
    dela_t = float(dela_t)
    if y1 == y2:
        if y1 > 0:
            x3 = (x1+x2)/2.0
            y3 = y1 - dela_t
            x0 = (x1 + x2) / 2.0
            y0 = (y1 + y2) / 2.0
            th4 = math.pi/2.0
            return [y3, x3, th4, y0, x0]
        else:
            x3 = (x1+x2)/2.0
            y3 = y1 + dela_t
            x0 = (x1 + x2) / 2.0
            y0 = (y1 + y2) / 2.0
            th4 = -math.pi/2.0
            return [y3, x3, th4, y0, x0]
    elif x1 == x2:
        x3 = x1 - dela_t
        x0 = (x1 + x2) / 2.0
        y0 = (y1 + y2) / 2.0
        y3 = y0
        th4 = 0
        return [y3, x3, th4, y0, x0]
    else:
        k1 = float(x1- x2) / (float(y1 - y2))
        #print "k1_c:  " + str(k1)
        k2 = -(float(1.0))/k1
        #print "k2_c:  " + str(k2)
        x0 = (x1 + x2) / 2.0
        y0 = (y1 + y2) / 2.0

        so = quadratic((k2*k2+1),-2*y0*(k2*k2+1),(y0*y0*(k2*k2+1)-dela_t*dela_t))
        temp_s1 = k2 * (float(so[0]) - y0) + x0
        temp_s2 = k2 * (float(so[1]) - y0) + x0
        if(temp_s1 < temp_s2):
            result = float(so[0])
        else:
            result = float(so[1])

        y3 = float(result)
        x3 = k2 * (y3 - y0) + x0
        th4 = caculateYaw(float(y0 - y3), float(x0 - x3))
        return [y3, x3, th4, y0, x0]


def Find_target2(y1, x1, y2, x2):
    if y1 == y2:
        if y1 < 0:
            th = -math.pi/2.0
            y_t = 0
            x_t = (x1 + x2) / 2.0
            x0 = (x1 + x2) / 2.0
            y0 = (y1 + y2) / 2.0
            distance = math.sqrt((x0 - x_t) * (x0 - x_t) + (y0 - y_t) * (y0 - y_t))
        else:
            th = math.pi/2.0
            y_t = 0
            x_t = (x1 + x2) / 2.0
            x0 = (x1 + x2) / 2.0
            y0 = (y1 + y2) / 2.0
            distance = math.sqrt((x0 - x_t) * (x0 - x_t) + (y0 - y_t) * (y0 - y_t))
    else:
        k1 = float(x1 - x2) / float(y1 - y2)
        k2 = -(float(1.0) / k1)
        x0 = (x1 + x2) / 2.0
        y0 = (y1 + y2) / 2.0
        x_t = x0 - k2*y0
        y_t = 0
        distance = math.sqrt((x0 - x_t) * (x0 - x_t) + (y0 - y_t) * (y0 - y_t))
        th = caculateYaw(float(y0-y_t),float(x0-x_t))
        if (y_t - y0)>0 and (x_t - x0)<0:
            th2 = -math.atan(abs((y_t-y0)/(x_t-x0)))
        elif (y_t - y0)>0 and (x_t - x0)>0:
            th2 = -(math.pi - math.atan(abs((y_t-y0)/(x_t-x0))))
        elif (y_t - y0)<0 and (x_t - x0)>0:
            th2 = math.pi-math.atan(abs((y_t-y0)/(x_t-x0)))
        elif (y_t - y0)<0 and (x_t - x0)<0:
            th2 = math.atan(abs((y_t-y0)/(x_t-x0)))
        #print "th2:  " + str(th2)

    return [y0, x0 + 0.7,th]

def Calculate( y1, x1, y2, x2,dela_t):
    if y1 == y2:
        y1 = y1+0.00001
    if (x1-x2)*(y1-y2)<0 :
        if(y1 > y2):
            th1 = math.atan(abs((x2-x1)/(y1-y2)))
            len1 = math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
            th2 = math.atan(dela_t/(len1/2.0))
            th3 = th2 - th1
            #if th3>0:
            len2 = dela_t / math.sin(th2)
            x3 = x1 - len2 * math.sin(th3)
            y3 = y1 - len2 * math.cos(th3)
            x0 = (x1+x2)/2.0
            y0 = (y1+y2)/2.0
            th4 = math.atan((y0-y3)/(x0-x3))
            return[ y3,x3,th4,y0,x0]
        else:
            th1 = math.atan(abs((x2-x1)/(y1-y2)))
            len1 = math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
            th2 = math.atan(dela_t/(len1/2.0))
            th3 = th2 - th1
            len2 = dela_t / math.sin(th2)
            x3 = x2 - len2 * math.sin(th3)
            y3 = y2 - len2 * math.cos(th3)
            x0 = (x1+x2)/2.0
            y0 = (y1+y2)/2.0
            th4 = math.atan((y0-y3)/(x0-x3))
            return[ y3,x3,th4,y0,x0]
    else:
        if(y1 > y2):
            th1 = math.atan(abs((x2-x1)/(y1-y2)))
            len1 = math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
            th2 = math.atan(dela_t/(len1/2.0))
            th3 = th2 - th1
            len2 = dela_t / math.sin(th2)
            x3 = x2 - len2 * math.sin(th3)
            y3 = y2 + len2 * math.cos(th3)
            x0 = (x1+x2)/2.0
            y0 = (y1+y2)/2.0
            th4 = -math.atan((y3-y0)/(x0-x3))
            return[ y3,x3,th4,y0,x0]
        else:
            th1 = math.atan(abs((x2-x1)/(y1-y2)))
            len1 = math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
            th2 = math.atan(dela_t/(len1/2.0))
            th3 = th2 - th1
            len2 = dela_t / math.sin(th2)
            x3 = x1 - len2 * math.sin(th3)
            y3 = y1 + len2 * math.cos(th3)
            x0 = (x1+x2)/2.0
            y0 = (y1+y2)/2.0
            th4 = -math.atan((y3-y0)/(x0-x3))
            return[ y3,x3,th4,y0,x0]

def Find_target(y1, x1, y2, x2):
    k1 = (x1 - x2) / (y1 - y2)
    k2 = -(1.0 / k1)
    x0 = (x1 + x2) / 2.0
    y0 = (y1 + y2) / 2.0
    x_t = -0.7
    y_t = y0 - (x0-x_t) / k2
    distance = math.sqrt((x0 - x_t) * (x0 - x_t) + (y0 - y_t) * (y0 - y_t))
    return [y_t, x_t, distance]

#这里传进去的参数都是gps坐标系下的坐标值和yaw角信息
def caculateCrossPoint(y0,x0,y1,x1,y_l,x_l,th):
    y0 = float(y0)
    x0 = float(x0)
    x1 = float(x1)
    y1 = float(y1)
    x_l = float(x_l)
    y_l = float(y_l)
    th = float(th)
    if y0 == y1:
        s2 = float(1.0)/float(math.tan(th))
        result_y = y1
        result_x = s2*(result_y-y_l)+x_l
        theta = th
        distance = math.sqrt(math.pow(result_x-x_l,2)+math.pow(result_y-y_l,2))
        return [result_y, result_x, theta,distance]
    else:
        s1 = float(x0-x1)/(float(y0-y1))
        s2 = float(1.0)/float(math.tan(th))
        result_y = (s1*y1-x1-s2*y_l+x_l)/(s1-s2)
        #result_y = solve(s1*(y_-y1)+x1-(s2*(y_-y_l)+x_l),y_)
        result_x = s1*(result_y-y1)+x1
        theta = th
        distance = math.sqrt(math.pow(result_x - x_l, 2) + math.pow(result_y - y_l, 2))
        return [result_y,result_x,theta,distance]

class LaserProcess:
    def __init__(self):
        rospy.init_node('LaserProcess', anonymous=False)
        self.laserprocess_pub = rospy.Publisher('is_laserprocess_work', ChannelFloat32, queue_size=5)
        self.rate = rospy.Rate(50)

        self.stop_radius = 0.6
        self.range_find_radius = 3.0
        self.search_radius = 2.0
        self.reflact_length = 1.0
        self.reach_length = 0.5
        self.dis_flag = 1
        self.back = 0
        self.alignment_distance = 1.0
        self.enter= 0
        self.accept = True
        self.distance = 1.0
        self.Pillar_distance = 1.0
        self.first = 1
        #self.reflact_x
        #self.reflact_y

        #self.reflact1_x = 0
        #self.reflact1_y = 0
        #self.reflact2_x = 0
        #self.reflact2_y = 0

        self.reach_x = 0
        self.reach_y = 0
        self.reach_th = 0#rad

        self.GpsCoordinate1 = [0,0]
        self.GpsCoordinate2 = [0,0]

        #gps
        self.North_base = float(rospy.get_param("North_base",'0.2'))
        self.East_base = float(rospy.get_param("East_base",'0.2'))
        self.Yaw_base = float(rospy.get_param("Yaw_base",'0.01'))
        self.Now_Position_rtkGPS = [self.North_base+0, self.East_base+0,self.Yaw_base]

        rospy.Subscriber("/base_scan", LaserScan, self.callback1)
        self.pub = rospy.Publisher('laser_target_position', ChannelFloat32, queue_size=5)

        rospy.Subscriber("rtkGPS_filter", rtkGPSmessage, self.callback2, queue_size=5)
        rospy.Subscriber('laser_alignment_distance',Float32,self.callback3,queue_size=5)

        rospy.Subscriber("/tcpData", readDataAll, self.callback4, queue_size=5)
        rospy.Subscriber('change_radius', Float32, self.callback5, queue_size=10)
        rospy.Subscriber("path_node", Path, self.callback6, queue_size=1)
        #countt=0
        while not rospy.is_shutdown():
            laser_process_workstate = ChannelFloat32()
            laser_process_workstate.name = "working"
            self.laserprocess_pub.publish(laser_process_workstate)
            self.rate.sleep()
            #countt = countt+1



    def callback1(self, data):
        #print data.angle_min
        #print len(data.ranges)
        i_state = False
        i_start=0
        i_end=0
        i_num=0
        self.reflact_x = []
        self.reflact_y = []
        self.reflact1_x = []
        self.reflact1_y = []
        self.reflact2_x = []
        self.reflact2_y = []

        self.reflact_i_start = []
        self.reflact_i_end = []
        self.reflact1_i_start = []
        self.reflact1_i_end = []
        self.reflact2_i_start = []
        self.reflact2_i_end = []
        

        reflact_num_find = 0
        #intensitity_max = 0

        stop_flag = False
        for i in range(0, len(data.ranges)):
            if data.ranges[i] < self.stop_radius and data.ranges[i] > 0.03 and (i * data.angle_increment - 5.0 * math.pi / 180) > 0 and (i * data.angle_increment - 5.0 * math.pi / 180) < math.pi:
                stop_flag = True
        pub_stop_data = ChannelFloat32()
        pub_stop_data.name = "stop_flag"
        if stop_flag:
            pub_stop_data.values.append(1.0)
            # print "stop"
        else:
            pub_stop_data.values.append(0.0)
            # print "not stop"
        self.pub.publish(pub_stop_data)

        # back home check1
        back_home_check1 = False
        for i in range(0, len(data.ranges)):
            if data.ranges[i] < 7.2 and data.ranges[i] > 1.3 and (
                    i * data.angle_increment - 5.0 * math.pi / 180) > math.pi / 2.0 - 0.263 and (
                    i * data.angle_increment - 5.0 * math.pi / 180) < math.pi / 2 + 0.263:
                if i_state is False:
                    i_state = True
                    i_start = i
                    i_num = i_num + 1
                else:
                    i_num = i_num + 1
            else:
                if i_state is True:
                    i_state = False
                    i_end = i
                    if i_num > 20:  # find~
                        back_home_check1 = True
                        # print "back_home_check1 find"
                        # print "____"
                        # print reflact_num_find
                        # else:
                        # print "no use check1"

                    i_start = 0
                    i_end = 0
                    i_num = 0

        pub_back_home_check1 = ChannelFloat32()
        pub_back_home_check1.name = "first_search"
        if back_home_check1:
            pub_back_home_check1.values.append(1.0)
        else:
            pub_back_home_check1.values.append(0.0)
        self.pub.publish(pub_back_home_check1)

        # pub_back_home_check2 = ChannelFloat32()
        # pub_back_home_check2.name = "second_search"
        # if back_home_check2:
        #     pub_back_home_check2.values.append(1.0)
        # else:
        #     pub_back_home_check2.values.append(0.0)
        # self.pub.publish(pub_back_home_check2)

        i_state = False
        for i in range(0, len(data.ranges)):
            #240是代表激光强度阈值，实际应用时按需修改
            if data.ranges[i] < self.range_find_radius and data.intensities[i]>240:
                if self.first == 1:
                    self.first = 0
                    return

                d_x = data.ranges[i] * math.sin(data.angle_increment * i - 5.0 * math.pi / 180)-data.ranges[i-1] * math.sin(data.angle_increment * (i-1) - 5.0 * math.pi / 180)
                d_y = data.ranges[i] * math.cos(data.angle_increment * i - 5.0 * math.pi / 180)-data.ranges[i-1] * math.cos(data.angle_increment * (i-1) - 5.0 * math.pi / 180)
                temp = math.sqrt(d_x*d_x+d_y*d_y)

            if data.ranges[i] < self.range_find_radius and data.intensities[i]>240 and temp<0.1:
                if i_state is False:
                    i_state = True
                    i_start = i
                    i_num = i_num+1
                else:
                    i_num = i_num + 1
            else:
                if i_state is True:
                    i_state = False
                    i_end = i
                    if i_num>5:#find~
                        x_temp = 0
                        y_temp = 0
                        for ii in range(i_start, i_end):
                            #x_temp = x_temp + data.ranges[ii]*math.sin(data.angle_increment*ii)
                            #y_temp = y_temp - data.ranges[ii]*math.cos(data.angle_increment*ii)
                            x_temp = x_temp + data.ranges[ii]*math.sin(data.angle_increment*ii - 5.0*math.pi/180)
                            y_temp = y_temp - data.ranges[ii]*math.cos(data.angle_increment*ii - 5.0*math.pi/180)
                        x_temp = x_temp / i_num
                        y_temp = y_temp / i_num
                        #print x_temp,y_temp #1
                        self.reflact_x.append(x_temp)
                        self.reflact_y.append(y_temp)
                        self.reflact_i_start.append(i_start)
                        self.reflact_i_end.append(i_end)
#                        self.reflact_x[reflact_num_find] = x_temp
#                       self.reflact_y[reflact_num_find] = y_temp
                        reflact_num_find = reflact_num_find + 1
                        #print self.reflact_x,self.reflact_y
                        #print "find one" #1
                        #print reflact_num_find

                    #else:
                    #    print "no use"

                    i_start = 0
                    i_end = 0
                    i_num = 0

        #print "reflact_num_find" + str(reflact_num_find)
        for i in range(0,len(self.reflact_x)):
            for j in range(i+1,len(self.reflact_x)):
                #print self.reflact_x[i],self.reflact_y[i]
                if reflact_num_find > 1:
                    temp_length = math.sqrt((self.reflact_x[i]-self.reflact_x[j])*(self.reflact_x[i]-self.reflact_x[j])+(self.reflact_y[i]-self.reflact_y[j])*(self.reflact_y[i]-self.reflact_y[j]))
                    #print temp_length #1
                    #print "========"
					#print (0.2*self.Pillar_distance)
                    #print abs(self.reflact_x[i] - self.reflact_x[j])
		    
                    if temp_length>(0.5*self.Pillar_distance) and temp_length<(1.5*self.Pillar_distance) and (abs(self.reflact_x[i] - self.reflact_x[j]) < 20):
                        self.reflact1_x.append(self.reflact_x[i])
                        self.reflact1_y.append(self.reflact_y[i])
                        self.reflact2_x.append(self.reflact_x[j])
                        self.reflact2_y.append(self.reflact_y[j])
                        self.reflact1_i_start.append(self.reflact_i_start[i])
                        self.reflact1_i_end.append(self.reflact_i_end[i])
                        self.reflact2_i_start.append(self.reflact_i_start[j])
                        self.reflact2_i_end.append(self.reflact_i_end[j])

        if len(self.reflact1_x) > 0:
            min_point_x = self.reflact1_x[0]+self.reflact2_x[0]
            min_index = 0
            for i in range(0, len(self.reflact1_x)):
                if ((self.reflact1_x[i]+self.reflact2_x[i]) < min_point_x):
                    min_point_x = self.reflact1_x[i]+self.reflact2_x[i]
                    min_index = i

            if self.dis_flag == 1:#-- the sentance is not necessary
                result = Find_target2(self.reflact1_y[min_index], self.reflact1_x[min_index], self.reflact2_y[min_index],self.reflact2_x[min_index])
                TF_body = TF.TF()
                rtkGPS_Target_midpoint = TF_body.BaseLinktortkGPS(result, self.Now_Position_rtkGPS)
                rtkGPS_Target = caculateCrossPoint(self.GpsCoordinate1[0], self.GpsCoordinate1[1],self.GpsCoordinate2[0], self.GpsCoordinate2[1],rtkGPS_Target_midpoint[1], rtkGPS_Target_midpoint[0],rtkGPS_Target_midpoint[2])

                pub_rtkGPS_data = ChannelFloat32()
                pub_rtkGPS_data.name = "target_position_rtk"
                pub_rtkGPS_data.values.append(rtkGPS_Target[1])
                pub_rtkGPS_data.values.append(rtkGPS_Target[0])
                pub_rtkGPS_data.values.append(rtkGPS_Target[2])
                pub_rtkGPS_data.values.append(rtkGPS_Target[3])
                self.pub.publish(pub_rtkGPS_data) #-- publish the gps coordinate of target point
                #print "transform to GPS"
                #self.dis_flag = 0


            self.pub_data = ChannelFloat32()
            self.pub_data.name = "laser_target_position_now"

            self.res = Calculate2(self.reflact1_y[min_index], self.reflact1_x[min_index], self.reflact2_y[min_index], self.reflact2_x[min_index],self.alignment_distance)
            self.pub_data.values.append(self.res[1])
            self.pub_data.values.append(self.res[0])
            self.pub_data.values.append(self.res[2])
            self.pub_data.values.append(0)
            #print "go"
            #print self.res[1], self.res[0], self.res[2]/math.pi*180 #1
            self.pub.publish(self.pub_data)

        else:
            pub_find_none = ChannelFloat32()
            pub_find_none.name = "find_none"
            self.pub.publish(pub_find_none)


    def callback2(self, data):
        if data.flash_state == 'YAW':
            if data.north_meter != 0 and data.east_meter != 0:
                self.Now_Position_rtkGPS[2] = data.yaw_rad
        if data.flash_state == 'POSITION':
            if data.north_meter != 0 and data.east_meter != 0:
                self.Now_Position_rtkGPS[0] = data.north_meter
                self.Now_Position_rtkGPS[1] = data.east_meter

    def callback3(self,data):
        self.alignment_distance = float(data.data)

    def callback4(self,data):
        self.Pillar_distance = data.Pillar_distance
        #print "Pillar_distance"
	    #print self.Pillar_distance

    def callback5(self,data):
        self.stop_radius = float(data.data)

    def callback6(self, data):
        if (len(data.poses) > 1):
            # self.GpsCoordinate1_x = data.poses[len(data.poses) - 1].pose.position.x
            # self.GpsCoordinate1_y = data.poses[len(data.poses) - 1].pose.position.y
            # self.GpsCoordinate2_x = data.poses[len(data.poses) - 2].pose.position.x
            # self.GpsCoordinate2_y = data.poses[len(data.poses) - 2].pose.position.y
            self.GpsCoordinate1 = [data.poses[len(data.poses) - 1].pose.position.y,data.poses[len(data.poses) - 1].pose.position.x]
            self.GpsCoordinate2 = [data.poses[len(data.poses) - 2].pose.position.y,data.poses[len(data.poses) - 2].pose.position.x]

if __name__ == "__main__":
    try:
        body = LaserProcess()
    except:
        rospy.logwarn("LaserProcess closed!")
