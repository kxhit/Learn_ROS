#!/usr/bin/env python
# -*- coding:utf-8 -*-

import math
import rospy
import tf
from nav_msgs.msg import Odometry
import numpy as np
from numpy import *
from sensor_msgs.msg import Imu
from tcp2ros.msg import rtkGPSmessage
from sensor_msgs.msg import ChannelFloat32

from filterpy.kalman import KalmanFilter
import TF
import logging

durTime=0.06

# 设置KF
def FirstOrderKF(R, Q, dt):
    kf = KalmanFilter(dim_x=4, dim_z=4)
    kf.x = np.array([[0, 0, 0, 0]]).T
    kf.P = np.eye(4) * 0.0001
    kf.R = [[0.04017680, 0., 0., 0.],
            [0., 0.04082702, 0., 0.],
            [0., 0., 0.0002017680, 0.],
            [0., 0., 0., 0.0002017680]]
    # np.eye(3) * R
    kf.Q = np.eye(4) * Q
    kf.F = np.array([[1, dt, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, dt],
                     [0, 0, 0, 1]])
    kf.H = np.array([[0, 1., 0, 0],
                     [0, 0, 0, 1.],
                     [1., 0, 0, 0],
                     [0, 0, 1., 0]])
    return kf


# batch滤波
def xyz_kalman_fliter(zs):
    robot_tracker = FirstOrderKF(0.0002, 0.0000001, 0.06)
    mu, cov, _, _ = robot_tracker.batch_filter(zs)
    return mu


# 每一时刻滤波
def xyz_kalman_filter_time(robot_tracker, zs):
    prior = robot_tracker.predict()
    robot_tracker.update(zs)
    return robot_tracker.x


# 初始的原始车体中心gps位姿,north,earth,yaw
init_gps_pose = [0.0,0.0,0.0]

# 每一帧的原始车体中心gps位姿,north,earth,yaw
gps_pose = [0.0,0.0,0.0]

# 基于第一帧车体坐标系的当前车体坐标系位姿,x,y,yaw
base_pose = [0.0,0.0,0.0]

# 里程计信息
odom_pose = [0.0,0.0,0.0]
odom_vel = [0.0,0.0,0.0]

# imu信息
imuP_pose = [0.0,0.0,0.0]
imuP_vel = [0.0,0.0,0.0]
# 当前和上一时刻imu角度偏差
errCurAndLastImuTh = 0
last_imuP_th = 0

# 是否获得第一帧gps
first_gps = False
# 是否初始化了robot_tracker
first_mix = False
# 当前gps是否有效
ifGPS = False
last_ifGPS = False
last_gps=rtkGPSmessage()
current_gps=rtkGPSmessage()
gps_count=0
gps_num=0
gps_length=10
gps_windows=[rtkGPSmessage()]*gps_length
init_ave_num=4

# 结果
result = [0.0, 0.0, 0.0, 0.0]
# 上一时刻结果
old_result = [0.0, 0.0, 0.0, 0.0]
result_th = 0.0

#结果
result_base_pose=[0.0,0.0,0.0]
result_gps_pose=[0.0,0.0,0.0]

# gps在车体的位姿
# gps_length_x = 0.46
# gps_length_y = 0.21

# 是否获取到imu
imu_get = False

# Pai和P
P = 57.2956
pai = 3.1415926
period = 0.0
updated_= rospy.Time()
# robot_tracker = FirstOrderKF(0.0002,0.0000001,0.00002017680,0.02017680,0.04)

# kalman滤波
robot_tracker = FirstOrderKF(0.0002, 0.0000001, 0.06)
# 发布滤波后的里程计
pub = rospy.Publisher('/odom_filter', Odometry, queue_size=3)
pub1 = rospy.Publisher('/rtkGPS_filter', rtkGPSmessage, queue_size=3)
pub2 = rospy.Publisher('/odom_imu_gps_start', ChannelFloat32, queue_size=3)

# 滤波后的tf变换
odom_broadcaster = tf.TransformBroadcaster()

myTF = TF.TF()

flash_state_list=["YAW","POSITION"]
flash_state_ind=0

# 角度范围限制
def check_angle(angle):
    if angle > pai:
        angle = -2 * pai + angle
    elif angle < -pai:
        angle = 2 * pai + angle
    return angle

def check_angle1(angle):
    if angle > 2*pai:
        angle = -2 * pai + angle
    elif angle < 0:
        angle = 2 * pai + angle
    return angle


# 更新IMU位置
def updateImuP():
    # 计算在当前坐标系下当前IMU的角度和使用IMU获得的里程计位置
    imuP_th = check_angle(imuP_pose[2])

    delta_Imux = (odom_vel[0] * cos(imuP_th) - odom_vel[1] * sin(imuP_th)) * period;
    delta_Imuy = (odom_vel[0] * sin(imuP_th) + odom_vel[2] * cos(imuP_th)) * period;
    imuP_pose[0] += delta_Imux
    imuP_pose[1] += delta_Imuy


# 获取imu数据
def imuCallback(imu):
    global imu_get
    global imuP_pose

    imu_get = True
    w = imu.orientation.w
    x = imu.orientation.x
    y = imu.orientation.y
    z = imu.orientation.z
    imuP_pose[2] = math.atan2(2.0 * (w * z + x * y), (1.0 - 2.0 * (y * y + z * z)))

    # 限制在-PI/2到PI之间
    imuP_pose[2] = check_angle(imuP_pose[2])

# 发布滤波后的里程计
def pub_odom_filter(pub,stamp):
    global result_base_pose
    global result_gps_pose
    global result_th
    global flash_state_ind

    # 结果

    # 当前在第一帧坐标系下的车体位姿
    result_base_pose = [0.0,0.0,0.0]
    result_base_pose[0] = result[0]
    result_base_pose[1] = result[2]
    result_base_pose[2] = check_angle(result_th)

    #print "result_base_pose: "+str(result_base_pose[0])+" "+str(result_base_pose[1])+" "+str(result_base_pose[2])

    # 实际的gps位姿
    result_gps_pose = myTF.BaseToGPS(result_base_pose, init_gps_pose)
    result_gps_pose[2] = check_angle1(result_gps_pose[2])

    pub_temp = rtkGPSmessage()
    pub_temp.ROS_time = stamp
    pub_temp.GPS_time = current_gps.GPS_time
    pub_temp.vaild_flag = ifGPS
    pub_temp.flash_state = flash_state_list[flash_state_ind]
    pub_temp.north_meter = result_gps_pose[0]
    pub_temp.east_meter = result_gps_pose[1]
    pub_temp.yaw_rad = result_gps_pose[2]
    pub1.publish(pub_temp)

    flash_state_ind += 1
    flash_state_ind = flash_state_ind % 2

    # send the transform
    odom_broadcaster.sendTransform((result_base_pose[0], result_base_pose[1], 0),
                                   # tf.transformations.quaternion_from_euler(0, 0, result[4]/P+3.14),
                                   (0, 0, sin(result_base_pose[2] / 2), cos(result_base_pose[2] / 2)),
                                   stamp,
                                   "base_link",
                                   "odom_filter"
                                   )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = stamp
    odom.header.frame_id = "odom_filter"
    odom.child_frame_id = "base_link"

    # set the position
    odom.pose.pose.position.x = result_base_pose[0]
    odom.pose.pose.position.y = result_base_pose[1]
    odom.pose.pose.position.z = 0.0
    # odom.pose.pose.orientation=odom_quat
    odom.pose.pose.orientation.x = 0
    odom.pose.pose.orientation.y = 0
    odom.pose.pose.orientation.z = sin((result_base_pose[2]) / 2)
    odom.pose.pose.orientation.w = cos((result_base_pose[2]) / 2)

    # set the velocity
    odom.header.frame_id = "odom_filter"
    odom.child_frame_id = "base_link"
    odom.twist.twist.linear.x = result[1]
    odom.twist.twist.linear.y = result[3]
    odom.twist.twist.angular.z = odom_vel[2]

    #print result_base_pose[0],result_base_pose[1],result_base_pose[2]
    pub.publish(odom)


# 里程计获取，最主要的滤波地方
def odomCallback(odom):
    global result
    global old_result
    global updated_
    global first_mix
    global odom_pose
    global odom_vel
    global last_imuP_th
    global imuP_pose
    global base_pose
    global result_th

    # 原始里程计的位置
    temp_odom_x = odom.pose.pose.position.x
    temp_odom_y = odom.pose.pose.position.y
    x = odom.pose.pose.orientation.x
    y = odom.pose.pose.orientation.y
    z = odom.pose.pose.orientation.z
    w = odom.pose.pose.orientation.w
    # 原始里程计的角度
    temp_odom_th = math.atan2(2.0 * (w * z + x * y), (1.0 - 2.0 * (y * y + z * z)))

    # 获取imu的差值
    if (imuP_pose[2] >= 0 and last_imuP_th >= 0) or (imuP_pose[2] <= 0 and last_imuP_th <= 0):
        errCurAndLastImuTh = imuP_pose[2] - last_imuP_th
    elif imuP_pose[2] < 0 and last_imuP_th > 0:
        errCurAndLastImuTh = -imuP_pose[2] + last_imuP_th
    elif imuP_pose[2] > 0 and last_imuP_th < 0:
        errCurAndLastImuTh = imuP_pose[2] - last_imuP_th
    last_imuP_th = imuP_pose[2]

    # 原始里程计的速度和角速度
    odom_vel[0] = odom.twist.twist.linear.x
    odom_vel[1] = odom.twist.twist.linear.y
    odom_vel[2] = odom.twist.twist.angular.z

    start_flag = ChannelFloat32()
    start_flag.name = "odom_imu_gps_start"
    start_flag.values.append(1.0)
    pub2.publish(start_flag)

    # 是否获得gps初值
    if first_gps == True and init_gps_pose[2] != 0 and init_gps_pose[2] != 2 * pai:
        # period = rospy.get_time()-updated_
        # updated_ = rospy.get_time()

        period = (odom.header.stamp-updated_).nsecs/1000000000.0
        updated_ = odom.header.stamp

        odom_th = check_angle(odom_pose[2])
        base_pose[2] = check_angle(base_pose[2])

        # 初始化robot_tracker
        if first_mix == False:
            robot_tracker.x = np.array([[0, 0, 0, 0]]).T
            first_mix = True

        #ifGPS=False
        # 有GPS信息，则kalman
        if ifGPS == True:

            # 根据当前的base_th角度去获取当前速度，用来更新
            odom_vx = odom_vel[0] * cos(float(base_pose[2])) - odom_vel[1] * sin(float(base_pose[2]))
            odom_vy = odom_vel[0] * sin(float(base_pose[2])) + odom_vel[1] * cos(float(base_pose[2]))
            # 当前里程计的角速度
            odom_vth = odom_vel[2]

            '''
            robot_tracker.R=[[0.02017680,   0.,              0.,                  0.,             0.,              0.],
                                        [0.,             0.02082702,    0.,                  0.,             0.,              0.],
                                        [0.,             0.,              0.0407528,        0.,             0.,              0.],
                                        [0.,             0.,              0.,                  0.00002017680,   0.,              0.],
                                        [0.,             0.,              0.,                  0.,             0.00002017680,    0.],
                                        [0.,             0.,              0.,                  0.,             0.,              0.00002017680]]
            '''
            robot_tracker.R = [[0.04017680, 0., 0., 0.],
                               [0., 0.04082702, 0., 0.],
                               [0., 0., 0.002017680, 0.],
                               [0., 0., 0., 0.002017680], ]
            # 加入测量值，odom_xv,odom_vy,base_x,base_y
            temp = []
            temp.append(odom_vx)
            temp.append(odom_vy)
            # temp.append(odom_vth)
            temp.append(base_pose[0])
            temp.append(base_pose[1])
            # temp.append(base_th)

            zs = np.array([temp[:]]).T
            # 获取结果
            result = xyz_kalman_filter_time(robot_tracker, zs)
            old_result = result

            base_pose[2] = check_angle(base_pose[2])
            result_th = check_angle(base_pose[2])

            #print "result: "+str(result[0])+" "+str(result[2])+" "+str(base_pose[2])
            result[0] = base_pose[0]
            result[1] = odom_vel[0]
            result[2] = base_pose[1]
            result[3] = odom_vel[1]

        else:
            #有imu情况下，根据imu的差值更新角度
            if imu_get == True:
                base_pose[2] += float(errCurAndLastImuTh)
                result_th += float(errCurAndLastImuTh)
            else:
                #根据odom_vth计算当前角度
                base_pose[2] += float(odom_vel[2]) * period
                result_th += float(odom_vel[2]) * period

            base_pose[2] = check_angle(base_pose[2])
            result_th = check_angle(result_th)

            # 更新速度
            odom_vx = odom_vel[0] * cos(float(result_th)) - odom_vel[1] * sin(float(result_th))
            odom_vy = odom_vel[0] * sin(float(result_th)) + odom_vel[1] * cos(float(result_th))

            # 更新位置
            result[0] = result[0] + odom_vx * period
            result[1] = odom_vel[0]
            result[2] = result[2] + odom_vy * period
            result[3] = odom_vel[1]

            #robot_tracker.x=np.array([result]).T

    pub_odom_filter(pub,odom.header.stamp)


# 获取gps
def gpsOdomCallback(message):
    global first_gps
    global ifGPS
    global init_gps_pose
    global gps_pose
    global base_pose
    global gps_count
    global gps_windows
    global last_gps
    global current_gps
    global last_ifGPS
    global gps_num
    global result
    global result_th
    global result

    logging.debug("vaild_flag " + str(message.vaild_flag) + " flash_state "+message.flash_state)
    current_gps=message

    # 如果收到gps数据
    if message.vaild_flag == True:
        if last_gps.north_meter != 0 and last_gps.east_meter != 0 and last_gps.yaw_rad != 0:
            if message.flash_state=='POSITION' and last_gps.flash_state=='YAW' and last_gps.vaild_flag==True:
                # gps标志位为true
                ifGPS = True
            elif message.flash_state=='YAW' and last_gps.flash_state=='POSITION' and last_gps.vaild_flag==True:
                ifGPS = True
            else:
                ifGPS = False
        else:
            ifGPS = False
    else:
        # gps标志位为false
        ifGPS = False

    # gps标志位为true
    if ifGPS == True:
        # gps值有效
        if message.north_meter != 0 and message.east_meter != 0 and message.yaw_rad != 0:

            gps_pose[0] = message.north_meter
            gps_pose[1] = message.east_meter
            gps_pose[2] = message.yaw_rad

            # 如果是第一帧的gps
            if first_gps == False and gps_num>gps_length:
                gps_init_windows_x=np.array(gps_windows)[:,0]
                gps_init_windows_y=np.array(gps_windows)[:,1]
                gps_init_windows_yaw=np.array(gps_windows)[:,2]
                gps_init_windows_x_sort=sorted(gps_init_windows_x)
                gps_init_windows_y_sort=sorted(gps_init_windows_y)
                gps_init_windows_yaw_sort=sorted(gps_init_windows_yaw)

                sum_x=0
                sum_y=0
                sum_yaw=0
                n=0
                for i in range(gps_length/2-init_ave_num/2,gps_length/2+init_ave_num/2):
                    sum_x+=gps_init_windows_x_sort[i]
                    sum_y+=gps_init_windows_y_sort[i]
                    sum_yaw+=gps_init_windows_yaw_sort[i]
                    n+=1

                # gps的初始位置,在gps坐标系下的位置
                init_gps_pose[0] = sum_x/n# message.yaw_rad
                init_gps_pose[1] = sum_y/n #message.north_meter
                init_gps_pose[2] = sum_yaw/n #message.east_meter

                # 获得了初始gps位置
                first_gps = True

                # 转换到相对于第一帧的车体坐标系下的位姿
                base_pose = myTF.rtkGPStoBaseLink(gps_pose, init_gps_pose)
                result_th = base_pose[2]

            elif gps_num>gps_length:
                # 转换到相对于第一帧的车体坐标系下的位姿
                base_pose = myTF.rtkGPStoBaseLink(gps_pose, init_gps_pose)

        gps_windows[gps_count] = gps_pose
        gps_count += 1
        gps_num +=1
        gps_count = gps_count % gps_length

        # print ("x is %f"%(gps_pose[0]))
        # print ("y is %f"%(gps_pose[1]))
        # print ("th is %f"%(gps_pose[2]))
        # result[0]=base_pose[0]
        # result[2]=base_pose[1]
        # pub_odom_filter(pub)

    if first_gps==True and ifGPS == False and last_ifGPS == True and gps_num>gps_length:
        gps_corrent_num = (gps_count+1) % gps_length
        base_pose = myTF.rtkGPStoBaseLink(gps_windows[gps_corrent_num],init_gps_pose)
        result[0]=base_pose[0]
        result[1]=base_pose[1]
        result_th=base_pose[2]
        print "base_pose: "+str(base_pose[0])+" "+str(base_pose[1])+" "+str(base_pose[2])

    last_ifGPS = ifGPS
    last_gps = message

    #print ifGPS

if __name__ == "__main__":
    rospy.init_node('odom_filter')

    logging.basicConfig(filename='cc.txt',
                        filemode='w',
                        format='%(asctime)s,%(msecs)d %(name)s %(levelname)s %(message)s',
                        datefmt='%H:%M:%S',
                        level=logging.DEBUG)

    #updated_ = rospy.get_time()

    sub1 = rospy.Subscriber('/odom', Odometry, odomCallback)
    sub2 = rospy.Subscriber('/rtkGPS', rtkGPSmessage, gpsOdomCallback)
    sub3 = rospy.Subscriber('/imu/data', Imu, imuCallback)

    rate = rospy.Rate(20)

    print 'start filter'
    while not rospy.is_shutdown():
        rospy.spin()
