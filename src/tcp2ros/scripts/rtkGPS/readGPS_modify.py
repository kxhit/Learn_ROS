#!/usr/bin/env python
# license removed for brevity
import mySerial
import rospy
from tcp2ros.msg import rtkGPSmessage
import math
import time
from std_msgs.msg import String

gps_length_x=0.420
gps_length_y=0.340

class readGPS:
    def __init__(self, serNum=None,bandrate=None):
        #time.sleep(5)
        #self.Serial_name = rospy.get_param("GPSport_name",'/dev/ttyS2')
        self.Serial_name = "/dev/ttyS3"
        #self.Serial_baudrate = rospy.get_param("GPSport_baudrate","115200")
        self.Serial_baudrate = 115200
        self.serial = mySerial.mySerial()
        self.serial.openSerial(self.Serial_name, int(self.Serial_baudrate))
        print "GPS serial "+self.Serial_name+" open"
        
        self.Data_Valid_Flag = 0
        self.E_POSITION = 0.0
        self.N_POSITION = 0.0
        self.YAW_POSITION = 0.0
        
    def readData(self):
        pub = rospy.Publisher('rtkGPS', rtkGPSmessage, queue_size=10)
        rospy.init_node('rtkGPS_node', anonymous=False)
        rate = rospy.Rate(30) # 10hz
        while not rospy.is_shutdown():
            self.temp = self.serial.readVal()
            if self.temp:
                self.strlist = self.temp.split(',')
                #print self.strlist
                #print self.temp, self.Data_Valid_Flag
                temp = self.processData()
                if temp == 'YAW':
                    pub_temp = rtkGPSmessage()
                    pub_temp.ROS_time = rospy.get_rostime()
                    pub_temp.GPS_time = "None"
                    pub_temp.vaild_flag = True
                    pub_temp.flash_state = 'YAW'
                    pub_temp.north_meter = self.N_POSITION+gps_length_y * math.sin(self.YAW_POSITION) + gps_length_x * math.cos(self.YAW_POSITION)
                    pub_temp.east_meter = self.E_POSITION - gps_length_y * math.cos(self.YAW_POSITION) + gps_length_x * math.sin(self.YAW_POSITION)
                    pub_temp.yaw_rad = self.YAW_POSITION
                    pub.publish(pub_temp)

                if temp == 'POSITION':
                    pub_temp = rtkGPSmessage()
                    pub_temp.ROS_time = rospy.get_rostime()
                    pub_temp.GPS_time = "None"
                    pub_temp.vaild_flag = True
                    pub_temp.flash_state = 'POSITION'
                    pub_temp.north_meter = self.N_POSITION+gps_length_y * math.sin(self.YAW_POSITION) + gps_length_x * math.cos(self.YAW_POSITION)
                    pub_temp.east_meter = self.E_POSITION - gps_length_y * math.cos(self.YAW_POSITION) + gps_length_x * math.sin(self.YAW_POSITION)
                    pub_temp.yaw_rad = self.YAW_POSITION
                    pub.publish(pub_temp)

                if temp == 'Yaw_Data_Valid_Fault':
                    pub_temp = rtkGPSmessage()
                    pub_temp.ROS_time = rospy.get_rostime()
                    pub_temp.GPS_time = "None"
                    pub_temp.vaild_flag = False
                    pub_temp.flash_state = 'Yaw'
                    pub_temp.north_meter = self.N_POSITION
                    pub_temp.east_meter = self.E_POSITION
                    pub_temp.yaw_rad = self.YAW_POSITION
                    pub.publish(pub_temp)

                #if temp =='FALSE':
                    #print "rtkGPS data False " +str(self.temp)
                if temp == 'Data_Valid_Fault':
                    pub_temp = rtkGPSmessage()
                    pub_temp.ROS_time = rospy.get_rostime()
                    pub_temp.GPS_time = "None"
                    pub_temp.vaild_flag = False
                    pub_temp.flash_state = 'POSITION'
                    pub_temp.north_meter = self.N_POSITION+gps_length_y * math.sin(self.YAW_POSITION) + gps_length_x * math.cos(self.YAW_POSITION)
                    pub_temp.east_meter = self.E_POSITION - gps_length_y * math.cos(self.YAW_POSITION) + gps_length_x * math.sin(self.YAW_POSITION)
                    pub_temp.yaw_rad = self.YAW_POSITION
                    pub.publish(pub_temp)
                
            #print "no data"
            rate.sleep()

    def processData(self):
        if self.strlist[0] == '$PTNL':
            if len(self.strlist) == 13:
                if self.strlist[8] == '3':
                    self.Data_Valid_Flag = 1
                else:
                    self.Data_Valid_Flag = 0
                    return 'Data_Valid_Fault'
            else:
                self.Data_Valid_Flag = 0
                return 'Data_Valid_Fault'
        if (self.strlist[0] == '$PSAT') and (len(self.strlist) == 7) and self.Data_Valid_Flag == 1:
            #print strlist[6]
            #print strlist[6][0]
            if self.strlist[6][0] != 'N':
                return 'Yaw_Data_Valid_Fault'            
            try:
                self.YAW_POSITION = float(self.strlist[3]) / 180.0 * math.pi
            except:
                #print "SERIAL ERROR! rtkGPS Data: "+ str(self.strlist)
                return 'FALSE'
            else:
                return 'YAW'
        if (self.strlist[0] == '$PTNL') and (len(self.strlist) == 13) and self.Data_Valid_Flag == 1:
            try:
                self.E_POSITION = float(self.strlist[6])
                self.N_POSITION = float(self.strlist[4])
            except:
               # print "SERIAL ERROR! rtkGPS Data: "+ str(self.strlist)
                return 'FALSE'
            else:
                return 'POSITION'
        return 'FALSE'
            
if __name__ == "__main__":
    body = readGPS()
    try :
        body.readData()
    except:
        body.serial.closeSerial()
        rospy.logwarn("rtkGPS serial "+body.Serial_name+" closed")
        #rospy.loginfo("rtkGPS closed")
        #rospy. logerr("rtkGPS start Fail!")