#!/usr/bin/env python
import math

gps_length_x=0.420#-- the vertical distance between the center point of car with gps module
gps_length_y=0.340 #-- the horizontal distance between the center point of car with gps module

class TF:
    def __init__(self):
        self.a=0

#GPS坐标转换为BaseLink下，坐标转换方面的知识请参考 《机器人建模和控制》
    def rtkGPStoBaseLink(self,TargetPoint, BaseLinkPoint):
        X_rtkGPS_Target = TargetPoint[0]
        Y_rtkGPS_Target = TargetPoint[1]
        YAW_rtkGPS_Target = TargetPoint[2]
        X_rtkGPS_BaseLink = BaseLinkPoint[0]
        Y_rtkGPS_BaseLink = BaseLinkPoint[1]
        YAW_rtkGPS_BaseLink = BaseLinkPoint[2]
        #cal the Target point in baselink coordinate system
        X_BaseLink_Target=(X_rtkGPS_Target - X_rtkGPS_BaseLink)*math.cos(YAW_rtkGPS_BaseLink)\
            + (Y_rtkGPS_Target - Y_rtkGPS_BaseLink)*math.sin(YAW_rtkGPS_BaseLink)

        Y_BaseLink_Target=-(X_rtkGPS_Target - X_rtkGPS_BaseLink)*math.sin(YAW_rtkGPS_BaseLink)\
            + (Y_rtkGPS_Target - Y_rtkGPS_BaseLink)*math.cos(YAW_rtkGPS_BaseLink)

        YAW_BaseLink_Target = YAW_rtkGPS_Target - YAW_rtkGPS_BaseLink

        Y_BaseLink_Target = -Y_BaseLink_Target
        YAW_BaseLink_Target = -YAW_BaseLink_Target
        
        if YAW_BaseLink_Target > 2*math.pi:
            YAW_BaseLink_Target = YAW_BaseLink_Target - 2*math.pi
        if YAW_BaseLink_Target < -2*math.pi:
            YAW_BaseLink_Target = YAW_BaseLink_Target + 2*math.pi
        if YAW_BaseLink_Target > math.pi:
            YAW_BaseLink_Target = YAW_BaseLink_Target - 2*math.pi
        if YAW_BaseLink_Target < -math.pi:
            YAW_BaseLink_Target = YAW_BaseLink_Target + 2*math.pi

        return [X_BaseLink_Target, Y_BaseLink_Target, YAW_BaseLink_Target]

#BaseLink坐标转换为GPS坐标
    def BaseLinktortkGPS(self,TargetPoint_BaseLink,BaseLinkPoint): #-- transform the baselink coordinate of the target to the gps coordinate
        X_BaseLink_Target_2 = TargetPoint_BaseLink[1]
        Y_BaseLink_Target_2 = TargetPoint_BaseLink[0]
        YAW_BaseLink_Target_2 = TargetPoint_BaseLink[2]
        X_rtkGPS_BaseLink_2 = BaseLinkPoint[0]
        Y_rtkGPS_BaseLink_2 = BaseLinkPoint[1]
        YAW_rtkGPS_BaseLink_2 = BaseLinkPoint[2]

#        X_rtkGPS_Target = (X_BaseLink_Target/math.sin(YAW_rtkGPS_BaseLink)*math.cos(YAW_rtkGPS_BaseLink) - Y_BaseLink_Target)/((math.cos(YAW_rtkGPS_BaseLink))/(math.sin(YAW_rtkGPS_BaseLink))*(math.cos(YAW_rtkGPS_BaseLink)) + math.sin(YAW_rtkGPS_BaseLink)) + X_rtkGPS_BaseLink
#        Y_rtkGPS_Target = (X_BaseLink_Target/(math.cos(YAW_rtkGPS_BaseLink))*(math.sin(YAW_rtkGPS_BaseLink)) + Y_BaseLink_Target)/((math.sin(YAW_rtkGPS_BaseLink))/math.cos(YAW_rtkGPS_BaseLink)*math.sin(YAW_rtkGPS_BaseLink) + math.cos(YAW_rtkGPS_BaseLink)) + Y_rtkGPS_BaseLink
#        YAW_rtkGPS_Target = YAW_BaseLink_Target + YAW_rtkGPS_BaseLink

        X_rtkGPS_Target_2 = X_rtkGPS_BaseLink_2+gps_length_x*math.cos(YAW_rtkGPS_BaseLink_2)+X_BaseLink_Target_2*math.cos(YAW_rtkGPS_BaseLink_2)+Y_BaseLink_Target_2*math.sin(YAW_rtkGPS_BaseLink_2)

        Y_rtkGPS_Target_2 = Y_rtkGPS_BaseLink_2+gps_length_x*math.sin(YAW_rtkGPS_BaseLink_2)+X_BaseLink_Target_2*math.sin(YAW_rtkGPS_BaseLink_2)-Y_BaseLink_Target_2*math.cos(YAW_rtkGPS_BaseLink_2)

        YAW_rtkGPS_Target_2 = YAW_rtkGPS_BaseLink_2 - YAW_BaseLink_Target_2

        if YAW_rtkGPS_Target_2 > 2*math.pi:
            YAW_rtkGPS_Target_2 = YAW_rtkGPS_Target_2 - 2*math.pi
        if YAW_rtkGPS_Target_2 < -2*math.pi:
            YAW_rtkGPS_Target_2 = YAW_rtkGPS_Target_2 + 2*math.pi
        if YAW_rtkGPS_Target_2 > math.pi:
            YAW_rtkGPS_Target_2 = YAW_rtkGPS_Target_2 - 2*math.pi
        if YAW_rtkGPS_Target_2 < -math.pi:
            YAW_rtkGPS_Target_2 = YAW_rtkGPS_Target_2 + 2*math.pi


        return [X_rtkGPS_Target_2,Y_rtkGPS_Target_2,YAW_rtkGPS_Target_2]

if __name__ == "__main__":
    body = TF()
    TargetPoint = [10, 10, 1/4*math.pi]
    BaseLinkPoint = [5, 5, 1/4*math.pi]
    temp1 = body.rtkGPStoBaseLink(TargetPoint, BaseLinkPoint)
    temp2 = body.BaseLinktortkGPS(temp1,BaseLinkPoint)
    print temp2
    print body.rtkGPStoBaseLink(TargetPoint, BaseLinkPoint)
