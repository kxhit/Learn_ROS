#!/usr/bin/env python
import math

class Map_Point:
    def __init__(self):
        self.pointlist_x = []
        self.pointlist_y = []
        self.pointlist_yaw = []
        self.id = []


    def add_point(self, point_x, point_y,point_yaw,id):
        self.pointlist_x.append(point_x)
        self.pointlist_y.append(point_y)
        self.pointlist_yaw.append(point_yaw)
        self.id.append(id)

    def get_point(self,num):
        return [self.pointlist_x[num], self.pointlist_y[num],self.pointlist_yaw[num]]

    def get_id(self,num):
        return self.id[num]

    def update_point(self,num,point_x,point_y,point_yaw):
        self.pointlist_x[num] = point_x
        self.pointlist_y[num] = point_y
        self.pointlist_yaw[num] = point_yaw

    def select_MapPoint(self,num):
        for i in range(len(self.pointlist_x)):
            if self.get_id(i) == num:
                return [self.pointlist_x[i], self.pointlist_y[i],self.pointlist_yaw[i],num]





if __name__ == "__main__":
    body = Map_Point()
    body.add_point(0,0,0,0)
    body.add_point(0,1,0,1)
    body.add_point(1,1,0,2)
    body.add_point(1,0,0,3)
    #body.add_point(1, 2, 3, 4)
    #temp = body.select_MapPoint(4)
    #body.add_point(temp[0], temp[1], temp[2], temp[3])
    #print body.select_MapPoint(4)
    #print "_______"
    #print body.get_point(4)
    print body.get_point(0)
    print body.get_point(1)
    print body.get_point(2)
    print body.get_point(3)

    print "length"
    print len(body.pointlist_x)
    for ii in range(len(body.pointlist_x)):
        print " "
    print ii
    body.update_point(1,2,2,2)
    print body.get_point(1)
    print body.get_id(1)

    #print body.get_point_number()
