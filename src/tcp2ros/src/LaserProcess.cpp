#include<ros/ros.h>
#include <math.h>
#include<iostream>
#include<vector>
#include "sensor_msgs/LaserScan.h"
using namespace std;

/*
angle_min: -1.65806281567 //95du
angle_max: 1.65806281567 //-95du
angle_increment: 0.00582798896357 //
time_increment: 0.0
scan_time: 0.0
range_min: 0.0
range_max: 81.0
*/

void callBack(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    cout <<"min" << scan->angle_min/3.1415926*180 << "max" << scan->angle_max/3.1415926*180 <<endl;
    vector<float> intensities(scan->intensities);
    vector<float> ranges(scan->ranges);

    cout << "size intensities " << intensities.size()<< " size ranges " << ranges.size()<<endl;


}
int main(int argc, char** argv)
{
        ros::init(argc, argv, "teleopJoy");
        ros::NodeHandle n;
        ros::Publisher pub;
        ros::Subscriber sub;

        sub = n.subscribe<sensor_msgs::LaserScan>("scanlms", 10, &callBack);
        ros::spin();
}
