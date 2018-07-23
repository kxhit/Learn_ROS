#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <tcp2ros/rtkGPSmessage.h>

using namespace std;

static std::string PARENT_FRAME="map";
static std::string CHILD_FRAME="odom";
static tf::TransformListener *tf_listener;

bool first=true;

string fileName="/home/exbot/catkin_ws/src/tcp2ros/NodeMap/map/map_init.txt";
string fileName1="/home/exbot/catkin_ws/src/tcp2ros/NodeMap/map/gps.txt";

ofstream outfile1;
void gps_callback(const tcp2ros::rtkGPSmessage::ConstPtr& msg)
{
    tcp2ros::rtkGPSmessage gps_data=*msg;
    if(gps_data.yaw_rad!=0.0)
    {

        tf::StampedTransform transform;
        try
        {
          tf_listener->waitForTransform(PARENT_FRAME, CHILD_FRAME, msg->ROS_time, ros::Duration(1));
          tf_listener->lookupTransform(PARENT_FRAME, CHILD_FRAME, msg->ROS_time, transform);
        }
        catch (tf::TransformException ex)
        {
          std::cout << "Transform not found" << std::endl;
          return;
        }

        double yaw=tf::getYaw(transform.getRotation());
        outfile1<<setiosflags(ios::fixed)<<setprecision(4)<<gps_data.north_meter<<" "<<gps_data.east_meter<<" "<<gps_data.yaw_rad<<" "<<transform.getOrigin().x()<<" "<<transform.getOrigin().y()<<" "<<yaw<<setiosflags(ios::fixed)<<endl;
        cout<<setiosflags(ios::fixed)<<setprecision(4)<<gps_data.north_meter<<" "<<gps_data.east_meter<<" "<<gps_data.yaw_rad<<setiosflags(ios::fixed)<<endl;

        if(first)
        {
            ofstream outfile;
            outfile.open(fileName.c_str());

            outfile<<setiosflags(ios::fixed)<<setprecision(4)<<gps_data.north_meter<<" "<<gps_data.east_meter<<" "<<gps_data.yaw_rad<<setiosflags(ios::fixed)<<endl;
            outfile<<setiosflags(ios::fixed)<<setprecision(4)<<transform.getOrigin().x()<<" "<<transform.getOrigin().y()<<" "<<yaw<<setiosflags(ios::fixed)<<endl;

            cout<<"mapping init record: "<<setiosflags(ios::fixed)<<setprecision(4)<<gps_data.north_meter<<" "<<gps_data.east_meter<<" "<<gps_data.yaw_rad<<setiosflags(ios::fixed)<<endl;
                    outfile.close();
            first=false;
        }
    }
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "gmapping_init_record");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    outfile1.open(fileName1.c_str());

    private_nh.getParam("parent_frame", PARENT_FRAME);
    private_nh.getParam("child_frame", CHILD_FRAME);

    ros::Subscriber gps_sub = nh.subscribe("/rtkGPS_filter", 10, gps_callback);
    tf_listener = new tf::TransformListener();
    ros::spin();
    return 0;
}
