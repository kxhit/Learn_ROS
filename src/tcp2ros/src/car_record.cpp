#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <stack>
#include <vector>
#include <math.h>
#include <fstream>
#include <sstream>
#include <string>
#include <tcp2ros/readDataAll.h>
#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<sensor_msgs/Joy.h>
#include<iostream>

rosbag::Bag bag;

int num=0;
int start=0;
int i=0;
geometry_msgs::Twist cmd_data;

using namespace std;

void callback1(const tcp2ros::readDataAll::ConstPtr& msg)
{
    if(msg->is_start_camera==2&&msg->next_target_num!=0)
    {
        start=1;
        num=msg->next_target_num;
    }
    else if(msg->is_start_camera==0&&msg->next_target_num==0)
    {
        start=0;
    }
}

void callback2(const geometry_msgs::Twist::ConstPtr& msg)
{
    if(start==1)
    {
            if(i==0)
            {
                cout<<"start num record"<<endl;
                stringstream ss;
                ss<<num;
                bag.open("/home/exbot/test"+ss.str()+".bag", rosbag::bagmode::Write);
                bag.write("/cmd_vel", ros::Time::now(), *msg);
            }
            else
            {
                cout<<"end num record"<<endl;
                cout<<msg->linear.x<<" "<<msg->angular.z<<endl;
                bag.write("/cmd_vel", ros::Time::now(), *msg);
            }
            i=i+1;
    }
    else
    {
           if(i!=0)
           {
                geometry_msgs::Twist cmd_data;
                cmd_data.linear.x=0.0;
                cmd_data.linear.y=0.0;
                cmd_data.linear.z=0.0;
                cmd_data.angular.x=0.0;
                cmd_data.angular.y=0.0;
                cmd_data.angular.z=0.0;
                bag.write("/cmd_vel", ros::Time::now(), cmd_data);
                bag.close();
                i=0;
           }
    }
}
int main(int argc,char** argv)
{
    ros::init(argc, argv, "car_record");
    ros::NodeHandle ph;
    //ros::Subscriber tcpData_sub = ph.subscribe<tcp2ros::readDataAll>("/tcpData",1, &callback1);
    ros::Subscriber cmd_sub = ph.subscribe<geometry_msgs::Twist>("/cmd_vel",1, &callback2);
    cout<<"start record"<<endl;
    while(ros::ok())
    {
      ros::spinOnce();
    }
    bag.close();
}
