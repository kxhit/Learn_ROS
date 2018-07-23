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
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
#include <sensor_msgs/ChannelFloat32.h>

rosbag::Bag bag;

int num=0;
int start=0;
int i=0;
double a=0.367;
double b=0.352;
double scale = 300;
geometry_msgs::Twist cmd_data;

using namespace std;

ros::Publisher* record_pub_;

void callback1(const tcp2ros::readDataAll::ConstPtr& msg)
{
    if(msg->is_start_camera==1&&msg->next_target_num!=0)
    {
        start=1;
        num=msg->next_target_num;
    }
    else if(msg->is_start_camera==0&&msg->next_target_num==0)
    {
        start=0;
    }
    if(start==1)
    {
            int control1,control2,control3,control4;
            control1=msg->control1;
	    control2=msg->control2;
            control3=msg->control3;
            control4=msg->control4;

	    geometry_msgs::Twist cmd_data;
            cmd_data.linear.x=(double)(control3+control4-control2-control1)/4.0/scale;
            cmd_data.linear.y=0.0;
            cmd_data.linear.z=0.0;
            cmd_data.angular.x=0.0;
            cmd_data.angular.y=0.0;
            cmd_data.angular.z=-(double)(control1+control2+control3+control4)/4.0/(a+b)/scale;

            if(i==0)
            {
                cout<<"start "<<num<<" record"<<endl;
                stringstream ss;
                ss<<num;
                bag.open("/home/exbot/test"+ss.str()+".bag", rosbag::bagmode::Write);
                bag.write("/cmd_vel", ros::Time::now(), cmd_data);
                sensor_msgs::ChannelFloat32 record_msg;
                record_msg.name="record_msg";
                record_msg.values.push_back(1);
                record_msg.values.push_back(num);
                record_pub_->publish(record_msg);
            }
            else
            {
                //cout<<cmd_data.linear.x<<" "<<cmd_data.angular.z<<endl;
                bag.write("/cmd_vel", ros::Time::now(), cmd_data);
            }
            i=i+1;
    }
    else
    {
           if(i!=0)
           {
                cout<<"end "<<num<<"record"<<endl;
                geometry_msgs::Twist cmd_data;
                cmd_data.linear.x=0.0;
                cmd_data.linear.y=0.0;
                cmd_data.linear.z=0.0;
                cmd_data.angular.x=0.0;
                cmd_data.angular.y=0.0;
                cmd_data.angular.z=0.0;
                bag.write("/cmd_vel", ros::Time::now(), cmd_data);
                bag.close();
                sensor_msgs::ChannelFloat32 record_msg;
                record_msg.name="record_msg";
                record_msg.values.push_back(0);
                record_pub_->publish(record_msg);
                i=0;
           }
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
                sensor_msgs::ChannelFloat32 record_msg;
                record_msg.name="record_msg";
                record_msg.values.push_back(1);
                record_msg.values.push_back(num);
                record_pub_->publish(record_msg);
            }
            else
            {
                cout<<"end num record"<<endl;
                //cout<<msg->linear.x<<" "<<msg->angular.z<<endl;
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
                sensor_msgs::ChannelFloat32 record_msg;
                record_msg.name="record_msg";
                record_msg.values.push_back(0);
                record_pub_->publish(record_msg);
                i=0;
           }
    }
}
int main(int argc,char** argv)
{
    ros::init(argc, argv, "car_record_control");
    ros::NodeHandle ph;
    ros::Subscriber tcpData_sub = ph.subscribe<tcp2ros::readDataAll>("/tcpData",1, &callback1);
    ros::Publisher record_pub = ph.advertise<sensor_msgs::ChannelFloat32>("/record_msg",1, true);
    record_pub_=&record_pub;
    cout<<"start record"<<endl;
    while(ros::ok())
    {
      ros::spinOnce();
    }
    bag.close();
}
