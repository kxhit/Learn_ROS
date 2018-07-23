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
#include <rosbag/view.h>
#include <tcp2ros/readDataAll.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
 
int i=0;
int start=1;
int num=0;
int stop_count=0;
bool stop_flag=false;
bool pause_flag=false;
ros::Publisher* play_pub_;
ros::Publisher* cmd_vel_pub_;
ros::Time last_t;

using namespace std;


void callback1(const tcp2ros::readDataAll::ConstPtr& msg)
{
    if(msg->is_start_camera==2&&msg->next_target_num!=0)
    {
        start=1;
        num=msg->next_target_num;
    }
}

void callback2(const sensor_msgs::ChannelFloat32::ConstPtr& msg)
{
    if(msg->name == "stop_flag")
    {
       if(msg->values[0] == 0.0)
       {
           stop_count = stop_count+1;
           if(stop_count>50)
           {
               stop_flag = false;
               stop_count = 51;
            }
       }
       else
       {
           stop_flag = true;
           stop_count = 0;
       }
      if(stop_flag &&pause_flag==false)
      {
          geometry_msgs::Twist cmd;
          cmd.linear.x = 0;
          cmd.angular.z = 0;              
          cmd_vel_pub_->publish(cmd);
          pause_flag=true;
      }
      else if(stop_flag==false && pause_flag==true)
      {
          pause_flag=false;
      }
    }
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "teleopJoy");
    ros::NodeHandle ph;
    ros::Subscriber tcpData_sub = ph.subscribe<tcp2ros::readDataAll>("/tcpData",1, &callback1);
    ros::Subscriber laserData_sub = ph.subscribe<sensor_msgs::ChannelFloat32>("/laser_target_position",1, &callback2);
    ros::Publisher play_pub = ph.advertise<sensor_msgs::ChannelFloat32>("/play_msg",1, true);
    ros::Publisher cmd_vel_pub = ph.advertise<geometry_msgs::Twist>("/cmd_vel",1, true);
    play_pub_=&play_pub;
    cmd_vel_pub_=&cmd_vel_pub;
    while(ros::ok())
    {
        if(start==1)
        {
            rosbag::Bag bag;
            stringstream ss;
            ss<<num;
            bag.open("/home/csc105/test"+ss.str()+".bag", rosbag::bagmode::Read);
            std::vector<std::string> topics;
            topics.push_back(std::string("/cmd_vel"));
            rosbag::View view(bag, rosbag::TopicQuery(topics));

            sensor_msgs::ChannelFloat32 play_data;
            play_data.name = "play_msg";
            play_data.values.push_back(1.0);
            play_data.values.push_back(num);
            play_pub_->publish(play_data);
            

            foreach(rosbag::MessageInstance const m, view)
            {
                  geometry_msgs::Twist::ConstPtr s = m.instantiate<geometry_msgs::Twist>();
                  if (s != NULL)
                  {
                        if(i==0)
			{
			    last_t=m.getTime();
			}
			else
			{
                            double duration_t=m.getTime().toSec()-last_t.toSec();
			    cout<<duration_t<<endl;
			    last_t=m.getTime();
			    ros::Duration(duration_t).sleep();
			}
			i++;
                  }
                  if(!stop_flag)   
                  {
                      cmd_vel_pub_->publish(*s);
                  }
                  else
                  {
                      while(stop_flag)
                      {

                      }
                  }
            }
            bag.close();

            sensor_msgs::ChannelFloat32 play_data1;
            play_data1.name = "play_msg";
            play_data1.values.push_back(0.0);
            play_pub_->publish(play_data1);
            num=0;
            start=0;
        }
    }
    return 0;
}
