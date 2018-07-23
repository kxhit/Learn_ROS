#include <stack>
#include <vector>
#include <math.h>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
#include <tcp2ros/rtkGPSmessage.h>
#include <tcp2ros/cmd.h>

using namespace std;

ofstream outfile;
string fileName="/home/exbot/catkin_ws/src/tcp2ros/NodeMap/Node.txt";

tcp2ros::rtkGPSmessage gps_data;

class TeleopJoy{
public:
  TeleopJoy();
private:
  void callBack(const sensor_msgs::Joy::ConstPtr& joy);
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Publisher pub1;
  ros::Subscriber sub;
  int i_velLinear, i_velAngular;
};

TeleopJoy::TeleopJoy()
{    
	n.param("axis_linear",i_velLinear,1);
	n.param("axis_angular",i_velAngular,0);
  pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
  pub1 = n.advertise<tcp2ros::cmd>("/cmd",1);
	sub = n.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::callBack, this);
}
void receivelGPS(const tcp2ros::rtkGPSmessage::ConstPtr& msg)
{
    gps_data=*msg;
}
void TeleopJoy::callBack(const sensor_msgs::Joy::ConstPtr& joy)
{
  tcp2ros::cmd vel;

  if(joy->buttons[5]!=1)
  {
    vel.Mode=2;
    vel.Turn= -joy->axes[3]/3;
    vel.Velocity = joy->axes[1];
    pub1.publish(vel);
  }
  else if(joy->buttons[5]==1)
  {
    vel.Mode=0;
    vel.Turn= -joy->axes[3]/3;
    vel.Velocity = joy->axes[1];
    pub1.publish(vel);
  }
	if(joy->buttons[3]==1)
	{
		outfile.open(fileName.c_str(),ios::app);
                
		string lineString="";
                /*stringstream nodexSS,nodeySS,nodeYawSS;
                nodexSS<<gps_data.north_meter;
                nodeySS<<gps_data.east_meter;
                nodeYawSS<<gps_data.yaw_rad;
                lineString=lineString+nodexSS.str()+" "+nodeySS.str()+" "+nodeYawSS.str()+"\n";*/
		outfile<<setiosflags(ios::fixed)<<setprecision(4)<<gps_data.north_meter<<" "<<gps_data.east_meter<<" "<<gps_data.yaw_rad<<setiosflags(ios::fixed)<<endl;
		cout<<setiosflags(ios::fixed)<<setprecision(4)<<gps_data.north_meter<<" "<<gps_data.east_meter<<" "<<gps_data.yaw_rad<<setiosflags(ios::fixed)<<endl;
                outfile.close();
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleopJoy");
	ros::NodeHandle ph;
	ros::Subscriber gps_sub = ph.subscribe<tcp2ros::rtkGPSmessage>("rtkGPS",1, &receivelGPS);
	TeleopJoy teleop_turtle;
	
	ros::spin();
}
