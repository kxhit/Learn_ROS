#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
using namespace std;


class TeleopTurtle
{
public:
  TeleopTurtle();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};


TeleopTurtle::TeleopTurtle():
  linear_(1),
  angular_(2)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 100, &TeleopTurtle::joyCallback, this);

}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
//   twist.angular.z = a_scale_*joy->axes[angular_];
//   twist.linear.x = l_scale_*joy->axes[linear_];
// 增加了加速按键 左功能键 joy->axes[2] 不按为1.0 按到底是-1.0 1.0~-1.0 
  twist.angular.z = (3-joy->axes[a_scale_])*joy->axes[angular_];
  twist.linear.x = (3-joy->axes[l_scale_])*joy->axes[linear_];
//   cout<<"a_scale "<<a_scale_<<endl;
//   cout<<"l_scale "<<l_scale_<<endl;
  ROS_INFO_STREAM("a_scale " <<a_scale_);
  ROS_INFO_STREAM("l_scale " <<l_scale_);
  ROS_INFO_STREAM("axis_linear " <<linear_);
  ROS_INFO_STREAM("axis_angular " <<angular_);
  vel_pub_.publish(twist);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

  ros::spin();
}