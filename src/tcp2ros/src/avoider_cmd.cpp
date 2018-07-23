#include <csignal>
#include <vector>
#include <iostream>
#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include "tcp2ros/cmd.h"

#include "obs_avoider.h"

using namespace std;

ros::Publisher* _cmd_vel_pub;
vector<long> laser_dist_points;  //单位为mm
int interest_oa_radius=1500;         //避障范围1500m
int strategy=1;                       //0代表虚拟力法，1代表逃避法
double max_speed=1.0;                 //最大速度
double max_oa_angle_speed=1.0/3.0;     //最大角速度
double oaWeight=0.8;                  //避障权重
obs_avoider obs;

double fuse_cmd_oa( double oa_angle_rad, double weight_oa, double g2g_angle_rad )
{
    if(oa_angle_rad!=0)
    	return ( weight_oa * oa_angle_rad + ( 1 - weight_oa ) * g2g_angle_rad );

    else
	return g2g_angle_rad;
}
double varied_speed( const double &control_angle_rad, const double &max_speed )
{
    const double P = 90.f;
    return max_speed / sqrt( 1.0 + P * control_angle_rad * control_angle_rad );
}

double varied_angle_speed(const double &angle_speed)
{
    return angle_speed*max_oa_angle_speed/2.5;
}

void cmdCallBack(const tcp2ros::cmd::ConstPtr& cmd)
{
    double cmd_angle_rad=-cmd->Turn;
    double cmd_velocity=cmd->Velocity;
    int mode=cmd->Mode;
   

    if(mode==0)
    {
        double oa_angle_rad=obs_avoider::laser_obstalce_avoid(laser_dist_points,interest_oa_radius,strategy);     
	
	double varied_oa_angle_rad=varied_angle_speed(oa_angle_rad);
        double fused_angle_rad = fuse_cmd_oa(varied_oa_angle_rad, oaWeight, cmd_angle_rad);
        double fused_speed = varied_speed(fused_angle_rad, max_speed);
        double speed=fused_speed*cmd_velocity;

	std::cout<<varied_oa_angle_rad<<" "<<fused_angle_rad<<" "<<fused_speed<<" "<<speed<<std::endl;

        geometry_msgs::Twist vel;
        vel.angular.z = fused_angle_rad;
        vel.linear.x = speed;
        _cmd_vel_pub->publish(vel);
    }
    else if(mode==2)
    {
        geometry_msgs::Twist vel;
        vel.angular.z = cmd_angle_rad;
        vel.linear.x = cmd_velocity;
        _cmd_vel_pub->publish(vel);
    }
}

void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    vector<float> ranges(scan->ranges);
    if(ranges.size()>0)
    {
        laser_dist_points.resize(ranges.size());
        for(int i=0;i<ranges.size();i++)
        {
            laser_dist_points[i]=(long)(ranges[i]*1000);
        }
    }
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "avoider_cmd");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param("interest_oa_radius", interest_oa_radius, 1500);	ROS_INFO("interest_oa_radius: %d", interest_oa_radius);
    private_nh.param("strategy", strategy, 1);	ROS_INFO("strategy: %d", strategy);
    private_nh.param("max_speed", max_speed, 1.0);	ROS_INFO("max_speed: %f", max_speed);
    private_nh.param("max_oa_angle_speed", max_oa_angle_speed, 1.0/3.0);	ROS_INFO("max_oa_angle_speed: %f", max_oa_angle_speed);
    private_nh.param("oaWeight", oaWeight, 0.8);	ROS_INFO("oaWeight: %f", oaWeight);

    ros::Subscriber cmd_sub = nh.subscribe<tcp2ros::cmd>("cmd",1, &cmdCallBack);
    ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/base_scan", 10, &scanCallBack);;

    ros::Publisher cmd_vel_pub =nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
    _cmd_vel_pub=&cmd_vel_pub;

    ros::spin();

    return 0;
}
