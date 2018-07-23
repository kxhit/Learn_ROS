#include <stack>
#include <vector>
#include <cmath>
#include <fstream>
#include <string>
#include <map>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <tcp2ros/rtkGPSmessage.h>
#include <math.h>

#include "loadMap.h"

using namespace std;

ros::Publisher* map_pose_pub_;
ros::Publisher* start_pub_;
MapMetaData mapMetaData;
Pose gps_init_pose,base_init_pose;
cv::Mat image;

bool is_map=false;

void receivelGPS(const tcp2ros::rtkGPSmessage::ConstPtr& msg)
{


    if(msg->yaw_rad!=0.0 && is_map)
    {
        Pose p;
        p.x=msg->north_meter;
        p.y=msg->east_meter;
        p.yaw=msg->yaw_rad;

        //将gps位姿转换到车体坐标系上
        Pose base= rtkGPStoBaseLink(p, gps_init_pose, base_init_pose);

        //将车体坐标系转换到地图坐标系上
        Pose map_pose = baseToMap(mapMetaData.origin, base, mapMetaData);

        geometry_msgs::PoseStamped map_pub_pose;
        map_pub_pose.pose.position.x=map_pose.x;
        map_pub_pose.pose.position.y=map_pose.y;
        map_pub_pose.pose.position.z=0;
        tf::Quaternion q=tf::createQuaternionFromYaw(map_pose.yaw);
        map_pub_pose.pose.orientation.x=q.getX();
        map_pub_pose.pose.orientation.y=q.getY();
        map_pub_pose.pose.orientation.z=q.getZ();
        map_pub_pose.pose.orientation.w=q.getW();

        map_pub_pose.header.stamp=msg->ROS_time;
        map_pub_pose.header.frame_id="map";
        map_pose_pub_->publish(map_pub_pose);
    }

//    cv::Mat show_image;
//    image.copyTo(show_image);
//    cv::circle(show_image, cv::Point(map_pose.x, map_pose.y), 8, cv::Scalar(0, 0, 255), 8);

//    //调整地图大小，原地图大小约有4000*4000多
//    cv::resize(show_image, show_image, cv::Size(800, 800));
//    //显示
//    cv::imshow("map", show_image);
//    cv::waitKey(1);
}

int main(int argc,char** argv)
{
    ros::init (argc, argv, "pubmappose");
    ros::NodeHandle ph;

    string map_set_file="/home/exbot/catkin_ws/src/tcp2ros/NodeMap/map/mymap.yaml";
    //读取yaml设置文件
    is_map=load_set(map_set_file, mapMetaData);

    if(is_map)
    {
	int n = map_set_file.find_last_of('/');
	string path = map_set_file.substr(0, n);
	string file_init = path + '/' + "map_init.txt";
	string gps_file = path + '/' + "gps.txt";


    	//读取gps和base的相对应变换的文件
    	load_init(file_init, gps_init_pose,base_init_pose);

    	//读取地图图片
    	image = cv::imread(mapMetaData.mapfname);

    	//蛇者地图原始长宽
    	mapMetaData.width = image.cols;
    	mapMetaData.height = image.rows;
    }

    ros::Publisher map_pose_pub = ph.advertise<geometry_msgs::PoseStamped>("map_pose",1, true);
    ros::Subscriber gps_sub = ph.subscribe<tcp2ros::rtkGPSmessage>("/rtkGPS_filter",1, &receivelGPS);

    ros::Publisher start_pub = ph.advertise<sensor_msgs::ChannelFloat32>("pub_map_pose", 10,false);
    start_pub_=&start_pub;

    map_pose_pub_=&map_pose_pub;

    ros::Rate r(50);

    while(ros::ok())
    {

    
	sensor_msgs::ChannelFloat32 start_flag;
	start_flag.name="pub_map_pose";
        start_flag.values.push_back(1.0);
        start_pub_->publish(start_flag);

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
