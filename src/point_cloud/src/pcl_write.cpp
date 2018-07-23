#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

void cloudCallback(const sensor_msgs::PointCloud2 &input)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(input, cloud);
    pcl::io::savePCDFile("test_write_PCD.pcd", cloud);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pcl_write");
    
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("pcl_output", 10, cloudCallback);
    
    ros::spin();
    
    return 0;
}