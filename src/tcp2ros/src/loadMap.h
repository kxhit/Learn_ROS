#ifndef LOADMAP_H_
#define LOADMAP_H_

#include <cstring>
#include <stdexcept>

#include <stdlib.h>
#include <stdio.h>

#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

//#include <yaml-cpp/yaml.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
//using namespace cv;

#define M_PI        3.14159265358979323846

struct Pose
{
	double x;
	double y;
	double yaw;
};

//地图参数
struct MapMetaData
{
	string mapfname;		//地图图片文件地址
	double resolution;		//分辨率
	int negate;				
	double occupied_thresh;	//栅格阈值
	double free_thresh;		//空闲阈值
	int width;				//地图宽度
	int height;				//地图长度
	Pose origin;			//地图初始点位姿(单位m)
};

//地图信息
struct MapData
{
	MapMetaData info;
	vector<int> data;
};

std::vector<std::string> splitString(const std::string &str,const char delimiter);
bool load_gps(string gps_file, vector<Pose>& pose_list);
bool load_gps_base(string gps_file, vector<Pose>& pose_list, vector<Pose>& base_list);
bool load_set(string map_set_file, MapMetaData& mapMetaData);
bool load_init(string map_init_file, Pose& gps_pose, Pose& base_pose);
Pose rtkGPStoBaseLink(Pose TargetPoint, Pose BaseLinkPoint);
Pose rtkGPStoBaseLink(Pose TargetPoint, Pose BaseLinkPoint, Pose basePose);
Pose BaseToGPS(Pose TargetPoint_BaseLink, Pose BaseLinkPoint);
Pose baseToMap(Pose origin, Pose pose, MapMetaData mapMetaData);
bool load_map(string map_set_file, MapData& map);

#endif
