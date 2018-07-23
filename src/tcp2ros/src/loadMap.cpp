#include "loadMap.h"

std::vector<std::string> splitString(const std::string &str,
	const char delimiter)
{
	std::vector<std::string> splited;
	std::string s(str);
	size_t pos;

	while ((pos = s.find(delimiter)) != std::string::npos) {
		std::string sec = s.substr(0, pos);

		if (!sec.empty()) {
			splited.push_back(s.substr(0, pos));
		}

		s = s.substr(pos + 1);
	}

	splited.push_back(s);

	return splited;
}

//读取记录的有gps位姿
bool load_gps(string gps_file,vector<Pose>& pose_list)
{
	ifstream inf(gps_file);
	string s;
	int i = 0;
	while (getline(inf, s))      //getline(inf,s)是逐行读取inf中的文件信息
	{
		vector<string> pose_s = splitString(s, ' ');
		Pose p;
		stringstream ss1, ss2, ss3;
		ss1 << pose_s[0];
		ss2 << pose_s[1];
		ss3 << pose_s[2];
		ss1 >> p.x;
		ss2 >> p.y;
		ss3 >> p.yaw;

		pose_list.push_back(p);
	}
}

//读取记录的有gps和车体位姿
bool load_gps_base(string gps_file, vector<Pose>& pose_list, vector<Pose>& base_list)
{
	ifstream inf(gps_file);
	string s;
	int i = 0;
	while (getline(inf, s))      //getline(inf,s)是逐行读取inf中的文件信息
	{
		vector<string> pose_s = splitString(s, ' ');
		Pose p,b_p;
		stringstream ss1, ss2, ss3,ss4,ss5,ss6;
		ss1 << pose_s[0];
		ss2 << pose_s[1];
		ss3 << pose_s[2];
		ss1 >> p.x;
		ss2 >> p.y;
		ss3 >> p.yaw;
		pose_list.push_back(p);

		if (pose_s.size() > 3)
		{
			ss4 << pose_s[3];
			ss5 << pose_s[4];
			ss6 << pose_s[5];
			ss4 >> b_p.x;
			ss5 >> b_p.y;
			ss6 >> b_p.yaw;

			base_list.push_back(b_p);
		}
	}
}

//读取地图参数文件yaml
bool load_set(string map_set_file, MapMetaData& mapMetaData)
{
    ifstream conf_file(map_set_file);
    if (!conf_file.is_open())
    {
        cout << "fail to load " << map_set_file << endl;
        return false;
    }
    string line;
    vector<int> data;
    while (conf_file.good())
    {
        getline(conf_file, line);
        istringstream line_s(line);
        string field;
        line_s >> field;
        if (field.compare("resolution:")==0)
        {
            line_s>>mapMetaData.resolution;
        }
        if (field.compare("negate:")==0)
        {
            line_s>>mapMetaData.negate;
        }
        if (field.compare("free_thresh:")==0)
        {
            line_s>>mapMetaData.free_thresh;
        }
        if (field.compare("occupied_thresh:")==0)
        {
            line_s>>mapMetaData.occupied_thresh;
        }
        if (field.compare("image:")==0)
        {
            line_s>>mapMetaData.mapfname;
            if (mapMetaData.mapfname.size() == 0)
            {
                cerr<<"The image tag cannot be an empty string."<<endl;
                exit(-1);
             }
             if (mapMetaData.mapfname[0] != '/')
             {
                // dirname can modify what you pass it
                int n = map_set_file.find_last_of('/');
                string path = map_set_file.substr(0, n);
                mapMetaData.mapfname = path + '/' + mapMetaData.mapfname;
             }
        }
        if (field.compare("origin:")==0)
        {
            int start_n = line.find_first_of('[');
            int end_n = line.find_last_of(']');
            string origin_s = line.substr(start_n+1, end_n-1);
            vector<string> origin=splitString(origin_s,',');
            stringstream ss1,ss2,ss3;
            ss1<<origin[0];
            ss2<<origin[1];
            ss3<<origin[2];
            ss1>>mapMetaData.origin.x;
            ss2>>mapMetaData.origin.y;
            ss3>>mapMetaData.origin.yaw;
        }
    }
    return true;

}

//读取初始gps和车体坐标系相对应变换
bool load_init(string map_init_file, Pose& gps_pose,Pose& base_pose)
{
	ifstream inf(map_init_file);
	string s;
	int i = 0;
	while (getline(inf, s))      //getline(inf,s)是逐行读取inf中的文件信息
	{
		vector<string> pose_s=splitString(s, ' ');
		if (i == 0)
		{
			stringstream ss1, ss2, ss3;
			ss1 << pose_s[0];
			ss2 << pose_s[1];
			ss3 << pose_s[2];
			ss1 >> gps_pose.x;
			ss2 >> gps_pose.y;
			ss3 >> gps_pose.yaw;
		}
		else if (i == 1)
		{
			stringstream ss1, ss2, ss3;
			ss1 << pose_s[0];
			ss2 << pose_s[1];
			ss3 << pose_s[2];
			ss1 >> base_pose.x;
			ss2 >> base_pose.y;
			ss3 >> base_pose.yaw;
		}
		i++;
	}
}

//gps坐标系转换到车体坐标系
Pose rtkGPStoBaseLink(Pose TargetPoint,Pose BaseLinkPoint)
{
	double X_rtkGPS_Target = TargetPoint.x;
	double Y_rtkGPS_Target = TargetPoint.y;
	double YAW_rtkGPS_Target = TargetPoint.yaw;
	double X_rtkGPS_BaseLink = BaseLinkPoint.x;
	double Y_rtkGPS_BaseLink = BaseLinkPoint.y;
	double YAW_rtkGPS_BaseLink = BaseLinkPoint.yaw;

	double X_BaseLink_Target = (X_rtkGPS_Target - X_rtkGPS_BaseLink)*cos(YAW_rtkGPS_BaseLink)+ (Y_rtkGPS_Target - Y_rtkGPS_BaseLink)*sin(YAW_rtkGPS_BaseLink);

	double Y_BaseLink_Target = -(X_rtkGPS_Target - X_rtkGPS_BaseLink)*sin(YAW_rtkGPS_BaseLink)+ (Y_rtkGPS_Target - Y_rtkGPS_BaseLink)*cos(YAW_rtkGPS_BaseLink);

	double YAW_BaseLink_Target = YAW_rtkGPS_Target - YAW_rtkGPS_BaseLink;

	Y_BaseLink_Target = -Y_BaseLink_Target;
	YAW_BaseLink_Target = -YAW_BaseLink_Target;

	if(YAW_BaseLink_Target > 2 * M_PI)
	{ 
		YAW_BaseLink_Target = YAW_BaseLink_Target - 2 * M_PI;
	}
	if(YAW_BaseLink_Target < -2 * M_PI)
	{
		YAW_BaseLink_Target = YAW_BaseLink_Target + 2 * M_PI;
	}

	if(YAW_BaseLink_Target > M_PI)
	{
		YAW_BaseLink_Target = YAW_BaseLink_Target - 2 * M_PI;
	}
	if (YAW_BaseLink_Target < -M_PI)
	{
		YAW_BaseLink_Target = YAW_BaseLink_Target + 2 * M_PI;
	}
	
	Pose reault;
	reault.x = X_BaseLink_Target;
	reault.y = Y_BaseLink_Target;
	reault.yaw = YAW_BaseLink_Target;
	return reault;
}

//gps坐标系转换到车体坐标系，目标gps位姿，有对应的gps位姿，车体坐标系
Pose rtkGPStoBaseLink(Pose TargetPoint, Pose BaseLinkPoint, Pose basePose)
{
	double X_rtkGPS_Target = TargetPoint.x;
	double Y_rtkGPS_Target = TargetPoint.y;
	double YAW_rtkGPS_Target = TargetPoint.yaw;
	double X_rtkGPS_BaseLink = BaseLinkPoint.x;
	double Y_rtkGPS_BaseLink = BaseLinkPoint.y;
	double YAW_rtkGPS_BaseLink = BaseLinkPoint.yaw;

	double X_BaseLink_Target = (X_rtkGPS_Target - X_rtkGPS_BaseLink)*cos(basePose.yaw + YAW_rtkGPS_BaseLink) + (Y_rtkGPS_Target - Y_rtkGPS_BaseLink)*sin(basePose.yaw + YAW_rtkGPS_BaseLink);

	double Y_BaseLink_Target = -(X_rtkGPS_Target - X_rtkGPS_BaseLink)*sin(basePose.yaw + YAW_rtkGPS_BaseLink) + (Y_rtkGPS_Target - Y_rtkGPS_BaseLink)*cos(basePose.yaw + YAW_rtkGPS_BaseLink);

	double YAW_BaseLink_Target = YAW_rtkGPS_Target - YAW_rtkGPS_BaseLink;

	Y_BaseLink_Target = -Y_BaseLink_Target;
	YAW_BaseLink_Target = -YAW_BaseLink_Target;

	if (YAW_BaseLink_Target > 2 * M_PI)
	{
		YAW_BaseLink_Target = YAW_BaseLink_Target - 2 * M_PI;
	}
	if (YAW_BaseLink_Target < -2 * M_PI)
	{
		YAW_BaseLink_Target = YAW_BaseLink_Target + 2 * M_PI;
	}

	//if (YAW_BaseLink_Target > M_PI)
	//{
	//	YAW_BaseLink_Target = YAW_BaseLink_Target - 2 * M_PI;
	//}
	//if (YAW_BaseLink_Target < -M_PI)
	//{
	//	YAW_BaseLink_Target = YAW_BaseLink_Target + 2 * M_PI;
	//}

	Pose reault;
	reault.x = X_BaseLink_Target + basePose.x;
	reault.y = Y_BaseLink_Target + basePose.y;
	reault.yaw = YAW_BaseLink_Target + basePose.yaw;
	return reault;
}

//车体坐标系转换到gps坐标系
Pose BaseToGPS(Pose TargetPoint_BaseLink,Pose BaseLinkPoint)
{
	double X_BaseLink_Target_2 = TargetPoint_BaseLink.x;
	double Y_BaseLink_Target_2 = TargetPoint_BaseLink.y;
	double YAW_BaseLink_Target_2 = TargetPoint_BaseLink.yaw;
	double X_rtkGPS_BaseLink_2 = BaseLinkPoint.x;
	double Y_rtkGPS_BaseLink_2 = BaseLinkPoint.y;
	double YAW_rtkGPS_BaseLink_2 = BaseLinkPoint.yaw;

	double X_rtkGPS_Target_2 = X_rtkGPS_BaseLink_2 + X_BaseLink_Target_2*cos(YAW_rtkGPS_BaseLink_2) + Y_BaseLink_Target_2*sin(YAW_rtkGPS_BaseLink_2);
	double Y_rtkGPS_Target_2 = Y_rtkGPS_BaseLink_2 + X_BaseLink_Target_2*sin(YAW_rtkGPS_BaseLink_2) - Y_BaseLink_Target_2*cos(YAW_rtkGPS_BaseLink_2);

	double YAW_rtkGPS_Target_2 = YAW_rtkGPS_BaseLink_2 - YAW_BaseLink_Target_2;

	Pose reault;
	reault.x = X_rtkGPS_Target_2;
	reault.y = Y_rtkGPS_Target_2;
	reault.yaw = YAW_rtkGPS_Target_2;
	return reault;
}

//车体坐标系转换到地图坐标系上
Pose baseToMap(Pose origin,Pose pose, MapMetaData mapMetaData)
{
	Pose base_p;
	base_p.x = (-origin.x + pose.x) / mapMetaData.resolution;
	base_p.y = mapMetaData.height + ((origin.y - pose.y) / mapMetaData.resolution);
	base_p.yaw = pose.yaw;

	return base_p;
}

//读取地图，地图设置yaml文件，地图参数
bool load_map(string map_set_file,MapData& map)
{
	string file_init,gps_file;
	MapMetaData mapMetaData;
	//读取yaml设置文件
	load_set(map_set_file, mapMetaData);
	int n = map_set_file.find_last_of('\\');
	string path = map_set_file.substr(0, n);
	file_init = path + '\\' + "map_init.txt";
	gps_file = path + '\\' + "gps.txt";

	Pose gps_pose, base_pose;
	//读取gps和base的相对应变换的文件
	load_init(file_init, gps_pose,base_pose);

	vector<Pose> gps_pose_list,base_pose_list;
	//读取gps
	//(gps_file, gps_pose_list);
	load_gps_base(gps_file, gps_pose_list, base_pose_list);

	//读取地图图片
	cv::Mat image = cv::imread(mapMetaData.mapfname);

	//蛇者地图原始长宽
	mapMetaData.width = image.cols;
	mapMetaData.height = image.rows;

	Pose pose_zero;
	pose_zero.x = 0;
	pose_zero.y = 0;
	pose_zero.yaw = 0;

	//将车体坐标系上的初始位姿转换到地图坐标系上
	Pose map_init_pose=baseToMap(mapMetaData.origin, pose_zero, mapMetaData);
	//画到地图上
	cv::circle(image, cv::Point(map_init_pose.x, map_init_pose.y), 8, cv::Scalar(255, 0, 0), 8);

	//遍历每一个gps位姿
	for (int i = 0; i < gps_pose_list.size(); i++)
	{
		//将gps位姿转换到车体坐标系上
		Pose base= rtkGPStoBaseLink(gps_pose_list[i], gps_pose, base_pose);

		//将车体坐标系转换到地图坐标系上
		Pose map_pose = baseToMap(mapMetaData.origin, base, mapMetaData);

		//在地图中画出
		cv::circle(image, cv::Point(map_pose.x, map_pose.y), 3, cv::Scalar(0, 0, 255), 3);
	}

	//将每一个gps相对应的激光定位(车体坐标系)画出
	for (int i = 0; i < base_pose_list.size(); i++)
	{
		//将车体坐标系转换到地图坐标系上
		Pose map_pose = baseToMap(mapMetaData.origin, base_pose_list[i], mapMetaData);
		//在地图中画出
		cv::circle(image, cv::Point(map_pose.x, map_pose.y), 3, cv::Scalar(0, 255, 0), 3);
	}

	//调整地图大小，原地图大小约有4000*4000多
	cv::resize(image, image, cv::Size(800, 800));
	//显示
	cv::imshow("map", image);
	cv::waitKey(0);
}
