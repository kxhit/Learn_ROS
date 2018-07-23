#include <tcp2ros/carDriver.h>

double a,b,p,scale;
string s;
void initParam(ros::NodeHandle& nh)
{
    nh.param<double>("/carDriver/a",a,0.38);
    nh.param<double>("/carDriver/b",b,0.360);
    nh.param<double>("/carDriver/p",p,300);
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "carDriver");
	ros::NodeHandle nh;
	initParam(nh);

	CarDriver car(nh,a,b,p);

	ros::Rate rate(car.getUpdateFreq());
	ros::spin();

	return 0;
}

