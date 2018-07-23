#include <tcp2ros/carDriver.h>
#include <math.h>
double x=0.0,y=0.0,th=0.0;

double check_angle(double angle)
{
    if (angle>pai)
        angle=-2*pai+angle;
    else if (angle<-pai) 
        angle=2*pai+angle;
    return angle;
}
CarDriver::CarDriver(ros::NodeHandle &nh,double a,double b,double p): nh_(nh),a(a),b(b),pulse_cycle(p)
{
	update_freq_ = 20;
	memset(cmd_, 0, sizeof cmd_);

	init=false;

	pose_x = 0.0;
	pose_y = 0.0;
	pose_th = 0.0;

	vx = 0.0;
	vy = 0.0;
	vth = 0.0;

	x=0.0;
	y=0.0;
	th=0.0;
	last_data[0]=0.0;
	last_data[1]=0.0;
	last_data[2]=0.0;
	last_data[3]=0.0;
	updated_ = ros::Time::now();

	cout<<"start"<<endl;
	odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
	tcp_sub=nh.subscribe("/tcpData",10,&CarDriver::readCallback,this);
	imu_sub=nh.subscribe("/imu/data",10,&CarDriver::imuCallback,this);
	get_imu=false;
}
void CarDriver::imuCallback(const sensor_msgs::Imu& imu_)
{
	get_imu=true;
	geometry_msgs::Quaternion q;
    	q.w=imu_.orientation.w;
    	q.x=imu_.orientation.x;
    	q.y=imu_.orientation.y;
    	q.z=imu_.orientation.z;
    	imu_yaw=tf::getYaw(q);
}
void CarDriver::readCallback(const tcp2ros::readDataAll& data)
{
	if(!init && get_imu)
	{
	    last_data[0]=data.odom1;
	    last_data[1]=data.odom2;
	    last_data[2]=data.odom3;
	    last_data[3]=data.odom4;

	    updated_ = ros::Time::now();
            init=true;
            if(get_imu)
	    {
               init_imu_yaw=imu_yaw;
               init_imu_yaw=check_angle(init_imu_yaw);
            }
	    return;
	}
	if(init)
	{
	double period = (ros::Time::now() - updated_).toSec();
	updated_ = ros::Time::now();

	double vel1 =-(double)((double)(data.odom1-last_data[0])/(double)pulse_cycle);
	double vel2 =-(double)((double)(data.odom2-last_data[1])/(double)pulse_cycle);
	double vel3 =(double)((double)(data.odom3-last_data[2])/(double)pulse_cycle);
	double vel4 =(double)((double)(data.odom4-last_data[3])/(double)pulse_cycle);

	last_data[0]=(double)data.odom1;
	last_data[1]=(double)data.odom2;
	last_data[2]=(double)data.odom3;
	last_data[3]=(double)data.odom4;

	vel_[0] = vel1;
	vel_[1] = vel3;
	vel_[2] = vel4;
	vel_[3] = vel2;

	carVelConvert();

	if(get_imu)
	{
	    th=imu_yaw-init_imu_yaw;
	    th=check_angle(th);
	    vth=(th-last_imu_th)/period;
	}

	double delta_x = (vx * cos(th) - vy * sin(th))*period;
	double delta_y = (vx * sin(th) + vy * cos(th))*period;
	double delta_th = vth*period;
	
	pose_x += (double)delta_x;
	pose_y += (double)delta_y;
	pose_th += (double)delta_th;

	x+=(double)delta_x;
	y+=(double)delta_y;
	th = th + (double)delta_th;
	//cout<<vth<<" "<<th<<endl;

	if(get_imu)
	{
	    th=imu_yaw-init_imu_yaw;
	    th=check_angle(th);
	    vth=(th-last_imu_th)/period;
	    last_imu_th=th;
	}
	th=check_angle(th);
        //cout<<th<<" "<<init_imu_yaw<<endl;
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = updated_;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	//odom_broadcaster.sendTransform(odom_trans);

	nav_msgs::Odometry odom;
	odom.header.stamp = updated_;
	odom.header.frame_id = "odom";

	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = vy;
	odom.twist.twist.angular.z = vth;

	odom_pub.publish(odom);
	}
}
void CarDriver::carVelConvert()
{
	vx=(vel_[0]+vel_[1]+vel_[2]+vel_[3])/4.0;
	//vy=-(vel_[1]+vel_[3]-vel_[0]-vel_[2])/4.0;
	vy=0.0;
	vth=((vel_[0]-vel_[2])/2.0/(a+b)+(vel_[3]-vel_[1])/2.0/(a+b))/2.0;
}
