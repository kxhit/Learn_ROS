#include <stack>
#include <vector>
#include <cmath>
#include <fstream>
#include <string>
#include <map>
#include <boost/shared_ptr.hpp>
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
#include <tcp2ros/readDataAll.h>
#include <tcp2ros/reach.h>
#include <sensor_msgs/ChannelFloat32.h>

#include "tcp2ros/dijkstra.h"


using namespace std;

#define MAX_VERTEX_NUM 1000                           //图中最大的节点数目
#define NIL -1

struct Point
{
    double x;
    double y;
    double yaw;
};

struct Node
{
    Point p;
    int num;
};

struct NodeLine
{
    Node start;
    Node end;
    double dist;
};

map<int,vector<int> > nodeMap;
map<int,Node> nodes;
vector<NodeLine> nodeLines;

int vexnum=1;
int nodeCount=0;
int nodeLineCount=0;
int nodeTankCount=0;
boost::shared_ptr<ALGraph> GPt;
//tf接收器
tf::TransformListener* tf_listener_;

string fileName="/home/exbot/catkin_ws/src/tcp2ros/NodeMap/NodeMap.txt";

bool init=false;
bool reach=false;

int d[MAX_VERTEX_NUM];
int pi[MAX_VERTEX_NUM];
int Q[MAX_VERTEX_NUM+1];

ros::Publisher* nodes_pub_;
ros::Publisher* nodeLines_pub_;
ros::Publisher* tanks_pub_;
ros::Publisher* path_pub_;
ros::Publisher* path_pub1_;

tcp2ros::rtkGPSmessage gps_data;
//int last_target=0;
int last_target_rviz=0;

int last_tank_target=0;
int last_target=0;

int last_back_home=0;

bool showMark=false;

/**
 * @brief updateVexNum
 * 节点最大id号
 */
void updateVexNum()
{
  map<int, Node>::iterator itBegin=nodes.begin();
  vexnum = itBegin->second.num;
  for (map<int, Node>::iterator it = nodes.begin(); it != nodes.end(); it++)
  {
    if (it->second.num > vexnum)
    {
      vexnum = it->second.num;
    }
  }
  vexnum += 1;
}

/**
 * @brief getDistance
 * @param p1
 * @param p2
 * @return
 * 两点距离
 */
double getDistance(Point p1,Point p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
}
/**
 * @brief addAllLines
 * @return
 * 增加边
 */
int addAllLines()
{
    for(int i=0;i<nodeLines.size();i++)
    {
         insertArcTwo(GPt,nodeLines[i].start.num,nodeLines[i].end.num,nodeLines[i].dist);
    }
    cout<<"init Graph ok"<<endl;
    return 1;
}

/**
 * @brief readPath
 * @param fileName
 * @return
 * 读取节点图
 */
int readPath(string fileName)
{
        cout<<"readPath"<<endl;
        ifstream infile;
        infile.open(fileName.c_str());
        string line;
        int x=0;
        Node n;
        NodeLine nl;
        int node1,node2,node3;

        while(!infile.eof())
        {
            if(getline(infile,line))
            {
                istringstream iss(line);
                string col;
                int i=0;
                while(getline(iss,col,' '))
                {
                    stringstream sstr(col);
                    //第一行，三个个数
                    if(x==0)
                    {
                        if(i==0)
                          sstr>>nodeCount;
                        else if(i==1)
                          sstr>>nodeLineCount;
                        else if(i==2)
                          sstr>>nodeTankCount;
                    }
                    //读取每个节点
                    else if(x<=nodeCount)
                    {
                        if(i==0)
                        {
                          //id号读取时,即最小值为1，文本中最小为1
                          sstr>>n.num;
                          //n.num-=1;
                        }
                        else if(i==1)
                        {
                            sstr>>n.p.x;
                            //n.p.x/=10.0;
                        }
                        else if(i==2)
                        {
                            sstr>>n.p.y;
                            //n.p.y/=10.0;
                        }
                        else if(i==3)
                        {
                            sstr>>n.p.yaw;
                            //n.p.yaw/=10.0;
                            nodes.emplace(n.num,n);
                        }
                    }
                    else if(x>nodeCount && x<=(nodeCount+nodeLineCount))
                    {
                        if(i==0)
                        {
                            double d;
                            sstr>>d;
                            nl.start=nodes[d];
                        }
                        else if(i==1)
                        {
                            double d;
                            sstr>>d;
                            nl.end=nodes[d];
                            nl.dist=0;
                            nodeLines.push_back(nl);
                        }
                    }
                    else if(x>(nodeCount+nodeLineCount))
                    {
                        if(i==0)
                        {
                            sstr>>node1;
                            //node1=node1-1;
                        }
                        else if(i==1)
                        {
                            sstr>>node2;
                            //node2=node2-1;
                        }
                        else if(i==2)
                        {
                            sstr>>node3;
                            //node3=node3-1;
                            vector<int> tmp;
                            tmp.push_back(node2);
                            tmp.push_back(node3);
                            nodeMap.insert(pair<int,vector<int> >(node1,tmp));
                        }
                    }
                    i++;
                }
            }
            x++;
        }
        for(int i=0;i<nodeLines.size();i++)
        {
            nodeLines[i].dist=getDistance(nodeLines[i].start.p,nodeLines[i].end.p);
        }

        cout<<"read finish"<<endl;
        nodeCount=nodes.size();
        nodeLineCount=nodeLines.size();
        nodeTankCount=nodeMap.size();
        cout<<nodeCount<<" "<<nodeLineCount<<" "<<nodeTankCount<<endl;

        //更新最大node的id;
        updateVexNum();
        //初始化整个图
        initALGraph(GPt,vexnum);
        //加边
        addAllLines();
        init=true;

      return 1;
}

/**
 * @brief receivelGPS
 * @param msg
 * 接收gps位置
 */
void receivelGPS(const tcp2ros::rtkGPSmessage::ConstPtr& msg)
{
    gps_data=*msg;
}

bool calculateAngle(Point current,Point fisrt,Point second)
{
    double a,b,c;
    a=sqrt((current.x-second.x)*(current.x-second.x)+(current.y-second.y)*(current.y-second.y));
    b=sqrt((fisrt.x-second.x)*(fisrt.x-second.x)+(fisrt.y-second.y)*(fisrt.y-second.y));
    c=sqrt((current.x-fisrt.x)*(current.x-fisrt.x)+(current.y-fisrt.y)*(current.y-fisrt.y));

    double angle=acos((b*b+c*c-a*a)/(2*b*c));

    if(angle>=M_PI/2)
    {
        return true;
    }
    else if(angle<M_PI/2)
    {
        return false;
    }
}

void calculatePoint(Point current,Point first,Point second,Point& aim)
{
    if(calculateAngle(current,first,second))
    {
        aim=first;
    }
    else
    {
        double A=second.y-first.y;
        double B=first.x-second.x;
        double C=second.x*first.y-first.x*second.y;

        aim.x=(B*B*current.x-A*B*current.y-A*C)/(A*A+B*B);
        aim.y=(-A*B*current.x+A*A*current.y-B*C)/(A*A+B*B);
    }
}

/**
 * @brief readCallback
 * @param data
 * 接收上位机的命令
 */
void readCallback(const tcp2ros::readDataAll& data)
{
  //初始节点图了并且未达到终点
  if(init && !reach && data.stop==0)
  {
      //回家模式
      if(data.back_home!=last_back_home && data.back_home==1)
      {
        cout<<data.tank_id<<" "<<data.track_point_id<<" "<<data.back_home<<endl;
        cout<<(int)gps_data.vaild_flag<<" "<<gps_data.north_meter<<" "<<gps_data.east_meter<<endl; 
        //获取里程计变换，这里从前段里程计中获取
        if(gps_data.yaw_rad!=0)
        {
          Point current_pose;
          current_pose.x=gps_data.north_meter;
          current_pose.y=gps_data.east_meter;
          map<int, Node>::iterator itBegin = nodes.begin();
          double minDist1=9999999,minDist2=9999999;
          Node tartget_node,current_node;
          //目标罐体
          tartget_node=nodes[1];

          //寻找当前最近的罐体号
          for (map<int, Node>::iterator it = nodes.begin(); it != nodes.end(); it++)
          {
            double dist=getDistance(current_pose,it->second.p);
            if(dist<minDist2)
            {
              minDist2=dist;
              current_node=it->second;
            }
          }
          cout<<"current_node: "<<current_node.p.x<<" "<<current_node.p.y<<endl;
          cout<<"tartget_node: "<<tartget_node.p.x<<" "<<tartget_node.p.y<<endl;
          //找到一串的航点
          vector<int> path_node;
          path_node=findMinRoute(*GPt,current_node.num,tartget_node.num,d,pi,Q);

          //发布路径，path为rviz看的，path1为用的
          nav_msgs::Path path,path1;
          path.header.stamp=data.time;
          path.header.frame_id="base_link";
          path1.header.stamp=data.time;
          path1.header.frame_id="base_link";

          //发布路径
          for(int i=path_node.size()-1;i>=0;i--)
          {
            geometry_msgs::PoseStamped this_pose_stamped;
            geometry_msgs::PoseStamped this_pose_stamped1;
            if(i==(path_node.size()-1)&&i>=1)
            {
                Point new_point;
                calculatePoint(current_pose,nodes[path_node[i]].p,nodes[path_node[i-1]].p,new_point);
                this_pose_stamped.pose.position.x = new_point.x - itBegin->second.p.x;
                this_pose_stamped.pose.position.y = new_point.y - itBegin->second.p.y;

                this_pose_stamped1.pose.position.x = new_point.x;
                this_pose_stamped1.pose.position.y = new_point.y;
                //节点号
                this_pose_stamped1.pose.position.z = path_node[i];
            }
            else
            {
                this_pose_stamped.pose.position.x = nodes[path_node[i]].p.x - itBegin->second.p.x;
                this_pose_stamped.pose.position.y = nodes[path_node[i]].p.y - itBegin->second.p.y;

                this_pose_stamped1.pose.position.x = nodes[path_node[i]].p.x;
                this_pose_stamped1.pose.position.y = nodes[path_node[i]].p.y;
                //节点号
                this_pose_stamped1.pose.position.z = path_node[i];
            }

            //判断终点
            double th;
            //不是终点，则根据下一个目标点的航向
            if(i!=0)
            {
              double x2=nodes[path_node[i-1]].p.x;
              double y2=nodes[path_node[i-1]].p.y;
              double x1=nodes[path_node[i]].p.x;
              double y1=nodes[path_node[i]].p.y;

              th=atan2((double)(y2-y1),double(x2-x1+0.00001));
            }
            //是终点
            else
            {
              //如果目标是起点，则起点的角度为目标角度
              if(path_node[i]==1)
                th=nodes[path_node[i]].p.yaw;
            }

            geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(th);
            this_pose_stamped.pose.orientation.x = goal_quat.x;
            this_pose_stamped.pose.orientation.y = goal_quat.y;
            this_pose_stamped.pose.orientation.z = goal_quat.z;
            this_pose_stamped.pose.orientation.w = goal_quat.w;

            this_pose_stamped1.pose.orientation.x = goal_quat.x;
            this_pose_stamped1.pose.orientation.y = goal_quat.y;
            this_pose_stamped1.pose.orientation.z = goal_quat.z;
            this_pose_stamped1.pose.orientation.w = goal_quat.w;

            this_pose_stamped.header.stamp=data.time;
            this_pose_stamped.header.frame_id="base_link";

            this_pose_stamped1.header.stamp=data.time;
            this_pose_stamped1.header.frame_id="base_link";

            cout<<path_node[i]<<endl;
            cout<<this_pose_stamped1.pose.position.x<<" "<<this_pose_stamped1.pose.position.y<<" "<<th<<endl;
            path.poses.push_back(this_pose_stamped);
            path1.poses.push_back(this_pose_stamped1);
          }
          cout<<endl;
          path_pub_->publish(path);
          path_pub1_->publish(path1);
          //reach=false;
          last_target=data.track_point_id;
          last_tank_target=data.tank_id;
          //last_back_home=data.back_home;
        }
      }
      //获取节点号,不是回家模式
      else if(data.track_point_id!=0 && (data.track_point_id!=last_target || data.tank_id!=last_tank_target))
      {
         cout<<data.tank_id<<" "<<data.track_point_id<<" "<<data.back_home<<endl;
         cout<<(int)gps_data.vaild_flag<<" "<<gps_data.north_meter<<" "<<gps_data.east_meter<<endl;
         //获取里程计变换，这里从前段里程计中获取
         if(gps_data.yaw_rad!=0)
         {
              //当前位置
              Point current_pose;
              current_pose.x=gps_data.north_meter;
              current_pose.y=gps_data.east_meter;

              //寻找最近的目标节点和当前节点
              double minDist1=9999999,minDist2=9999999;
              Node tartget_node,other_target_node,current_node;
              //目标罐体
              tartget_node=nodes[data.track_point_id];

              map<int, Node>::iterator itBegin = nodes.begin();
              //寻找当前最近的罐体号
              for (map<int, Node>::iterator it = nodes.begin(); it != nodes.end(); it++)
              {
                double dist=getDistance(current_pose,it->second.p);
                if(dist<minDist2)
                {
                  minDist2=dist;
                  current_node=it->second;
                }
              }
              cout<<"current_node: "<<current_node.p.x<<" "<<current_node.p.y<<endl;
              cout<<"tartget_node: "<<tartget_node.p.x<<" "<<tartget_node.p.y<<endl;
              //找到一串的航点
              vector<int> path_node;
              path_node=findMinRoute(*GPt,current_node.num,tartget_node.num,d,pi,Q);

              //发布路径，path为rviz看的，path1为用的
              nav_msgs::Path path,path1;
              path.header.stamp=data.time;
              path.header.frame_id="base_link";
              path1.header.stamp=data.time;
              path1.header.frame_id="base_link";

              //发布路径
              for(int i=path_node.size()-1;i>=0;i--)
              {
                geometry_msgs::PoseStamped this_pose_stamped;
                geometry_msgs::PoseStamped this_pose_stamped1;
                if(i==(path_node.size()-1)&&i>=1)
                {
                    Point new_point;
                    calculatePoint(current_pose,nodes[path_node[i]].p,nodes[path_node[i-1]].p,new_point);

                    this_pose_stamped.pose.position.x = new_point.x - itBegin->second.p.x;
                    this_pose_stamped.pose.position.y = new_point.y - itBegin->second.p.y;

                    this_pose_stamped1.pose.position.x = new_point.x;
                    this_pose_stamped1.pose.position.y = new_point.y;
                    //节点号
                    this_pose_stamped1.pose.position.z = path_node[i];
                }
                else
                {
                    this_pose_stamped.pose.position.x = nodes[path_node[i]].p.x - itBegin->second.p.x;
                    this_pose_stamped.pose.position.y = nodes[path_node[i]].p.y - itBegin->second.p.y;

                    this_pose_stamped1.pose.position.x = nodes[path_node[i]].p.x;
                    this_pose_stamped1.pose.position.y = nodes[path_node[i]].p.y;
                    //节点号
                    this_pose_stamped1.pose.position.z = path_node[i];
                }

  //              //0为原点，1为非原点
  //              if(path_node[i]==0)
  //                this_pose_stamped1.pose.position.z=0;
  //              else
  //                this_pose_stamped1.pose.position.z=1;

                //判断终点
                double th;
                //不是终点，则根据下一个目标点的航向
                if(i!=0)
                {
                  double x2=nodes[path_node[i-1]].p.x;
                  double y2=nodes[path_node[i-1]].p.y;
                  double x1=nodes[path_node[i]].p.x;
                  double y1=nodes[path_node[i]].p.y;

                  th=atan2((double)(y2-y1),double(x2-x1+0.00001));
                }
                //是终点
                else
                {
                  //如果目标是起点，则起点的角度为目标角度
                  if(data.track_point_id==1)
                    th=nodes[path_node[i]].p.yaw;
                  //如果目标不是起点,且当前处于自动模式，则根据另一个方向的点来判断角度
                  else if(data.tank_id!=0)
                  {
                    vector<int> tank_nodes=nodeMap[data.tank_id];
                    other_target_node = data.track_point_id==tank_nodes[0]?nodes[tank_nodes[1]]:nodes[tank_nodes[0]];
                    double x2=other_target_node.p.x;
                    double y2=other_target_node.p.y;
                    double x1=tartget_node.p.x;
                    double y1=tartget_node.p.y;

                    th=atan2((double)(y2-y1),(double)(x2-x1+0.00001));
                  }
                  //处于半自动走法，则目标角度根据上一个角度
                  else if(data.tank_id==0)
                  {
                    double x2=nodes[path_node[i]].p.x;
                    double y2=nodes[path_node[i]].p.y;
                    double x1=nodes[path_node[i+1]].p.x;
                    double y1=nodes[path_node[i+1]].p.y;
                    th=atan2((double)(y2-y1),(double)(x2-x1+0.00001));
                  }
                }

                geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(th);
                this_pose_stamped.pose.orientation.x = goal_quat.x;
                this_pose_stamped.pose.orientation.y = goal_quat.y;
                this_pose_stamped.pose.orientation.z = goal_quat.z;
                this_pose_stamped.pose.orientation.w = goal_quat.w;

                this_pose_stamped1.pose.orientation.x = goal_quat.x;
                this_pose_stamped1.pose.orientation.y = goal_quat.y;
                this_pose_stamped1.pose.orientation.z = goal_quat.z;
                this_pose_stamped1.pose.orientation.w = goal_quat.w;

                this_pose_stamped.header.stamp=data.time;
                this_pose_stamped.header.frame_id="base_link";

                this_pose_stamped1.header.stamp=data.time;
                this_pose_stamped1.header.frame_id="base_link";

                cout<<path_node[i]<<endl;
                cout<<this_pose_stamped1.pose.position.x<<" "<<this_pose_stamped1.pose.position.y<<" "<<th<<endl;
                path.poses.push_back(this_pose_stamped);
                path1.poses.push_back(this_pose_stamped1);
             }
             cout<<endl;
             path_pub_->publish(path);
             path_pub1_->publish(path1);
             //reach=false;
             last_target=data.track_point_id;
             last_tank_target=data.tank_id;
         }
     }
  }
  else if(data.stop==1)
  {
	last_target=0;
	last_tank_target=0;
	last_back_home=0;
  }
  //last_target=data.track_point_id;
  //last_tank_target=data.tank_id;
  last_back_home=data.back_home;       
}

/**
 * @brief receiveReach
 * @param msg
 * 接收是否达到目的地
 */
void receiveReach(const tcp2ros::reach::ConstPtr& msg)
{
  if(msg->reach==1)
    reach=true;
  else if(msg->reach==0)
    reach=false;
}

/**
 * @brief receiveGoal
 * @param msg
 * 接收rviz的目标
 */
void receiveGoal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Point target_pose;
    double target_angle;
    map<int, Node>::iterator itBegin = nodes.begin();
    target_pose.x = msg->pose.position.x + itBegin->second.p.x;
    target_pose.y = msg->pose.position.y + itBegin->second.p.y;
    target_angle = tf::getYaw(msg->pose.orientation);

    if(init && !reach)
    {
        //获取里程计变换，这里从前段里程计中获取
        if(gps_data.vaild_flag)
        {
            //当前位置
            Point current_pose;
            current_pose.x=gps_data.north_meter;
            current_pose.y=gps_data.east_meter;

            //获取目标节点和当前节点
            double minDist1=9999999,minDist2=9999999;
            Node tartget_node,current_node;

            //寻找当前最近的罐体号
            for (map<int, Node>::iterator it = nodes.begin(); it != nodes.end(); it++)
            {
                    double dist=getDistance(target_pose,it->second.p);
                    if(dist<minDist1)
                    {
                        minDist1=dist;
                        tartget_node=it->second;
                    }
            }
            for (map<int, Node>::iterator it = nodes.begin(); it != nodes.end(); it++)
            {
                    double dist=getDistance(current_pose,it->second.p);
                    if(dist<minDist2)
                    {
                        minDist2=dist;
                        current_node=it->second;
                    }
            }
            cout<<"current_node: "<<current_node.p.x<<" "<<current_node.p.y<<endl;
            cout<<"tartget_node: "<<tartget_node.p.x<<" "<<tartget_node.p.y<<endl;
            //找到一串的航点
            vector<int> path_node;
            path_node=findMinRoute(*GPt,current_node.num,tartget_node.num,d,pi,Q);
            nav_msgs::Path path,path1;
            path.header.stamp=msg->header.stamp;
            path.header.frame_id="base_link";

            double x2=nodes[path_node[path_node.size()-1]].p.x;
            double y2=nodes[path_node[path_node.size()-1]].p.y;
            double x1=current_pose.x;
            double y1=current_pose.y;
            double th;
            th=atan2((double)(y2-y1),double(x2-x1+0.00001));

            geometry_msgs::PoseStamped this_pose_stamped;
            geometry_msgs::PoseStamped this_pose_stamped1;
            this_pose_stamped.pose.position.x = current_pose.x - itBegin->second.p.x;
            this_pose_stamped.pose.position.y = current_pose.y - itBegin->second.p.y;

            this_pose_stamped1.pose.position.x = current_pose.x;
            this_pose_stamped1.pose.position.y = current_pose.y;
            this_pose_stamped1.pose.position.z = path_node[path_node.size()-1];

            geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(th);
            this_pose_stamped.pose.orientation.x = goal_quat.x;
            this_pose_stamped.pose.orientation.y = goal_quat.y;
            this_pose_stamped.pose.orientation.z = goal_quat.z;
            this_pose_stamped.pose.orientation.w = goal_quat.w;

            this_pose_stamped1.pose.orientation.x = goal_quat.x;
            this_pose_stamped1.pose.orientation.y = goal_quat.y;
            this_pose_stamped1.pose.orientation.z = goal_quat.z;
            this_pose_stamped1.pose.orientation.w = goal_quat.w;

            this_pose_stamped.header.stamp=msg->header.stamp;
            this_pose_stamped.header.frame_id="base_link";

            this_pose_stamped1.header.stamp=msg->header.stamp;
            this_pose_stamped1.header.frame_id="base_link";

            path.poses.push_back(this_pose_stamped);
            path1.poses.push_back(this_pose_stamped1);
            //发布路径
            for(int i=path_node.size()-1;i>=0;i--)
            {
                    geometry_msgs::PoseStamped this_pose_stamped;
                    geometry_msgs::PoseStamped this_pose_stamped1;
                    this_pose_stamped.pose.position.x = nodes[path_node[i]].p.x - itBegin->second.p.x;
                    this_pose_stamped.pose.position.y = nodes[path_node[i]].p.y - itBegin->second.p.y;

                    this_pose_stamped1.pose.position.x = nodes[path_node[i]].p.x;
                    this_pose_stamped1.pose.position.y = nodes[path_node[i]].p.y;
                    this_pose_stamped1.pose.position.z = path_node[i];
//                    //0为原点，1为非原点
//                    if(path_node[i]==0)
//                        this_pose_stamped1.pose.position.z=0;
//                    else
//                        this_pose_stamped1.pose.position.z=1;

                    double th;
                    if(i!=0)
                    {
                            double x2=nodes[path_node[i-1]].p.x;
                            double y2=nodes[path_node[i-1]].p.y;
                            double x1=nodes[path_node[i]].p.x;
                            double y1=nodes[path_node[i]].p.y;
                            th=atan2((double)(y2-y1),(double)(x2-x1+0.00001));
                    }
                    else
                    {
                            th=nodes[path_node[i]].p.yaw;
                    }

                    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(th);
                    this_pose_stamped.pose.orientation.x = goal_quat.x;
                    this_pose_stamped.pose.orientation.y = goal_quat.y;
                    this_pose_stamped.pose.orientation.z = goal_quat.z;
                    this_pose_stamped.pose.orientation.w = goal_quat.w;

                    this_pose_stamped1.pose.orientation.x = goal_quat.x;
                    this_pose_stamped1.pose.orientation.y = goal_quat.y;
                    this_pose_stamped1.pose.orientation.z = goal_quat.z;
                    this_pose_stamped1.pose.orientation.w = goal_quat.w;

                    this_pose_stamped.header.stamp=msg->header.stamp;
                    this_pose_stamped.header.frame_id="base_link";

                    this_pose_stamped1.header.stamp=msg->header.stamp;
                    this_pose_stamped1.header.frame_id="base_link";

                    cout<<path_node[i]<<endl;
                    //cout<<nodes[path_node[i]].p.x<<" "<<nodes[path_node[i]].p.y<<" "<<th<<endl;
                    path.poses.push_back(this_pose_stamped);
                    path1.poses.push_back(this_pose_stamped1);
            } 
            path_pub_->publish(path);
            path_pub1_->publish(path1);
        }
    }
}
int main(int argc,char ** argv)
{
    ros::init (argc, argv, "pathPublish");
    ros::NodeHandle ph;

    ros::Publisher path_pub = ph.advertise<nav_msgs::Path>("trajectory",1, true);
    ros::Publisher path_pub1 = ph.advertise<nav_msgs::Path>("path_node",1, true);
    ros::Publisher nodes_pub = ph.advertise<visualization_msgs::Marker>("nodes", 10,false);
    ros::Publisher tanks_pub = ph.advertise<visualization_msgs::Marker>("tanks", 10,false);
    ros::Publisher nodeLines_pub = ph.advertise<visualization_msgs::Marker>("nodeLines", 10,false);
    ros::Publisher start_pub = ph.advertise<sensor_msgs::ChannelFloat32>("path_publish_start", 10,false);

    ros::Subscriber target_sub = ph.subscribe("/move_base_simple/goal", 1, &receiveGoal);
    ros::Subscriber target_sub1 = ph.subscribe("/tcpData",10,readCallback);
    ros::Subscriber gps_sub = ph.subscribe<tcp2ros::rtkGPSmessage>("rtkGPS_filter",1, &receivelGPS);

    nodes_pub_=&nodes_pub;
    nodeLines_pub_=&nodeLines_pub;
    tanks_pub_=&tanks_pub;
    path_pub_=&path_pub;
    path_pub1_=&path_pub1;
    tf_listener_=new tf::TransformListener(ros::Duration(180));

    boost::shared_ptr<ALGraph> t(new ALGraph());
    GPt=t;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    //读取节点地图
    readPath(fileName);
    ros::Rate r(50);
    while(ros::ok())
    {
      sensor_msgs::ChannelFloat32 start_flag;
      start_flag.name="path_publish_start";
      start_flag.values.push_back(1.0);
      start_pub.publish(start_flag);

      if(showMark)
      {
	      //显示nodes
	      if (nodes_pub_->getNumSubscribers() > 0)
	      {
		    visualization_msgs::Marker m;
		    m.header.frame_id = "base_link";
		    m.ns = "base_link";
		    m.id = 2;
		    m.action = visualization_msgs::Marker::ADD;
		    m.type = visualization_msgs::Marker::SPHERE_LIST;
		    m.color.r = 0.3;
		    m.color.g = 0.0;
		    m.color.b = 1.0;
		    m.color.a = 0.8;
		    m.scale.x = 1.0;
		    m.scale.y = 1.0;
		    m.scale.z = 1.0;

		    map<int, Node>::iterator itBegin = nodes.begin();
		    for (map<int, Node>::iterator it = nodes.begin(); it != nodes.end(); it++)
		    {
		            geometry_msgs::Point msg;
		            msg.x = it->second.p.x - itBegin->second.p.x;
		            msg.y = it->second.p.y - itBegin->second.p.y;
		            msg.z = 0;
		            m.points.push_back(msg);
		    }
		    nodes_pub_->publish(m);
	      }

	      //显示nodeLine
	      if (nodeLines_pub_->getNumSubscribers() > 0)
	      {
		    visualization_msgs::Marker m;
		    m.header.frame_id = "base_link";
		    m.ns = "base_link";
		    m.id = 1;
		    m.action = visualization_msgs::Marker::ADD;
		    m.type = visualization_msgs::Marker::LINE_LIST;
		    m.color.r = 0.6;
		    m.color.g = 0.6;
		    m.color.b = 0.0;
		    m.color.a = 0.8;
		    m.scale.x = 0.2;

		    map<int, Node>::iterator itBegin = nodes.begin();
		    for (size_t ii = 0; ii < nodeLines.size(); ++ii)
		    {
		          geometry_msgs::Point msg1;
		          msg1.x = nodeLines[ii].start.p.x - itBegin->second.p.x;
		          msg1.y = nodeLines[ii].start.p.y - itBegin->second.p.y;
		          msg1.z = 0;
		          geometry_msgs::Point msg2;
		          msg2.x = nodeLines[ii].end.p.x - itBegin->second.p.x;
		          msg2.y = nodeLines[ii].end.p.y - itBegin->second.p.y;
		          msg2.z = 0;
		          m.points.push_back(msg1);
		          m.points.push_back(msg2);
		    }
		    nodeLines_pub_->publish(m);
	      }

	      //显示罐体
	      if(nodeLines_pub_->getNumSubscribers() > 0)
	      {
		visualization_msgs::Marker m;
		m.header.frame_id = "base_link";
		m.ns = "base_link";
		m.id = 3;
		m.action = visualization_msgs::Marker::ADD;
		m.type = visualization_msgs::Marker::SPHERE_LIST;
		m.color.r = 1.0;
		m.color.g = 0.0;
		m.color.b = 0.3;
		m.color.a = 0.8;
		m.scale.x = 1.0;
		m.scale.y = 1.0;
		m.scale.z = 1.0;

		map<int, Node>::iterator itBegin = nodes.begin();
		for (map<int,vector<int> >::iterator it = nodeMap.begin(); it != nodeMap.end(); it++)
		{
		  geometry_msgs::Point msg;
		  msg.x = (nodes[it->second[0]].p.x+nodes[it->second[1]].p.x)/2 - itBegin->second.p.x;
		  msg.y = (nodes[it->second[0]].p.y+nodes[it->second[1]].p.y)/2 - itBegin->second.p.y;
		  msg.z = 0;
		  m.points.push_back(msg);
		}
		tanks_pub_->publish(m);
	      }
      }
      ros::spinOnce();
      r.sleep();
    }
    return 0;
}
