#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  ros::service::waitForService("spawn");
  ros::ServiceClient add_turtle =
    node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  add_turtle.call(srv);

  ros::Publisher turtle_vel =
    node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;

    // try{
    // //   listener.lookupTransform("/turtle2", "/turtle1",
    // //                            ros::Time(0), transform);
    // //   listener.lookupTransform("/turtle2", "/carrot1",
    // //                             ros::Time(0), transform);

    // //  error: Lookup would require extrapolation into the future.  Requested time 1532397905.219252465 but the latest data is at time 1532397905.204810638, when looking up transform from frame [carrot1] to frame [turtle2]
    // //  reason: When a broadcaster sends out a transform, it takes some time before that transform gets into the buffer (usually a couple of milliseconds). So, when you request a frame transform at time "now", you should wait a few milliseconds for that information to arrive.  
    // //   listener.lookupTransform("/turtle2", "/carrot1",
    // //                             ros::Time::now(), transform);

    // // For real tf use cases, it is often perfectly fine to use Time(0). 
    // // To use time::now
    // ros::Time now = ros::Time::now();
    // listener.waitForTransform("/turtle2", "/turtle1",
    //                           now, ros::Duration(3.0));
    // listener.lookupTransform("/turtle2", "/turtle1",
    //                          now, transform);
    // }

    //time travel
    try{
        //wrong code! We asked tf, "What was the pose of /turtle1 5 seconds ago, relative to /turtle2 5 seconds ago?". This means we are controlling the second turtle based on where it was 5 seconds ago as well as where the first turtle was 5 seconds ago. What we really want to ask is, "What was the pose of /turtle1 5 seconds ago, relative to the current position of the /turtle2?". 
        // ros::Time past = ros::Time::now() - ros::Duration(5.0);
        // listener.waitForTransform("/turtle2", "/turtle1",
        //                         past, ros::Duration(1.0));
        // listener.lookupTransform("/turtle2", "/turtle1",
        //                         past, transform);

        //right code
        ros::Time now = ros::Time::now();
        ros::Time past = now - ros::Duration(5.0);
        listener.waitForTransform("/turtle2", now,
                                "/turtle1", past,
                                "/world", ros::Duration(1.0));
        listener.lookupTransform("/turtle2", now,
                                "/turtle1", past,
                                "/world", transform);
    }

    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
                                    transform.getOrigin().x());
    vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                  pow(transform.getOrigin().y(), 2));
    turtle_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
};