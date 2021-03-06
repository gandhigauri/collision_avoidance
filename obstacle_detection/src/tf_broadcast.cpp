#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>

tf::StampedTransform listen_transform(std::string parent, std::string child)
{
  tf::TransformListener listener;
  while (ros::ok()) {
  tf::StampedTransform transform;
  try
  {
    listener.waitForTransform(parent, child, ros::Time(0), ros::Duration(10.0));
    listener.lookupTransform(parent, child, ros::Time(0), transform);
    return transform;
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcast");
  ros::NodeHandle nh;
  //ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("/odom", 1000);
  ros::Publisher pub = nh.advertise<geometry_msgs::TransformStamped>("turtlebot01/transform_bf_p3",1);
  //tf::TransformBroadcaster br;
  tf::StampedTransform transform = listen_transform("base_footprint","plate_3_link");
  geometry_msgs::TransformStamped ts;
  //geometry_msgs::Transform t;
  //nav_msgs::Odometry od;
  //od.header.frame_id = "frame1";
  //od.child_frame_id = "frame2";
  ros::Rate rate(10.0);
  while (nh.ok()){
   tf::transformStampedTFToMsg(transform,ts);
  
 //transform.setOrigin( tf::Vector3(1, 1, 0.0) );
  //tf::Quaternion q;
  //q.setRPY(0, 0, 1);
  //transform.setRotation(q);
  //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "frame1", "base_footprint"));
  pub.publish(ts);
  rate.sleep();
}
  return 0;
};

