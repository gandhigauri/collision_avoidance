#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <nav_msgs/Odometry.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcast");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("/odom", 1000);
  tf::TransformBroadcaster br;
  tf::Transform transform;
  nav_msgs::Odometry od;
  od.header.frame_id = "frame1";
  od.child_frame_id = "frame2";
  ros::Rate rate(10.0);
  while (nh.ok()){
  //transform.setOrigin( tf::Vector3(1, 1, 0.0) );
  //tf::Quaternion q;
  //q.setRPY(0, 0, 1);
  //transform.setRotation(q);
  //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "frame1", "base_footprint"));
  pub.publish(od);
  rate.sleep();
}
  return 0;
};

