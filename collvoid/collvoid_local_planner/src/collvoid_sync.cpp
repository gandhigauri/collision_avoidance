#include <ros/ros.h>
#include <collvoid_msgs/PoseTwistWithCovariance.h>

ros::Publisher pose_pub1;
ros::Publisher pose_pub2;
ros::Publisher pose_pub3;
ros::Publisher pose_pub;

void pose_cb(const collvoid_msgs::PoseTwistWithCovariance::ConstPtr& msg)
{
  pose_pub.publish(*msg);
  pose_pub1.publish(*msg);
  pose_pub2.publish(*msg);
  pose_pub3.publish(*msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "collvoid_sync");
  ros::NodeHandle n;
  ros::Subscriber pose_sub1 = n.subscribe("/turtlebot01/position_share_out",1000,pose_cb);
  ros::Subscriber pose_sub2 = n.subscribe("/turtlebot02/position_share_out",1000,pose_cb);
  ros::Subscriber pose_sub3 = n.subscribe("/turtlebot03/position_share_out",1000,pose_cb);
  pose_pub = n.advertise<collvoid_msgs::PoseTwistWithCovariance>("position_share", 1000);
  pose_pub1 = n.advertise<collvoid_msgs::PoseTwistWithCovariance>("/turtlebot01/position_share_in", 1000);
  pose_pub2 = n.advertise<collvoid_msgs::PoseTwistWithCovariance>("/turtlebot02/position_share_in", 1000);
  pose_pub3 = n.advertise<collvoid_msgs::PoseTwistWithCovariance>("/turtlebot03/position_share_in", 1000);
  ros::spin();
  return 0;
}