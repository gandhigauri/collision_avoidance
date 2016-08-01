#include <ros/ros.h>
#include <collvoid_msgs/PoseTwistWithCovariance.h>
#include <map>
#include <boost/foreach.hpp>
#include <boost/range/adaptor/map.hpp>
#include <iostream>

std::map<std::string, int> robot_ids;
std::map<int, ros::Publisher> pose_pubs;
ros::Publisher pose_pub;
collvoid_msgs::PoseTwistWithCovariance pos;

void pose_cb(const collvoid_msgs::PoseTwistWithCovariance::ConstPtr& msg)
{
  pos = *msg;  
  pose_pub.publish(*msg);
  BOOST_FOREACH(const std::string & name, robot_ids | boost::adaptors::map_keys) {
    int rid = robot_ids[name];
    pose_pubs[rid].publish(*msg);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "collvoid_sync");
  ros::NodeHandle nh;
  nh.getParam("robot_ids", robot_ids);
  std::map<int, ros::Subscriber> pose_subs;
  pose_pub = nh.advertise<collvoid_msgs::PoseTwistWithCovariance>("position_share", 1);
  BOOST_FOREACH(const std::string & name, robot_ids | boost::adaptors::map_keys) {
    int rid = robot_ids[name];
    std::string pub_topic("/" + name + "/position_share_in");
    std::string sub_topic("/" + name + "/position_share_out");
    pose_pubs[rid] = nh.advertise<collvoid_msgs::PoseTwistWithCovariance>(pub_topic, 1);
    pose_subs[rid] = nh.subscribe<collvoid_msgs::PoseTwistWithCovariance>(
      sub_topic, 1, pose_cb);
  }  
  ros::spin();
  return 0;
}
