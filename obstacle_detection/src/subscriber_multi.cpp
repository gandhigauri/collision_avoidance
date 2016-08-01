#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

ros::Publisher chatter_pub;
std_msgs::String st;

void chatter_cb(const std_msgs::String::ConstPtr& msg)
{
  //std_msgs::String chtr = *msg;
  //st = *msg;
  chatter_pub.publish(*msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscriber_multi");
  ros::NodeHandle n;
  ros::Subscriber chatter_sub1 = n.subscribe("/chatter1",1000,chatter_cb);
  ros::Subscriber chatter_sub2 = n.subscribe("/chatter2",1000,chatter_cb);
  ros::Subscriber chatter_sub3 = n.subscribe("/chatter3",1000,chatter_cb);
  chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
//ros::Rate rate(10);
  ros::spin();
 //return 0;
}
