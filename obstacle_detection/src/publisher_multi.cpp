#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publisher_multi");
  ros::NodeHandle n;
  ros::Publisher chatter_pub1 = n.advertise<std_msgs::String>("chatter1", 1000);
  ros::Publisher chatter_pub2 = n.advertise<std_msgs::String>("chatter2", 1000);
  ros::Publisher chatter_pub3 = n.advertise<std_msgs::String>("chatter3", 1000);
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std_msgs::String msg1;
    std_msgs::String msg2;
    std_msgs::String msg3;
    std::stringstream ss1;
    std::stringstream ss2;
    std::stringstream ss3;
    ss1 << "hello world ";
    ss2 << "hi world ";
    ss3 << "kse ho world ";
    msg1.data = ss1.str();
    msg2.data = ss2.str();
    msg3.data = ss3.str();
    chatter_pub1.publish(msg1);
    chatter_pub2.publish(msg2);
    chatter_pub3.publish(msg3);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}