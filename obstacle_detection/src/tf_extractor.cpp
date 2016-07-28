#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

ros::Publisher tf_pub;
ros::Subscriber tf_sub;

 void listen_transform( std::string parent,std::string child)
{
  tf::TransformListener listener;
  while (ros::ok()) {
 tf::StampedTransform transform;
  try
  {
    listener.waitForTransform(child, parent, ros::Time(0), ros::Duration(10.0));
    listener.lookupTransform(child, parent, ros::Time(0), transform);
    geometry_msgs::TransformStamped transform_stamped;
    transformStampedTFToMsg(transform,transform_stamped);
    tf_pub.publish(transform_stamped);   
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  }
}
void send_transform(std::string parent, std::string child, tf::Transform transform)
{
  tf::TransformBroadcaster br;
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent, child));
}


void tf_callback(const geometry_msgs::TransformStamped::ConstPtr & msg)
{
  tf::StampedTransform transform;
  transformStampedMsgToTF(*msg, transform);
  send_transform("/odom","/base_footprint",transform);

}

int main (int argc, char** argv)
{
	ros::init(argc,argv,"tf_extractor");
	ros::NodeHandle nh;
	std::string child= "/plate_3_link";
	std::string parent = "/base_footprint";
    tf_pub = nh.advertise<geometry_msgs::TransformStamped>("/transform_bf_p3",1);
    tf_sub = nh.subscribe("/tranform_odom_bf",10,&tf_callback);
    listen_transform(parent,child);
}

