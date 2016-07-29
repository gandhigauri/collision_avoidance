#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

ros::Publisher tf_bp_pub, tf_ob_pub;
ros::Subscriber tf_ob_sub, tf_mo_sub;

 void listen_transform( std::string parent,std::string child)
{
  tf::TransformListener listener;
  while (ros::ok()) {
  tf::StampedTransform transform;
  try
  {
    listener.waitForTransform(parent, child, ros::Time(0), ros::Duration(10.0));
    listener.lookupTransform(parent, child, ros::Time(0), transform);
    geometry_msgs::TransformStamped transform_stamped;
    tf::transformStampedTFToMsg(transform,transform_stamped);
    tf_bp_pub.publish(transform_stamped);   
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  ros::spin();
  }
}
void send_transform(std::string parent, std::string child, tf::Transform transform)
{
  tf::TransformBroadcaster br;
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent, child));
}


void tf_ob_callback(const geometry_msgs::TransformStamped::ConstPtr & msg)
{
  nav_msgs::Odometry odom;
  tf::StampedTransform transform;
  geometry_msgs::TransformStamped ts = *msg;
  tf::transformStampedMsgToTF(ts, transform);
  send_transform("/odom","/base_footprint",transform);
  odom.header = ts.header;
  odom.child_frame_id = ts.child_frame_id;
  tf::poseTFToMsg(transform, odom.pose.pose);
  tf_ob_pub.publish(odom);
}


void tf_mo_callback(const geometry_msgs::TransformStamped::ConstPtr & msg)
{
  tf::StampedTransform transform;
  tf::transformStampedMsgToTF(*msg, transform);
  send_transform("/map","/odom",transform);

}

int main (int argc, char** argv)
{
	ros::init(argc,argv,"tf_extractor");
	ros::NodeHandle nh;
	std::string child= "/plate_3_link";
	std::string parent = "/base_footprint";
  tf_bp_pub = nh.advertise<geometry_msgs::TransformStamped>("/transform_bf_p3",1);
  tf_ob_pub = nh.advertise<nav_msgs::Odometry>("/overhead_odom",1);
  tf_ob_sub = nh.subscribe("/tranform_odom_bf",10,&tf_ob_callback);
  tf_mo_sub = nh.subscribe("/tranform_map_odom",10,&tf_mo_callback);
  listen_transform(parent,child);
}

