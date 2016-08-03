#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

ros::Publisher tf_bp_pub, nav_ob_pub, initpose_mo_pub;
tf::Transform prevTransform;

void listen_transform(std::string parent, std::string child)
{
  tf::TransformListener listener;
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
  
}

void tf_ob_callback(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
  nav_msgs::Odometry odom;
  tf::StampedTransform currentTransform;
  geometry_msgs::TransformStamped ts = *msg;
  tf::transformStampedMsgToTF(ts, currentTransform);
  odom.header.stamp = ts.header.stamp;
  odom.header.frame_id = ts.header.frame_id;
  odom.child_frame_id = ts.child_frame_id;
  tf::poseTFToMsg(currentTransform, odom.pose.pose);
  tfScalar delx =  currentTransform.getOrigin().x() - prevTransform.getOrigin().x();
  tfScalar dely =  currentTransform.getOrigin().y() - prevTransform.getOrigin().y();
  odom.twist.twist.angular.z = 4.0 * atan2(dely, delx);
  odom.twist.twist.linear.x = 0.5 * sqrt(pow(delx, 2) + pow(dely, 2));
  nav_ob_pub.publish(odom);
  prevTransform = currentTransform;
}


void tf_mo_callback(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
  geometry_msgs::PoseWithCovarianceStamped initpose;
  geometry_msgs::TransformStamped ts = *msg;
  tf::StampedTransform transform;
  tf::transformStampedMsgToTF(ts, transform);
  initpose.header.stamp = ros::Time::now();
  initpose.header.frame_id = ts.header.frame_id;
  tf::poseTFToMsg(transform, initpose.pose.pose); 
  initpose_mo_pub.publish(initpose);
}

int main (int argc, char** argv)
{
  ros::init(argc,argv,"tf_extractor");
  ros::NodeHandle nh;
  ros::Subscriber tf_ob_sub, tf_mo_sub;
  prevTransform.setIdentity();
  //all publishers and subscribers
  tf_bp_pub = nh.advertise<geometry_msgs::TransformStamped>("/turtlebot01/transform_bf_p3", 1);
  nav_ob_pub = nh.advertise<nav_msgs::Odometry>("/overhead_odom", 1);
  initpose_mo_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/overhead_initial_pose", 1);
  tf_ob_sub = nh.subscribe<geometry_msgs::TransformStamped>("/turtlebot01/transform_odom_bf", 1, tf_ob_callback);
  tf_mo_sub = nh.subscribe<geometry_msgs::TransformStamped>("/turtlebot01/transform_map_odom", 1, tf_mo_callback);
  
  //publishing static transform between plate3 and base footprint from tf tree
  std::string child = "/plate_3_link";
  std::string parent = "/base_footprint";
  while(ros::ok())
  {
    listen_transform(parent,child);
    ros::spinOnce();
  }
}

