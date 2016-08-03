#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

ros::Publisher tf_bp_pub, nav_ob_pub, initpose_mo_pub;
tf::Transform prevTransform;
ros::Time prevTime;
tfScalar prevYaw;
double LINEAR_THRESHOLD = 0.03, ANGULAR_THRESHOLD = 0.01;

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
  ros::Time currentTime = ros::Time::now();
  geometry_msgs::TransformStamped ts = *msg;
  tf::transformStampedMsgToTF(ts, currentTransform);
  odom.header.stamp = currentTime;
  odom.header.frame_id = ts.header.frame_id;
  odom.child_frame_id = ts.child_frame_id;
  tf::poseTFToMsg(currentTransform, odom.pose.pose);


  //velocity calculation for odom
  tfScalar currentRoll, currentPitch, currentYaw;
  currentTransform.getBasis().getRPY(currentRoll, currentPitch, currentYaw);
  //std::cout<<" roll "<<roll<<" pitch "<<pitch<<" yaw "<<yaw<<std::endl;
  //double del_x =  currentTransform.getOrigin().x() - prevTransform.getOrigin().x();
  //double del_y =  currentTransform.getOrigin().y() - prevTransform.getOrigin().y();
  double del_d = sqrt(pow(currentTransform.getOrigin().x(), 2) + pow(currentTransform.getOrigin().y(), 2)) - sqrt(pow(prevTransform.getOrigin().x(), 2) + pow(prevTransform.getOrigin().y(), 2));
  double del_th =  currentYaw - prevYaw;
  double dt = (currentTime - prevTime).toSec();
  if (std::abs(del_th) > ANGULAR_THRESHOLD)
    odom.twist.twist.angular.z = del_th/dt;
  if (std::abs(del_d) > LINEAR_THRESHOLD)
    odom.twist.twist.linear.x = del_d/dt;//sqrt(pow(del_x, 2) + pow(del_y, 2))/dt;
  nav_ob_pub.publish(odom);
  prevTransform = currentTransform;
  prevTime = currentTime;
  prevYaw = currentYaw;
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
  prevTime = ros::Time::now();
  prevYaw = 0;

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

