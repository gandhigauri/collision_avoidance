/*
 *  Copyright 2016 Sasanka Nagavalli
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>

#include <iostream>
#include <map>

#include <boost/foreach.hpp>

#include <boost/algorithm/clamp.hpp>
#include <boost/range/adaptor/map.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <angles/angles.h>

#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <std_msgs/String.h>

#include "turtlebot_behaviors/turtlebot_behaviors.h"

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>

namespace {

const int MAP_ORIGIN_TAG_ID = 50;
bool reset_map = true;
std::map<int, bool> reset_odom, pub_check;
std::map<std::string, boost::function0<void> > behaviors;
std::string current_behavior = "";
std::string current_leader = "";
std::map<int, ros::Publisher> cmd_pubs, tf_ob_pubs, tf_mo_pubs;
std::map<int, Eigen::Affine3d> Tbf_tag, tag_origin, Tod_or, Tmap_od, Tod_bf;
Eigen::Affine3d Tmap_root;

void poseCallback(const apriltags_ros::AprilTagDetectionArray::ConstPtr & msg) 
{
  // XXX: robot to tag frame (move this elsewhere?)
  Eigen::Affine3d T_rt(Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitZ()));
  BOOST_FOREACH(const apriltags_ros::AprilTagDetection & tag, msg->detections) {
    if (reset_map) {    
      if (tag.id == MAP_ORIGIN_TAG_ID) {
      	Eigen::Affine3d T_to;
      	tf::poseMsgToEigen(tag.pose.pose, T_to);      
      	Tmap_root = (T_to * T_rt).inverse();
	reset_map = false;
	ROS_INFO("Established map origin");
      }
    }    
    if (pub_check[tag.id]) {
    if (turtlebot_behaviors::poses.find(tag.id) != turtlebot_behaviors::poses.end()) {
      Eigen::Affine3d T_to;
      tf::poseMsgToEigen(tag.pose.pose, T_to);
      if (reset_odom[tag.id]) {
	Eigen::Affine3d T_wo = T_to * T_rt;
	tag_origin[tag.id] = T_wo.inverse();
	reset_odom[tag.id] = false;
	Tod_or[tag.id] = Tod_bf[tag.id] * Tbf_tag[tag.id];
	ROS_INFO("Established robot origin for tag %d", tag.id);
	Tmap_od[tag.id] = Tmap_root * T_to * T_rt * Tod_or[tag.id].inverse();
      } 
      else {
        geometry_msgs::TransformStamped ts_mo, ts_ob;

        const Eigen::Affine3d & T_ow = tag_origin[tag.id];
        Eigen::Affine3d T_rw = T_ow * T_to * T_rt;
        turtlebot_behaviors::poses[tag.id] = T_rw;	
        Tod_bf[tag.id] = Tod_or[tag.id] * T_rw * Tbf_tag[tag.id].inverse();
 
	geometry_msgs::Transform tf_ob;
	tf::transformEigenToMsg(Tod_bf[tag.id], tf_ob);
        ts_ob.header.frame_id= "odom";
	ts_ob.header.stamp = ros::Time::now();
        ts_ob.child_frame_id = "base_footprint";
        ts_ob.transform = tf_ob;
	tf_ob_pubs[tag.id].publish(ts_ob);
	ROS_INFO_ONCE("Published odom to base footprint transform for tag %d", tag.id);

	geometry_msgs::Transform tf_mo;
	tf::transformEigenToMsg(Tmap_od[tag.id], tf_mo);
	ts_mo.header.frame_id= "map";
	ts_mo.header.stamp = ros::Time::now();
	ts_mo.child_frame_id = "odom";
	ts_mo.transform = tf_mo;
	tf_mo_pubs[tag.id].publish(ts_mo);
	ROS_INFO_ONCE("Published map to odom transform for tag %d", tag.id);
      }
    }  
    }
  }
}

void odomCallback(int rid, 
  const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg)
{
  tf::poseMsgToEigen(msg->pose.pose, turtlebot_behaviors::odoms[rid]);
  // XXX: Do something more useful with odometry data
}

void behaviorCallback(const std_msgs::String::ConstPtr & msg)
{
  current_behavior = msg->data;
  ROS_INFO("current_behavior: %s", current_behavior.c_str());
}

void leaderCallback(const std_msgs::String::ConstPtr & msg)
{
  current_leader = msg->data;
  ROS_INFO("current_leader: %s", current_leader.c_str());
}

void transformCallback(int rid, const geometry_msgs::TransformStamped::ConstPtr &msg)
{
  geometry_msgs::TransformStamped ts = *msg;
  tf::transformMsgToEigen(ts.transform, Tbf_tag[rid]);
  pub_check[rid] = true;
}

}

namespace turtlebot_behaviors {

std::map<int, Eigen::Affine3d> poses;
std::map<int, Eigen::Affine3d> odoms;
std::map<std::string, int> robot_ids ;

void publishCommand(int rid, double v, double w)
{
  const double MAX_LINEAR_VELOCITY = 0.5;
  const double MAX_ANGULAR_VELOCITY = M_PI/4.0;

  if (robot_ids.find(current_leader) != robot_ids.end()) {
    if (rid == robot_ids[current_leader]) { return; }
  }

  namespace ba = boost::algorithm;
  geometry_msgs::Twist::Ptr cmd(new geometry_msgs::Twist);
  cmd->linear.x = ba::clamp(v, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  cmd->angular.z = ba::clamp(w, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  cmd_pubs[rid].publish(cmd);
}

void moveRobotToPoint(int rid, const Eigen::Vector3d & point)
{
  const double GAIN_LINEAR = 0.2;
  const double GAIN_ANGULAR = 1.0;
  const double DISTANCE_TOLERANCE = 0.1;

  Eigen::Vector3d position = poses[rid].translation();
  Eigen::Vector3d bearing = poses[rid].rotation() * Eigen::Vector3d::UnitX();
  double yaw = std::atan2(bearing.y(), bearing.x());
  ROS_DEBUG("robot %d: %lf %lf %lf", rid, position.x(), position.y(), yaw);
  
  Eigen::Vector3d displacement = point - position;
  double target_yaw = std::atan2(displacement.y(), displacement.x());
  double v = GAIN_LINEAR * displacement.dot(bearing);
  double w = GAIN_ANGULAR * angles::shortest_angular_distance(yaw, target_yaw);
  v = ((displacement.norm() > DISTANCE_TOLERANCE) && (v > 0.0)) ? v : 0.0;

  publishCommand(rid, v, w); 
}

} // turtlebot_behaviors


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "swarm_controller");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Enabled behaviors
  behaviors["moveStop"] = turtlebot_behaviors::moveStop;
  behaviors["moveForward"] = turtlebot_behaviors::moveForward;
  behaviors["moveBackward"] = turtlebot_behaviors::moveBackward;
  behaviors["turnClockwise"] = turtlebot_behaviors::turnClockwise;
  behaviors["turnCounterclockwise"] = turtlebot_behaviors::turnCounterclockwise;
  behaviors["flocking"] = turtlebot_behaviors::flocking;
  behaviors["alignHeading"] = turtlebot_behaviors::alignHeading;
  behaviors["formationLineX"] = turtlebot_behaviors::formationLineX;
  behaviors["formationLineY"] = turtlebot_behaviors::formationLineY;
  behaviors["formationCircle"] = turtlebot_behaviors::formationCircle;
  

  /*change nh to pnh for using controller.launch!!!
***************************************************
  */ 
  // Get a list of available robots
  nh.getParam("robot_ids", turtlebot_behaviors::robot_ids);

  // Setup command publishers and odometry subscribers
  std::map<int, ros::Subscriber> odom_subs;
  std::map<int, ros::Subscriber> tf_subs;

  BOOST_FOREACH(const std::string & name, turtlebot_behaviors::robot_ids | boost::adaptors::map_keys) {
    int rid = turtlebot_behaviors::robot_ids[name];
    std::string pub_topic("/" + name + "/teleop/cmd_vel");
    std::string sub_topic("/" + name + "/teleop/odom_combined");

    //tf topics
    std::string tf_sub_topic("/" + name + "/transform_bf_p3");
    std::string tf_ob_pub_topic("/" + name + "/transform_odom_bf");
    std::string tf_mo_pub_topic("/" + name + "/transform_map_odom");

    turtlebot_behaviors::poses[rid] = Eigen::Affine3d::Identity();
    turtlebot_behaviors::odoms[rid] = Eigen::Affine3d::Identity();

    tag_origin[rid] = Eigen::Affine3d::Identity();
    Tod_bf[rid] = Eigen::Affine3d::Identity();
    reset_odom[rid] = true;
    pub_check[rid] = false;

    cmd_pubs[rid] = nh.advertise<geometry_msgs::Twist>(pub_topic, 1);
    odom_subs[rid] = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
      sub_topic, 1, boost::bind(odomCallback, rid, _1));

    //Setup tf publishers and subscribers
    tf_subs[rid] = nh.subscribe<geometry_msgs::TransformStamped>(
      tf_sub_topic, 1, boost::bind(transformCallback, rid, _1));
    tf_ob_pubs[rid] = nh.advertise<geometry_msgs::TransformStamped>(tf_ob_pub_topic, 1);
    tf_mo_pubs[rid] = nh.advertise<geometry_msgs::TransformStamped>(tf_mo_pub_topic, 1);
  }

  // Subscriber for tag detections 
  ros::Subscriber pose_sub = 
    nh.subscribe<apriltags_ros::AprilTagDetectionArray>(
      "tag_detections", 1, poseCallback);

  // Subscriber for current behavior
  ros::Subscriber behavior_sub = nh.subscribe<std_msgs::String>(
    "behavior", 1, behaviorCallback);

  // Subscriber for selected leader
  ros::Subscriber leader_sub = nh.subscribe<std_msgs::String>(
    "leader", 1, leaderCallback);

  
  ros::Rate rate(10);
  while (ros::ok()) {
    if (!reset_map) {
      if (behaviors.find(current_behavior) != behaviors.end()) { 
        behaviors[current_behavior]();
      } else {
        turtlebot_behaviors::moveStop(); 
      }
    }
    
    ros::spinOnce();
    rate.sleep();
  }
  
  return 0;
}

