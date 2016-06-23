/*node to avoid close obstacles*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include "geometry_msgs/Twist.h"
#include <std_msgs/Int32.h>
#include <iostream>

ros::Publisher filter_flag;
ros::Publisher velocity_pub;

void points_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	sensor_msgs::PointCloud2 sensor_msg = *msg;
	geometry_msgs::Twist base_cmd;
	std_msgs::Int32 flag_pub;
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::PCLPointCloud2 pts;

  //convert point cloud2 msg from sensor to pcl point cloud
	pcl_conversions::toPCL(sensor_msg,pts);
	pcl::fromPCLPointCloud2(pts, cloud);
	int cloud_width = cloud.width;
	int cloud_height = cloud.height;
	int nanpts_unfiltered = 0;
		for (int iter = 0; iter < (cloud_height*cloud_width); iter++)
		{	
			if (std::isnan(cloud.points[iter].x) && std::isnan(cloud.points[iter].y) && std::isnan(cloud.points[iter].z))
			{
				nanpts_unfiltered++;
			}
		}
    //check if more than 40% of points are NaN
		if (nanpts_unfiltered > (0.4*cloud_height*cloud_width))
		{
			ROS_INFO_STREAM("more than 40 percent are invalid pts");
			base_cmd.angular.z=0.10;
			flag_pub.data = 0;
			velocity_pub.publish(base_cmd);
			filter_flag.publish(flag_pub);
		}
		else
		{
			flag_pub.data = 1;
			filter_flag.publish(flag_pub);

		}
	}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"close_obstacle");
	ros::NodeHandle nh;
	ros::Subscriber pointcloud_sub = nh.subscribe("/camera/depth_registered/points", 1000, points_callback);
	filter_flag = nh.advertise<std_msgs::Int32>("filter_flag",1000);
	velocity_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1000);
  ros::spin();
}
