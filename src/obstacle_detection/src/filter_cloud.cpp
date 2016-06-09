#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/foreach.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>

class filter_cloud
{
public:
	filter_cloud()
	{
		pointcloud_sub = nh.subscribe("/camera/depth_registered/points", 1000, &filter_cloud::points_callback, this);
	}

	/* data */
};