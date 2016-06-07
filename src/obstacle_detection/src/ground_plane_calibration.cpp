#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/foreach.hpp>
#include <pcl/point_types.h>

class ground_plane_calibration
{
	ros::NodeHandle nh;
	ros::Subscriber sub;
	ros::Publisher pub;
	int check = 0;
public:
	ground_plane_calibration()
	{
		sub = nh.subscribe("/camera/depth_registered/points", 1000, &ground_plane_calibration::points_callback, this);
		pub = nh.advertise<sensor_msgs::PointCloud2>("ground_plane", 1000);
	}
	void points_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{
		check = 1;
		pub.publish(*msg);
		//BOOST_FOREACH (const pcl::PointXYZ& pt, msg->data)
		//	std::cout<<"gg";//"\t(%f, %f, %f)\n"<<	pt.x << pt.y << pt.z;
	}
};

int main(int argc, char **argv)
{
	ros::init(argc,argv,"ground_plane_calibration");
	ground_plane_calibration gp_calib;
	ros::spin();
	return 0;
}