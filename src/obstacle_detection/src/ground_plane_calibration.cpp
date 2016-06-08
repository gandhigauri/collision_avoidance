#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/foreach.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

//typedef pcl::PointXYZ pcl_type;

class ground_plane_calibration
{
	ros::NodeHandle nh;
	ros::Subscriber sub;
	ros::Publisher pub;
	sensor_msgs::PointCloud2 pcl_msg;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PCLPointCloud2 pts;
public:
	ground_plane_calibration()
	{
		sub = nh.subscribe("/camera/depth_registered/points", 1000, &ground_plane_calibration::points_callback, this);
		//pub = nh.advertise<pcl_type>("ground_plane", 1000);
	}
	void points_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{
		pcl_msg = *msg;
  		pcl_conversions::toPCL(pcl_msg,pts);
  		pcl::fromPCLPointCloud2(pts,cloud);
		BOOST_FOREACH (const pcl::PointXYZ& pt, cloud.points)
			std::cout<<	pt.x << " , "<< pt.y <<" , "<< pt.z << std::endl;
	}
};

int main(int argc, char **argv)
{
	ros::init(argc,argv,"ground_plane_calibration");
	ground_plane_calibration gp_calib;
	ros::spin();
	return 0;
}