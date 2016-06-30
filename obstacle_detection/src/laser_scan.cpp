#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h> 

class laser_scan
{
	ros::NodeHandle nh;
	ros::Subscriber scan_sub;
	ros::Publisher map_pub;
	sensor_msgs::
	nav_msgs::OccupancyGrid finalmap;

public:
	laser_scan()
	{
		load_params();
		scan_sub = nh.subscribe("/scan", 1000, &laser_scan::scan_callback, this);
		map_pub = nh.publish<nav_msgs::OccupancyGrid>("/final_map", 1000);
	}

	void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
	{

	}

	void load_params()
	{
		nh.getParam("/laser_scan/resolution", resolution);
		ROS_INFO_STREAM("loaded params in laser_scan");
	}
};

int main(int argc, char **argv)
{
	ros::init(argv, argc, "laser_scan");
	laser_scan laserscan; 
	ros::spin();
}