#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h> 
#include <std_msgs/Int32.h>

class laser_scan
{
	ros::NodeHandle nh;
	ros::Subscriber scan_sub;
	ros::Publisher map_pub;
	ros::Publisher flag_pub;
	sensor_msgs::LaserScan scan;
	std_msgs::Int32 flag;
	int height, width;
	float resolution;

public:
	laser_scan()
	{
		load_params();
		scan_sub = nh.subscribe("/scan", 1000, &laser_scan::scan_callback, this);
		map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/final_map", 1000);
		flag_pub = nh.advertise<std_msgs::Int32>("flag",1000);
	}

	void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
	{
		scan = *msg;
		//std::cout<<scan.ranges.size()<<std::endl;
		//occupancy grid parameters
		nav_msgs::OccupancyGrid finalmap;
		finalmap.header.frame_id = scan.header.frame_id;
  		finalmap.info.width = width/resolution;
  		finalmap.info.height = height/resolution;
  		int map_height = finalmap.info.height;
  		int map_width = finalmap.info.width;
		int total_cells = map_height * map_width;
		finalmap.data.resize(total_cells);
		//std::cout<<total_cells;
		finalmap.info.origin.position.x = 0;
		finalmap.info.origin.position.y = 0;
		finalmap.info.origin.position.z = 0;
		finalmap.info.origin.orientation.x = 0;
		finalmap.info.origin.orientation.y = 0;
		finalmap.info.origin.orientation.z = 0;
		finalmap.info.origin.orientation.w = 1;
		finalmap.info.resolution = resolution;
		int nan_count = 0;
		for (int i = 0; i < scan.ranges.size(); i++)
		{
			if (!(std::isnan(scan.ranges[i])))
			{	
			int X_map = std::floor(scan.ranges[i]);
			int Y_map = i;
			int index = (Y_map * map_width) + X_map;
			finalmap.data[index] = 100;
			//std::cout<<"haha";

			}
			else
				nan_count++;
		}
		//std::cout<<scan.ranges.size()<<std::endl;
		if (nan_count>(0.6*scan.ranges.size()))
			flag.data = 0;
		else
			flag.data = 1;
		flag_pub.publish(flag);
		map_pub.publish(finalmap);
	}

	void load_params()
	{
		//nh.getParam("/laser_scan/height", height);
		//nh.getParam("/laser_scan/width", width);
		//nh.getParam("/laser_scan/resolution", resolution);
		width = 7;
		height = 7;
		resolution = 0.01;
		ROS_INFO_STREAM("loaded params in laser_scan");
	}

	float occupancy_grid_mapping(float l_prev, float z)
	{
		
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_scan");
	laser_scan laserscan; 
	while (ros::ok())
		ros::spin();
}