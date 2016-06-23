/*node for avoiding obstacles*/
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <limits>

float c1, c2, min_x, max_x, min_y, max_y;
bool flag_filter;
ros::Publisher velocity_pub;
int robot_frame_to_index(int x, int y, int width, int height);

void flag_callback(const std_msgs::Int32::ConstPtr& msg)
{
	std_msgs::Int32 filter_flag = *msg;
	if (filter_flag.data == 1)
		flag_filter = true;
	else
		flag_filter = false;

}

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	nav_msgs::OccupancyGrid occ_grid = *msg;
	geometry_msgs::Twist base_cmd;
	float resolution = occ_grid.info.resolution;
	int width = occ_grid.info.width;
	int height = occ_grid.info.height;
	float v,w;
	int sum_mxy = 0;
	int sum_xmxy = 0;
	int sum_ymxy = 0;
	float min_dist = std::numeric_limits<float>::max();
	//defining rectangular region in the grid
	float rect_region[] = {min_x/resolution, min_y/resolution, max_x/resolution, max_y/resolution};
	if (flag_filter)
	{
	for (int x = std::floor(rect_region[0]); x <= std::floor(rect_region[2]); x++)
	{
		for (int y = std::floor(rect_region[1]); y<= std::floor(rect_region[3]); y++)
		{
			int m_xy = occ_grid.data[robot_frame_to_index(x,y,width,height)]/100;
			float dist = (x*x) + (y*y);
			if ((m_xy != 0) && dist < min_dist)
				min_dist = dist;
			sum_mxy = sum_mxy + m_xy;
			sum_xmxy = sum_xmxy + x * m_xy;
			sum_ymxy = sum_ymxy + y * m_xy;
		}
	}
	v = c1 * resolution * min_dist;
	if (sum_mxy!=0)
	{
		int mu_x = sum_xmxy/sum_mxy;
		int mu_y = sum_ymxy/sum_mxy;
		w = c2 * std::atan2(mu_x, mu_y);
	}
	else
	{
		base_cmd.angular.z = 0.5;
		base_cmd.linear.x = -0.08;
	}
	base_cmd.angular.z = w;
	base_cmd.linear.x = v;
	}
	else
	{
		base_cmd.angular.z = 0.8;
		base_cmd.linear.x = 0;
	}
	velocity_pub.publish(base_cmd);
}

int robot_frame_to_index(int x, int y, int width, int height)
{
	//converting to map frame
	int X_map = y ;
    int Y_map = -x  + (width/2) - 1;
    //converting to index
    int index = (Y_map * width) + X_map;
    //std::cout<<index<<std::endl;
	return index;
}


int main(int argc, char **argv)
{
	ros::init(argc,argv,"obstacle_avoidance");
	ros::NodeHandle nh;
	nh.getParam("/obstacle_avoidance/linear_velocity_const", c1);
	nh.getParam("/obstacle_avoidance/angular_velocity_const",c2);
	nh.getParam("/obstacle_avoidance/min_x",min_x);
	nh.getParam("/obstacle_avoidance/min_y",min_y);
	nh.getParam("/obstacle_avoidance/max_x",max_x);
	nh.getParam("/obstacle_avoidance/max_y",max_y);
	ROS_INFO_STREAM("loaded params in obstacle avoidance");
	ros::Subscriber map_sub = nh.subscribe("/final_map", 1000, map_callback);
	ros::Subscriber flag_sub = nh.subscribe("/flag", 1000, flag_callback);
	velocity_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1000);
	ros::Rate r(10);
	while (ros::ok())
	{
		ros::spin();
		r.sleep();
	}
}