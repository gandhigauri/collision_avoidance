#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <vector>

class filter_cloud
{
	ros::NodeHandle nh;
	ros::Subscriber pointcloud_sub;
	ros::Subscriber filterflag_sub;
	ros::Publisher pointcloud_pub;
	//ros::Publisher velocity_pub;
	sensor_msgs::PointCloud2 sensor_msg;
	sensor_msgs::PointCloud2 published_msg;
	std_msgs::Int32 filter_flag;
	//geometry_msgs::Twist base_cmd;
	pcl::PointCloud<pcl::PointXYZRGB> unfiltered_cloud;
	pcl::PointCloud<pcl::PointXYZRGB> filtered_cloud;
	pcl::PCLPointCloud2 pts;
	bool flag_filter;
	float alpha, beta, gamma; 
public:
	filter_cloud()
	{
		load_params();
		pointcloud_sub = nh.subscribe("/camera/depth_registered/points", 1000, &filter_cloud::points_callback, this);
		filterflag_sub = nh.subscribe("/filter_flag", 1000, &filter_cloud::flag_callback, this);
		pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1000);
		//velocity_pub=nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1000);
	}
	void load_params()
	{
		nh.getParam("/filter_cloud/alpha", alpha);
   		nh.getParam("/filter_cloud/beta",beta);
   		nh.getParam("/filter_cloud/gamma",gamma);
   		ROS_INFO_STREAM("loaded params in filter cloud");
	}

	void flag_callback(const std_msgs::Int32::ConstPtr& msg)
	{
		filter_flag = *msg;
		if (filter_flag.data == 1)
			flag_filter = true;
		else
			flag_filter = false;

	}

	void points_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{
		//convert point cloud2 msg from sensor to pcl point cloud
		sensor_msg = *msg;
  		pcl_conversions::toPCL(sensor_msg,pts);
  		pcl::fromPCLPointCloud2(pts,unfiltered_cloud);
  		//filtered_cloud = unfiltered_cloud;
  		int cloud_width = unfiltered_cloud.width;
  		int cloud_height = unfiltered_cloud.height;
		/*int nanpts_unfiltered = 0;
  		for (int iter = 0; iter < (cloud_height*cloud_width); iter++)
  		{	
  			if (std::isnan(unfiltered_cloud.points[iter].x) && std::isnan(unfiltered_cloud.points[iter].y) && std::isnan(unfiltered_cloud.points[iter].z))
  			{
  				nanpts_unfiltered++;
  			}
  		}
  		if (nanpts_unfiltered > (0.4*cloud_height*cloud_width))
  		{
  			ROS_INFO_STREAM("more than 40 percent are invalid pts");
  			base_cmd.angular.z=0.10;
  			velocity_pub.publish(base_cmd);
  		}*/
  		if (1)
  		{
  			std::vector<int> indices;
  			pcl::removeNaNFromPointCloud(unfiltered_cloud, filtered_cloud, indices);
		
  			/*for (int iter = 0; iter < (cloud_height*cloud_width); iter++)
  			{	
  				if (std::isnan(unfiltered_cloud.points[iter].x))
  				{	
	  				std::vector<int> neighbors_indices;
	  				neighbors_indices = get_neighbor_indices(iter, cloud_width, cloud_height);
	  				for (int i =0 ; i < neighbors_indices.size(); i++)
	  				{
	  					pcl::PointXYZRGB neighbor_pt = unfiltered_cloud.points[neighbors_indices[i]];
	  					int *neighbor_coordinates;
	  					neighbor_coordinates = get_grid_coordinates(i, cloud_width, cloud_height);
	  					if (satisfies_ground_plane(neighbor_pt))
	  					{
	  						for (int j = 0; j <= neighbor_coordinates[0]; j++)
	  						{
	  							int col_coord[2] = {j, neighbor_coordinates[1]};
								int pos = get_point_index(col_coord, cloud_width, cloud_height);
	  							filtered_cloud.points[pos].x = neighbor_pt.x;
	  							filtered_cloud.points[pos].z = neighbor_pt.z;
	  							int sum = 0;
	  							int count = 0;
	  							for (int k = 0; k < cloud_width; k++)
	  							{
	  								int row_coord[2] = {j, k}; 
	  								float y_coord = unfiltered_cloud.points[get_point_index(row_coord, cloud_width, cloud_height)].y;
	  								if (!std::isnan(y_coord))
	  								{
	  									count++;
	  									sum = sum + y_coord;
	  								}
	  							}
	  							filtered_cloud.points[pos].y = sum/count;
	  						}
	  					}
	  				}
  				}
  			iter++;
  			}*/
  			pcl::toROSMsg(filtered_cloud, published_msg);
  			pointcloud_pub.publish(published_msg);
  		}
  	}

	int* get_grid_coordinates(int index, int width, int height)
	{
		int grid_coordinates[] = {std::floor(index/width), (index%width)};
		return grid_coordinates;
	}

	int get_point_index(int grid[], int width, int height)
	{
		return (width*grid[0] + grid[1]);
	}

	std::vector<int> get_neighbor_indices(int index, int width, int height)
	{
		std::vector<int> neighbor_indices;
		int *grid_coord;
	  	grid_coord = get_grid_coordinates(index, width, height);
	  	int neighbors_coord[8][2] = {
	  					{grid_coord[0]-1,grid_coord[1]-1},
	  					{grid_coord[0]-1,grid_coord[1]},
	  					{grid_coord[0]-1,grid_coord[1]+1},
	  					{grid_coord[0],grid_coord[1]-1},
	  					{grid_coord[0],grid_coord[1]+1},
	  					{grid_coord[0]+1,grid_coord[1]-1},
	  					{grid_coord[0]+1,grid_coord[1]},
	  					{grid_coord[0]+1,grid_coord[1]+1},
	  				};
	  	for (int i = 0; i < 8; i++)
	  	{
	  		int grid[2] = {neighbors_coord[i][0],neighbors_coord[i][1]};
	  		if (grid[0]>=0 && grid[0]<height && grid[1]>=0 && grid[1]<width)
				neighbor_indices.push_back(get_point_index(grid,width,height));
		}
	  	return neighbor_indices;
	}

	bool satisfies_ground_plane(pcl::PointXYZRGB pt)
	{
		if (!(std::isnan(pt.x) && std::isnan(pt.y) && std::isnan(pt.z)))
		{
			if ( ( (alpha * pt.x) + (beta * pt.y) + gamma - pt.z) == 0 )
				return true;
			else
				return false;
		}
	}

};

int main(int argc, char **argv)
{
	ros::init(argc,argv,"filter_cloud");
	filter_cloud filter_obj;
	//ros::Rate r(10);
	//while (ros::ok())
	//{
		ros::spin();
	//	r.sleep();
	//}
	//return 0;
}