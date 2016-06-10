#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/foreach.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>

class filter_cloud
{
	ros::NodeHandle nh;
	ros::Subscriber pointcloud_sub;
	sensor_msgs::PointCloud2 sensor_msg;
	pcl::PointCloud<pcl::PointXYZ> unfiltered_cloud;
	pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
	pcl::PCLPointCloud2 pts;
public:
	filter_cloud()
	{
		pointcloud_sub = nh.subscribe("/camera/depth_registered/points", 1000, &filter_cloud::points_callback, this);
	}
	void points_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
		{
			
			//convert point cloud2 msg from sensor to pcl point cloud
			sensor_msg = *msg;
	  		pcl_conversions::toPCL(sensor_msg,pts);
	  		pcl::fromPCLPointCloud2(pts,unfiltered_cloud);
	  		//pcl::PointCloud<pcl::PointXYZ>::Ptr ptrCloud(&unfiltered_cloud);
	  		//pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
			//kdtree.setInputCloud(ptrCloud);
			//int K = 8;
			//std::vector<int> pointIdxNKNSearch(K);
			//std::vector<float> pointNKNSquaredDistance(K);
			int iter =0;
	  		BOOST_FOREACH (const pcl::PointXYZ& pt, unfiltered_cloud.points)
	  		{
	  			if (std::isnan(pt.x) && std::isnan(pt.y) && std::isnan(pt.z))
	  			{
	  				int *neighbors_index;
	  				neighbors_index = get_neighbor_indices(iter, unfiltered_cloud.width, unfiltered_cloud.height);
											  				

	  				//std::cout<<pt+1;
	  				//pcl::PointXYZ neighbors[kdtree.nearestKSearch (pt, K, pointIdxNKNSearch, pointNKNSquaredDistance)];
	  				//for (int i = 0; i < pointIdxNKNSearch.size (); ++i)
	  				//{	
	  				//	neighbors[i] = unfiltered_cloud.points[pointIdxNKNSearch[i]];
	  				//	std::cout<<neighbors[i];
	  				//}
	  			}
	  			iter++;
	  			if (iter==5)
	  				break; 
	  		}	
	  			//std::cout<<pt<<","<<pt.x<<","<<pt.y<<","<<pt.z<<std::endl;
	  		ros::shutdown();
	  	}

	int* get_grid_coordinates(int index, int width, int height)
	{
		static int grid_coordinates[] = {std::floor(index/width), (index%width)};
		return grid_coordinates;
	}

	int get_point_index(int *grid, int width, int height)
	{
		return (width*grid[0] + grid[1]);
	}

	int* get_neighbor_indices(int index, int width, int height)
	{
		static int neighbor_indices[8];
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
	  	int iter;
	  	BOOST_FOREACH (int* grid,neighbors_coord)
	  	{
	  		neighbor_indices[iter] = get_point_index(grid,width,height);
	  		iter++;
	  	}
	  	return neighbor_indices;
	}

};

int main(int argc, char **argv)
{
	ros::init(argc,argv,"filter_cloud");
	filter_cloud filter_obj;
	while (ros::ok())
		ros::spin();
	return 0;
}