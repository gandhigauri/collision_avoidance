#include <ros/ros.h>
#include <cstdio>
#include <sensor_msgs/PointCloud2.h>
#include <boost/foreach.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
//#include <std_msgs/Float32MultiArray.h>
#include <Eigen/Dense>
#include <vector>

//typedef pcl::PointXYZ pcl_type;

class ground_plane_calibration
{
	ros::NodeHandle nh;
	ros::Subscriber pointcloud_sub;
	//ros::Publisher groundplane_pub;
	sensor_msgs::PointCloud2 sensor_msg;
	pcl::PointCloud<pcl::PointXYZ> unfiltered_cloud;
	pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
	pcl::PCLPointCloud2 pts;
	//std_msgs::Float32MultiArray coeff_msg;
public:
	ground_plane_calibration()
	{
		pointcloud_sub = nh.subscribe("/camera/depth_registered/points", 1000, &ground_plane_calibration::points_callback, this);
		//groundplane_pub = nh.advertise<std_msgs::Float32MultiArray>("ground_plane", 1000);
	}
	void points_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{
		
		//convert point cloud2 msg from sensor to pcl point cloud
		sensor_msg = *msg;
  		pcl_conversions::toPCL(sensor_msg,pts);
  		pcl::fromPCLPointCloud2(pts,unfiltered_cloud);

  		//filter the cloud by removing NaN values
  		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(unfiltered_cloud, filtered_cloud, indices);
		
		//create the matrix equation using all points
  		int num_points = indices.size();
  		Eigen::MatrixXf A(num_points,3);
  		Eigen::MatrixXf b(num_points,1);
  		Eigen::VectorXf coeff_vector(3,1);
  		int iter = 0;
  		BOOST_FOREACH (const pcl::PointXYZ& pt, filtered_cloud.points)
		{
			A(iter,0) = -pt.x;
			A(iter,1) = -pt.y;
			A(iter,2) = -1;
			b(iter,0) = pt.z;
			iter++;
		}

		//solve Ax = b
		coeff_vector = A.colPivHouseholderQr().solve(b);

		//publish the coefficient vector in a multiarray of Float type
		//coeff_msg.data.clear();
		//for (float i = 0; i < coeff_vector.size(); i++)
		//{
		//	coeff_msg.data.push_back(coeff_vector(i));
		//}
		//groundplane_pub.publish(coeff_msg);	
		ROS_INFO_STREAM("Computed ground plane parameters");

		//write to yaml file
		std::string groundplanedatafile = "src/obstacle_detection/ground_plane.yaml";
      	ROS_INFO("Writing ground plane data to %s", groundplanedatafile.c_str());
      	FILE* yaml = fopen(groundplanedatafile.c_str(), "w");
    	fprintf(yaml, "alpha: %f\nbeta: %f\ngamma: %f\n", coeff_vector(0), coeff_vector(1), coeff_vector(2));
		fclose(yaml);

		//stop publishing after calibration and writing to yaml file
		ros::shutdown();
		
	}
};

int main(int argc, char **argv)
{
	ros::init(argc,argv,"ground_plane_calibration");
	ground_plane_calibration gp_calib;
	while (ros::ok())
		ros::spin();
	return 0;
}