#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>
#include <Eigen/Dense>
#include <nav_msgs/OccupancyGrid.h>
#include <boost/foreach.hpp>
#include <math.h>
double al;  //alpha
double be;  // beta
double ga;  //gamma
double floor_threshold;
double ceil_threshold;
float resolution;
nav_msgs::OccupancyGrid GridMap;

class costmap_2D
{
private: 
	
		ros::NodeHandle nh;
		float plane_coeffs[];
		ros::Subscriber cam_depth_pts_sub;
		//pcl::PointCloud<pcl::PointXYZ> unfiltered_cloud;
		pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
	
public:
	costmap_2D();
	void load_params();
	void camera_cb(const sensor_msgs::PointCloud2::ConstPtr& msg);
	
};
float calculate_height(float x, float y, float z)
{
  float sum_of_squares = sqrt(al*al + be*be);
  float height = ((al* x) + (be*y )+ z + ga);
  height = height / sum_of_squares;
  return height;
}
costmap_2D::costmap_2D()
{
  load_params();	
  cam_depth_pts_sub = nh.subscribe("/filtered_cloud", 1000, &costmap_2D::camera_cb, this);

}
void costmap_2D::camera_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::PCLPointCloud2 pts;

  pcl_conversions::toPCL(*msg,pts);
  pcl::fromPCLPointCloud2(pts,filtered_cloud);
  BOOST_FOREACH(const pcl::PointXYZ & it , filtered_cloud.points)
  {
     float x = it.x;
     float y = it.y;
     float z = it.z;
     float ht = calculate_height(x,y,z);
     float h0 = calculate_height(0,0,0);
     if ( ( (al * x) + (be * y) + ga - z) != 0 )
     {
      if (ht < ceil_threshold & ht > floor_threshold )
      {
        Eigen::Vector3d p (x,y,z);
        Eigen::Vector3d n (al,be,1);
        Eigen::Vector3d o = p - (ht * n);
        Eigen::Vector3d c = - h0 * n;
        Eigen::Vector3d d= c-o;
        float dl = d.norm();
        float theta = std::atan2(   (o(1)-c(1)) ,   (o(0) - c(0))  );
        int X = floor((dl* cos(theta))/ resolution);
        int Y = floor((dl* sin(theta))/resolution);
        //ROS_INFO("%d,%d\n",X,Y);
        
     } 
    }     
  }
}


void costmap_2D::load_params()
{
   nh.getParam("/costmap_2D/alpha", al);
   nh.getParam("/costmap_2D/beta",be);
   nh.getParam("/costmap_2D/gamma",ga);
   nh.getParam("/costmap_2D/floor_threshold", floor_threshold);
   nh.getParam("/costmap_2D/ceil_threshold",ceil_threshold);
   nh.getParam("/costmap_2D/resolution", resolution);
   ROS_INFO_STREAM("loaded params in costmap");
}



int main (int argc, char **argv)
{
ros::init(argc,argv,"costmap_2D"); 
costmap_2D Co;
while (ros::ok())
  ros::spin();	
}
