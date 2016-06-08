#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>

float alpha=0;
float beta=0;
float delta=0;
class costmap_2D
{
private: 
	
		ros::NodeHandle nh;
		float plane_coeffs[];
		float resolution;
		ros::Subscriber plane_coeff_sub;
		ros::Subscriber cam_depth_pts_sub;
	
public:
	costmap_2D();
	~costmap_2D();
	void plane_coeff_cb(const std_msgs::Float32MultiArray::ConstPtr& plane_coeffs);
	void camera_cb(const sensor_msgs::PointCloud2::ConstPtr& msg);
	
};
costmap_2D::costmap_2D()
{
  cam_depth_pts_sub = nh.subscribe("/camera/depth_registered/points", 1000, &costmap_2D::camera_cb, this);
  plane_coeff_sub = nh.subscribe("ground_plane",10,&costmap_2D::plane_coeff_cb, this);

}
void costmap_2D::camera_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PCLPointCloud2 pts;
  pcl_conversions::toPCL(*msg,pts);
  pcl::fromPCLPointCloud2(pts,cloud);

}


void costmap_2D::plane_coeff_cb(const std_msgs::Float32MultiArray::ConstPtr& msg2)
{
 std::vector<float>::const_iterator it = msg2->data.begin();
 alpha = *it;
 it++;
 beta = *it;
 it++;
 delta = *it;
}



int main (int argc, char **argv)
{
ros::init(argc,argv,"costmap_2D");

}
