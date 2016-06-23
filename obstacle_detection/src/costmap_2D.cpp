/*node to create occupancy map from the point cloud*/

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32.h>
#include <Eigen/Dense>
#include <nav_msgs/OccupancyGrid.h>
#include <boost/foreach.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_datatypes.h>
double al;  //alpha
double be;  // beta
double ga;  //gamma
double floor_threshold;
double ceil_threshold;
float resolution;
int min_cells;

class buildmap
{
  private: 
  
    ros::NodeHandle nh;
    float plane_coeffs[];
    ros::Subscriber cam_depth_pts_sub;
    //pcl::PointCloud<pcl::PointXYZ> unfiltered_cloud;
    std::vector<int> indices;
    pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
    ros::Publisher map_pub;
    ros::Publisher flag_pub;
    std_msgs::Int32 flag;
  
  public:
    buildmap(); 
    void load_params();
    void camera_cb(const sensor_msgs::PointCloud2::ConstPtr& msg);
};

/*function to calculate height from the ground plane*/
float calculate_height(float x, float y, float z)
{
  float sum_of_squares = sqrt(al*al + be*be);
  float height = ((al* x) + (be*y )+ z + ga);
  height = - height / sum_of_squares; //added minus sign
  return height;
}

buildmap::buildmap()
{
  load_params();  
  cam_depth_pts_sub = nh.subscribe("/camera/depth_registered/points", 1000, &buildmap::camera_cb, this);
  map_pub = nh. advertise<nav_msgs::OccupancyGrid>("/final_map", 1000);
  flag_pub = nh.advertise<std_msgs::Int32>("flag",1000);

}

/*call back function for point cloud;publishes the occupancy grid*/
void buildmap::camera_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  /*filtered point cloud from the ros message*/
  pcl::PCLPointCloud2::Ptr pts (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr pts_filtered (new pcl::PCLPointCloud2 ());
  pcl_conversions::toPCL(*msg,*pts);
  //throttle cloud
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (pts);
  sor.setLeafSize (0.1f, 0.1f, 0.1f);
  sor.filter (*pts_filtered);
  pcl::fromPCLPointCloud2(*pts_filtered,filtered_cloud);

  float h0 = calculate_height(0,0,0);
  Eigen::Vector3d n (al,be,1);
  Eigen::Vector3d c = - h0 * n;

  int cloud_height = filtered_cloud.height;
  int cloud_width = filtered_cloud.width;

  /*occupancy map parameters*/
  nav_msgs::OccupancyGrid finalmap;
  finalmap.header.frame_id = "camera_link";
  finalmap.info.width = 10/resolution;
  finalmap.info.height = 10/resolution;
  int map_height = finalmap.info.height;
  int map_width = finalmap.info.width;
  int total_cells = map_height * map_width;
  finalmap.data.resize(total_cells);
  finalmap.info.origin.position.x = 0;
  finalmap.info.origin.position.y = 0;
  finalmap.info.origin.position.z = 0;
  finalmap.info.origin.orientation.x = 0;
  finalmap.info.origin.orientation.y = 0;
  finalmap.info.origin.orientation.z = 0;
  finalmap.info.origin.orientation.w = 1;
  finalmap.info.resolution = resolution;

  //condition to avoid all invalid points when very close to obstacles
  if ((cloud_width * cloud_height) < min_cells)
    flag.data = 0;

  else
  {
    flag.data = 1;
    for (int iter = 0; iter < (cloud_width * cloud_height); iter++)
    {
      pcl::PointXYZ pt = filtered_cloud.points[iter]; 
      float x = pt.x;
      float y = pt.y; 
      float z = pt.z;
      float ht = calculate_height(x,y,z);
      if ( ( (al * x) + (be * y) + ga - z) != 0 )
      {
        if (ht > ceil_threshold && ht < floor_threshold )
        {
          Eigen::Vector3d p (x,y,z);
          Eigen::Vector3d o = p - (ht * n);
          Eigen::Vector3d d= c-o;
          float dl = d.norm();
          float theta = std::atan2(   (o(1)-c(1)) ,   (o(0) - c(0))  ); 
          //projecting points in the grid
          int X = std::floor((dl* cos(theta))/ resolution);
          int Y = std::floor((dl* sin(theta))/resolution);
          int X_map = Y ;//+ (finalmap.info.width/2) - 1;
          int Y_map = -X  + (map_width/2) - 1;
          int index = (Y_map * map_width) + X_map;
          if (X_map < map_height && Y_map < map_width && index < total_cells)
            finalmap.data[index]= 100;
        }
      }   
    }
  }
  flag_pub.publish(flag);
  map_pub.publish(finalmap);
}


void buildmap::load_params()
{
  nh.getParam("/costmap_2D/alpha", al);
  nh.getParam("/costmap_2D/beta",be);
  nh.getParam("/costmap_2D/gamma",ga);
  nh.getParam("/costmap_2D/floor_threshold", floor_threshold);
  nh.getParam("/costmap_2D/ceil_threshold",ceil_threshold);
  nh.getParam("/costmap_2D/resolution", resolution);
  nh.getParam("/costmap_2D/min_cells", min_cells);
  ROS_INFO_STREAM("loaded params in costmap");
   
}



int main (int argc, char **argv)
{
 ros::init(argc,argv,"costmap_2D"); 
 buildmap Co; 
 ros::Rate r(10);
 while (ros::ok())
 {
  ros::spin();
  r.sleep();
 } 
}

