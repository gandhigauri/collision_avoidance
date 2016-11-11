# collision_avoidance
Multi-robot localization and collision avoidance

Package: obstacle_detection
This package implements obstacle detection and avoidance using the point cloud data from the kinect by creating a 2D occupancy grid map and updating the velocities in the map.
It has the following files:
1. ground_plane_calibration : This file contains the code for the calibration step by estimating the ground plane from the point cloud data.
2. filter_cloud : This file implements the infinite pole approach for determining reflective obstacles.
3. close_obstacle : This file deals with the obstacles very close to the sensor.
4. costmap_2D : This file creates the 2D occupancy grid map from the filtered point cloud data.
5. obstacle_avoidance : This node geometrically finds out the linear and angular velocities from the 2D map.
