#include "ros/ros.h"
#include "process.hpp"

#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <pcl_conversions/pcl_conversions.h> // japanese version

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "geometry_msgs/Point.h" 
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

void callback (const sensor_msgs::PointCloud2ConstPtr& msg_input)
{
  // http://docs.pointclouds.org/1.7.2/a01420.html#a89aca82e188e18a7c9a71324e9610ec9
  // tutorial in Japanese is wrong (using depricated header)  
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg_input, cloud); 
  std::cout<<"hoge"<<std::endl;
}

int main (int argc, char** argv)
{
  tmp();
  int i = 1;
  ros::init(argc, argv, "projector");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/core/multi_plane_extraction/output_nonplane_cloud", 10, callback);
  ros::spin();
}


