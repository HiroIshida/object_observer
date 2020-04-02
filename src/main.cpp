#include "ros/ros.h"
#include "process.hpp"

#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <pcl_conversions/pcl_conversions.h> // japanese version

#include <object_observer/Cloud.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <geometry_msgs/Point32.h>

void callback (const sensor_msgs::PointCloud2ConstPtr& msg_input)
{
  // http://docs.pointclouds.org/1.7.2/a01420.html#a89aca82e188e18a7c9a71324e9610ec9
  // tutorial in Japanese is wrong (using depricated header)  
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg_input, cloud); 

  std_msgs::Float32MultiArray x_array, y_array, z_array;
  for(int i=0; i< cloud.points.size(); i++){
    x_array.data[i] = cloud.points[i].x;
    y_array.data[i] = cloud.points[i].y;
    z_array.data[i] = cloud.points[i].z;
  }
  object_observer::Cloud msg;
  msg.x_array = x_array;
  msg.y_array = y_array;
  msg.z_array = z_array;
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "projector");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/core/multi_plane_extraction/output_nonplane_cloud", 10, callback);
  ros::spin();
}


