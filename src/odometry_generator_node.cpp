/**
 * @file odom_generator_node.cpp
 * @brief Publish and broadcast odometry
 * @author Parker Lusk <plusk@mit.edu>
 * @date 5 May 2023
 */

#include <ros/ros.h>

#include "kmd_tools/odometry_generator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_generator");
  ros::NodeHandle nhtopics("");
  ros::NodeHandle nhparams("~");
  kmd_tools::OdometryGenerator node(nhtopics, nhparams);
  ros::spin();
  return 0;
}
