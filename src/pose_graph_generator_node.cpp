/**
 * @file pose_graph_generator_node.cpp
 * @brief Build pose graph from VIO log odometry
 * @author Parker Lusk <plusk@mit.edu>
 * @date 7 May 2023
 */

#include <ros/ros.h>

#include "kmd_tools/pose_graph_generator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_graph_generator");
  ros::NodeHandle nhtopics("");
  ros::NodeHandle nhparams("~");
  kmd_tools::PoseGraphGenerator node(nhtopics, nhparams);
  ros::spin();
  return 0;
}
