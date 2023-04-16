/**
 * @file pose_graph_generator.h
 * @brief Build pose graph from VIO log odometry
 * @author Parker Lusk <plusk@mit.edu>
 * @date 7 May 2023
 */

#pragma once

#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>

#include <nav_msgs/Odometry.h>
#include <pose_graph_tools/PoseGraph.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "kmd_tools/pose_graph_defines.h"

namespace kmd_tools {

  class PoseGraphGenerator
  {
  public:
    enum class LookaheadMethod { gtInterp, kfInterp, kfExtrap, gtNextKF };
    enum class TFOdometryBroadcaster { gt, violog };

    struct Params {
      bool connect_map_frame_using_gt = true; ///< Connect tf tree map --> odom using ground truth
      TFOdometryBroadcaster tf_odom_broadcaster = TFOdometryBroadcaster::gt; ///< for connecting odom --> base_link
      bool pub_ground_truth = true; ///< should ground truth be published
      bool pub_violog = false; ///< should estimated VIO odom be published
      bool pub_at_kf_times = false; ///< pub odometry msg with each keyframe

      LookaheadMethod lookahead_method_gt = LookaheadMethod::gtInterp;
      LookaheadMethod lookahead_method_violog = LookaheadMethod::kfExtrap;
    };

  public:
    PoseGraphGenerator(const ros::NodeHandle& nh, const ros::NodeHandle& nhp);
    ~PoseGraphGenerator() = default;

  private:
    ros::NodeHandle nh_, nhp_;
    ros::Subscriber sub_odom_;
    ros::Publisher pub_posegraph_, pub_posegraph_incremental_;

    ///\brief Dataset
    int robot_id_;
    std::string robot_;
    std::string logdir_;
    std::filesystem::path logpath_;

    std::map<uint64_t, KeyframePtr> keyframes_;
    std::unordered_map<size_t, EdgePtr> odometry_;
    std::unordered_map<size_t, EdgePtr> privateLoopClosures_;
    std::unordered_map<size_t, EdgePtr> publicLoopClosures_;

    uint64_t t_ns_last_; ///< time of last keyframe that was published

    bool get_robot_name_and_id();

    /**
     * @brief      Loads pose graph measurements from VIO log.
     *
     * @param[in]  logdir    Kimera Multi Distributed log directory
     * @param[in]  robot_id  Robot ID
     *
     * @return     keyframes (keyed by t_ns),
     *             odom edges (keyed by src kf id),
     *             private loop closures (keyed by src kf id),
     *             public loop closures (keyed by src kf id)
     */
    std::tuple<std::map<uint64_t, KeyframePtr>,
      std::unordered_map<size_t, EdgePtr>,
      std::unordered_map<size_t, EdgePtr>,
      std::unordered_map<size_t, EdgePtr>>
    read_pose_graph(const std::filesystem::path& logdir, const int robot_id);

    pose_graph_tools::PoseGraphPtr
    build_pose_graph(uint64_t start_ns, uint64_t end_ns);

    void odom_cb(const nav_msgs::OdometryConstPtr& msg);
    
  };

} // ns kmd_tools
