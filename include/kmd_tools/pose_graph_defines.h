/**
 * @file pose_graph_defines.h
 * @brief Pose graph definitions, e.g., Keyframe and Edge
 * @author Parker Lusk <plusk@mit.edu>
 * @date 7 May 2023
 */

#pragma once

#include <cstdint>
#include <memory>
#include <tuple>

#include <Eigen/Geometry>

namespace kmd_tools {

  struct Keyframe
  {
    uint64_t t_ns; ///< timestamp [ns]
    size_t kid; ///< keyframe id
    size_t sid; ///< submap id
    //\brief Data from VIO log
    Eigen::Affine3d T_submap_kf; ///< keyframe pose w.r.t submap
    Eigen::Affine3d T_viodom_kf; ///< keyframe pose w.r.t viodom (PG gen only)
    Eigen::Affine3d T_viodom_baselink; ///< baselink (not cam) w.r.t viodom
    //\brief Uses ground truth to define odom w.r.t viodom frame
    Eigen::Affine3d T_odom_baselink; ///< baselink (not cam) w.r.t odom frame
    //\brief Data from ground truth (LOCUS, lidar odometry and mapping)
    Eigen::Affine3d Tgt_odom_baselink; ///< baselink w.r.t odom frame
    Eigen::Affine3d Tgt_map_baselink; ///< baselink w.r.t map frame (raw gt)

    bool operator<(const Keyframe& other)
    {
      return kid < other.kid;
    }
  };
  using KeyframePtr = std::shared_ptr<Keyframe>;


  struct Edge
  {
    size_t r1; ///< source robot id
    size_t s1; ///< source submap pose id
    size_t k1; ///< source keyframe id
    size_t r2; ///< dest robot id
    size_t s2; ///< dest submap pose id
    size_t k2; ///< dest keyframe id
    Eigen::Affine3d T; ///< relative pose

    bool operator<(const Edge& other)
    {
      return std::tie(r1, r2, s1, s2)
              < std::tie(other.r1, other.r2, other.s1, other.s2);
    }
  };
  using EdgePtr = std::shared_ptr<Edge>;

} // ns kmd_tools
