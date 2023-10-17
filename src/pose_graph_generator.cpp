/**
 * @file pose_graph_generator.cpp
 * @brief Build pose graph from VIO log odometry
 * @author Parker Lusk <plusk@mit.edu>
 * @date 7 May 2023
 */

#include <iostream>
#include <iomanip>

#include "csv/csv.h"

#include "kmd_tools/pose_graph_generator.h"
#include "kmd_tools/utils.h"

namespace kmd_tools {

namespace fs = std::filesystem;

PoseGraphGenerator::PoseGraphGenerator(const ros::NodeHandle& nh, const ros::NodeHandle& nhp)
: nh_(nh), nhp_(nhp)
{
  if (!get_robot_name_and_id()) {
    ros::shutdown();
    return;
  }
  ROS_INFO_STREAM("Robot " << robot_id_ << ": " << robot_);

  if (!nhp_.getParam("logdir", logdir_)) {
    ROS_FATAL("Must specify log directory containing Kimera-Multi outputs in param `logdir`");
    ros::shutdown();
    return;
  }

  logpath_ = logdir_;
  if (!fs::exists(logpath_)) {
    ROS_FATAL_STREAM("Kimera-Multi output log directory `" << logdir_ << "` does not exist");
    ros::shutdown();
    return;
  }

  //
  // Initialization
  //

  std::tie(keyframes_, odometry_, privateLoopClosures_, publicLoopClosures_) =
      read_pose_graph(logpath_, robot_id_);

  pub_posegraph_ = nh_.advertise<pose_graph_tools::PoseGraph>("pose_graph", 1);
  pub_posegraph_incremental_ = nh_.advertise<pose_graph_tools::PoseGraph>("pose_graph_incremental", 1);
  sub_odom_ = nh_.subscribe("kimera_vio_ros/odom", 1, &PoseGraphGenerator::odom_cb, this);
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

bool PoseGraphGenerator::get_robot_name_and_id()
{
  // Check namespace to find name of robot -- leverage fs module
  fs::path ns(ros::this_node::getNamespace());
  robot_ = ns.filename();
  if (robot_.empty()) {
    ROS_FATAL("Required robot namespace is missing. Hint: use launch file "
              "with node namespacing or `rosrun ... __ns:=robot_name`\n");
    return false;
  }

  std::map<std::string, std::string> robot_names;
  nh_.getParam("/robot_names", robot_names);
  robot_id_ = -1;
  for (const auto& [id, name] : robot_names) {
    if (name == robot_) {
      robot_id_ = std::stoi(id);
      break;
    }
  }
  if (robot_id_ == -1) return false;

  return true;
}

// ----------------------------------------------------------------------------


std::tuple<std::map<uint64_t, KeyframePtr>,
  std::unordered_map<size_t, EdgePtr>,
  std::unordered_map<size_t, EdgePtr>,
  std::unordered_map<size_t, EdgePtr>>
PoseGraphGenerator::read_pose_graph(const fs::path& logdir, const int robot_id)
{
  const fs::path path = logdir
                     / ("robot" + std::to_string(robot_id));

  //
  // Read keyframes and their poses relative to their corresponding submap
  //

  std::unordered_map<size_t, std::vector<KeyframePtr>> keyframesOfSubmap;

  {
    const fs::path csv = path / "kimera_distributed_keyframes.csv";
    io::CSVReader<10> in(csv.string());
    in.read_header(io::ignore_extra_column, "keyframe_stamp_ns", "keyframe_id",
                    "submap_id", "qx", "qy", "qz", "qw", "tx", "ty", "tz");
    uint64_t t_ns;
    size_t kid, sid;
    double qx, qy, qz, qw, tx, ty, tz;
    while (in.read_row(t_ns, kid, sid, qx, qy, qz, qw, tx, ty, tz)) {
      Eigen::Affine3d T_submap_kf = Eigen::Affine3d::Identity();
      T_submap_kf.translation() = Eigen::Vector3d(tx, ty, tz);
      T_submap_kf.linear() = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();

      // TODO(plusk): fix(?) kimera distributed saving of keyframes so that
      // the keyframe 0 and keyframe 1 DO NOT have the same timestamp
      if (kid == 0) t_ns -= 300000000; // 0.3 second previous <-- total hack
      // without this hack, the 0th keyframe gets overwritten by the 1st one.

      keyframesOfSubmap[sid].emplace_back(new Keyframe{t_ns, kid, sid, T_submap_kf});
    }
  }

  // Sort keyframes belonging to each submap by keyframe id.
  // This allows quick identification of the submap's representative keyframe,
  // i.e., the first keyframe of the submap.
  for (auto&& [sid, kfs] : keyframesOfSubmap) {
    std::sort(kfs.begin(), kfs.end());
  }

  //
  // Read submaps and their poses relative to the viodom frame
  //

  std::map<uint64_t, KeyframePtr> keyframes; // keyed by keyframe time

  {
    const fs::path csv = path / "kimera_distributed_submaps.csv";
    io::CSVReader<9> in(csv.string());
    in.read_header(io::ignore_extra_column, "submap_stamp_ns", "submap_id",
                    "qx", "qy", "qz", "qw", "tx", "ty", "tz");
    uint64_t t_ns;
    size_t sid;
    double qx, qy, qz, qw, tx, ty, tz;
    while (in.read_row(t_ns, sid, qx, qy, qz, qw, tx, ty, tz)) {
      Eigen::Affine3d T_viodom_submap = Eigen::Affine3d::Identity();
      T_viodom_submap.translation() = Eigen::Vector3d(tx, ty, tz);
      T_viodom_submap.linear() = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
      // process each of the keyframes associated with this submap
      for (const auto& kf : keyframesOfSubmap[sid]) {
        keyframes[kf->t_ns] = kf;
        // transform {kf w.r.t submap} to {kf w.r.t odom}
        keyframes[kf->t_ns]->T_viodom_kf = T_viodom_submap * kf->T_submap_kf;
      }
    }
  }

  //
  // Read pose graph measurement edges
  //

  std::vector<EdgePtr> edges;

  {
    const fs::path csv = path / "measurements.csv";
    io::CSVReader<15> in(csv.string());
    in.read_header(io::ignore_extra_column, "robot_src", "pose_src",
                    "robot_dst", "pose_dst", "qx", "qy", "qz", "qw",
                    "tx", "ty", "tz", "kappa", "tau",
                    "is_known_inlier", "weight");
    size_t r1, s1, r2, s2;
    double qx, qy, qz, qw, tx, ty, tz, tmp;
    while (in.read_row(r1, s1, r2, s2, qx, qy, qz, qw,
                        tx, ty, tz, tmp, tmp, tmp, tmp)) {
      // only care about edges that start from this robot's pose graph
      if (r1 != robot_id) continue;

      // private measurements involve only one robot
      const bool privateMeas = r1 == r2;

      // Since the submap's vector of keyframes is sorted by kid, we can
      // quickly find the submap's representative (i.e., the first keyframe).
      const size_t k1 = keyframesOfSubmap[s1].front()->kid;
      const size_t k2 = (privateMeas) ? keyframesOfSubmap[s2].front()->kid
                                      : std::numeric_limits<size_t>::max();

      Eigen::Affine3d T_src_dst = Eigen::Affine3d::Identity();
      T_src_dst.translation() = Eigen::Vector3d(tx, ty, tz);
      T_src_dst.linear() = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
      edges.emplace_back(new Edge{r1, s1, k1, r2, s2, k2, T_src_dst});
    }
  }

  // sort edges so that odometry can be chained
  std::sort(edges.begin(), edges.end());

  //
  // Build nodes of pose graph (keyframes), with private and public LCs
  //

  std::unordered_map<size_t, EdgePtr> privateLoopClosures; // keyed by kid
  std::unordered_map<size_t, EdgePtr> publicLoopClosures; // keyed by kid

  for (const auto& e : edges) {
    if (e->r1 == e->r2) {
      // private measurement
      if (e->s1 + 1 == e->s2) {
        // odometry
      } else {
        privateLoopClosures[e->k2] = e; // k2 > k1
      }
    } else {
      publicLoopClosures[e->k2] = e; // k2 > k1
    }
  }

  //
  // Build odometry edges from keyframes (since measurements uses submaps)
  //

  std::unordered_map<size_t, EdgePtr> odometry; // keyed by kid of y; x --> y

  for (auto kit = keyframes.begin(); kit != keyframes.end(); kit++) {
    const auto& kf = kit->second;
    if (kit != keyframes.begin()) {
      const auto& prevkf = std::prev(kit)->second;
      const Eigen::Affine3d T_src_dst = prevkf->T_viodom_kf.inverse() * kf->T_viodom_kf;
      odometry[kf->kid] = std::make_shared<Edge>(
            Edge{static_cast<size_t>(robot_id_), prevkf->sid, prevkf->kid,
                static_cast<size_t>(robot_id_), kf->sid, kf->kid, T_src_dst});
    }
  }

  std::cout << "Keyframes: " << keyframes.size() << std::endl;
  std::cout << "Submaps: " << keyframesOfSubmap.size() << std::endl;
  std::cout << "Odometry edges: " << odometry.size() << std::endl;
  std::cout << "Private Loop Closures: " << privateLoopClosures.size() << std::endl;
  std::cout << "Public Loop Closures: " << publicLoopClosures.size() << std::endl;

  return {keyframes, odometry, privateLoopClosures, publicLoopClosures};
}

// ----------------------------------------------------------------------------

pose_graph_tools::PoseGraphPtr
PoseGraphGenerator::build_pose_graph(uint64_t start_ns, uint64_t end_ns)
{
  pose_graph_tools::PoseGraphPtr pgmsg{new pose_graph_tools::PoseGraph};

  pgmsg->header.stamp.fromNSec(end_ns);
  pgmsg->header.frame_id = robot_ + "/viodom";

  for (auto kit = keyframes_.begin(); kit != keyframes_.end(); kit++) {
    const auto& kf = kit->second;
    if (kf->t_ns > start_ns && kf->t_ns <= end_ns) { // up to, and including

      // Create node for each keyframe
      pose_graph_tools::PoseGraphNode node;
      node.header.stamp.fromNSec(kf->t_ns);
      node.header.frame_id = pgmsg->header.frame_id;
      node.robot_id = robot_id_;
      node.key = kf->kid;
      tf::poseEigenToMsg(kf->T_viodom_kf, node.pose);
      pgmsg->nodes.push_back(node);

      // Create odometry edge
      if (kit != keyframes_.begin()) {
        const auto& odomedge = odometry_[kf->kid];

        pose_graph_tools::PoseGraphEdge edge;
        edge.header = node.header;
        edge.robot_from = odomedge->r1;
        edge.key_from = odomedge->k1;
        edge.robot_to = odomedge->r2;
        edge.key_to = odomedge->k2;
        edge.type = pose_graph_tools::PoseGraphEdge::ODOM;
        tf::poseEigenToMsg(odomedge->T, edge.pose);
        pgmsg->edges.push_back(edge);
      }

      if (auto search = privateLoopClosures_.find(kf->kid);
                    search != privateLoopClosures_.end()) {
        const auto& lc = search->second;

        pose_graph_tools::PoseGraphEdge edge;
        edge.header = node.header;
        edge.robot_from = lc->r1;
        edge.key_from = lc->k1;
        edge.robot_to = lc->r2;
        edge.key_to = lc->k2;
        edge.type = pose_graph_tools::PoseGraphEdge::LOOPCLOSE;
        tf::poseEigenToMsg(lc->T, edge.pose);
        pgmsg->edges.push_back(edge);
      }

      // TODO(plusk): public loop closures
    }
  }

  return pgmsg;
}

// ----------------------------------------------------------------------------
// ROS Callbacks
// ----------------------------------------------------------------------------

void PoseGraphGenerator::odom_cb(const nav_msgs::OdometryConstPtr& msg)
{
  // find keyframe
  const uint64_t t_ns = msg->header.stamp.toNSec();
  if (auto search = keyframes_.find(t_ns); search != keyframes_.end()) {
    pose_graph_tools::PoseGraphPtr pgmsg = build_pose_graph(0, t_ns);
    pub_posegraph_.publish(pgmsg);

    if (t_ns > t_ns_last_) {
      pose_graph_tools::PoseGraphPtr pgimsg = build_pose_graph(t_ns_last_, t_ns);
      pub_posegraph_incremental_.publish(pgimsg);
      t_ns_last_ = t_ns;
    } else {
      t_ns_last_ = 0;
    }
  }
}

} // ns kmd_tools
