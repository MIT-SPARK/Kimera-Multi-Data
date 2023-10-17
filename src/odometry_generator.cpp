/**
 * @file odometry_generator.cpp
 * @brief Publish and broadcast odometry
 * @author Parker Lusk <plusk@mit.edu>
 * @date 5 May 2023
 */

#include <iostream>
#include <iomanip>

#include "csv/csv.h"

#include "kmd_tools/odometry_generator.h"
#include "kmd_tools/utils.h"

namespace kmd_tools {

namespace fs = std::filesystem;

OdometryGenerator::OdometryGenerator(const ros::NodeHandle& nh, const ros::NodeHandle& nhp)
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
    ROS_FATAL_STREAM("data root `" << logdir_ << "` does not exist");
    ros::shutdown();
    return;
  }

  if (!nhp_.getParam("datadir", datadir_)) {
    ROS_FATAL("Must specify Kimera-Multi Data root in param `datadir`");
    ros::shutdown();
    return;
  }

  datapath_ = datadir_;
  if (!fs::exists(datapath_)) {
    ROS_FATAL_STREAM("data root `" << datadir_ << "` does not exist");
    ros::shutdown();
    return;
  }

  int seq;
  if (!nhp_.getParam("sequence", seq)) {
    ROS_FATAL("Must specify which sequence in param `sequence` (1014, 1207, or 1208).");
    ros::shutdown();
    return;
  } else {
    if (seq != 1014 && seq != 1207 && seq != 1208) {
      ROS_FATAL("The `sequence` param can only be 1014, 1207, or 1208.");
      ros::shutdown();
      return;
    }
    sequence_ = std::to_string(seq);
  }

  // nhp_.getParam("broadcast_odom_tf", broadcast_odom_tf_);

  params_.pub_ground_truth = true;
  params_.pub_violog = true;
  params_.pub_at_kf_times = true;
  params_.tf_odom_broadcaster = TFOdometryBroadcaster::violog;
  params_.lookahead_method_violog = LookaheadMethod::kfInterp;

  //
  // Initialization
  //

  tf_listener_.reset(new tf2_ros::TransformListener(tfbuf_));

  if (params_.pub_violog || params_.pub_at_kf_times) {
    keyframes_ = load_vio_log(logpath_, robot_id_);
  }

  if (params_.pub_ground_truth) {
    gtposes_ = load_ground_truth(datapath_, sequence_, robot_);
    T_mo_ = gtposes_.begin()->second; // robot's odom frame w.r.t map
    if (params_.connect_map_frame_using_gt)
      tf_connect_map_to_odom(T_mo_);
    if (params_.pub_at_kf_times) {
      set_keyframe_ground_truth_poses(keyframes_, gtposes_);
    }
  }

  if (!keyframes_.empty()) {
    register_viodom_and_odom();
  }

  pub_gt_odom_ = nh_.advertise<nav_msgs::Odometry>("ground_truth/odom", 1);
  pub_gt_path_ = nh_.advertise<nav_msgs::Path>("ground_truth/path", 1);
  pub_gt_pathall_ = nh_.advertise<nav_msgs::Path>("ground_truth/complete_path", 1);
  pub_violog_odom_ = nh_.advertise<nav_msgs::Odometry>("kimera_vio_ros/odom", 1);
  pub_violog_path_ = nh_.advertise<nav_msgs::Path>("kimera_vio_ros/path", 1);
  // sub_odom_ = nh_.subscribe("jackal_velocity_controller/odom", 1, &OdometryGenerator::odom_cb, this);
  sub_cinfo_ = nh_.subscribe("forward/infra1/camera_info", 1, &OdometryGenerator::cinfo_cb, this);

  if (params_.pub_ground_truth && params_.pub_at_kf_times)
    tim_pathall_ = nh_.createTimer(ros::Duration(1), &OdometryGenerator::pathall_cb, this);
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

bool OdometryGenerator::get_robot_name_and_id()
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

std::map<uint64_t, Eigen::Affine3d> OdometryGenerator::load_ground_truth(
                            const fs::path& datapath, const std::string& seq,
                            const std::string& robot)
{
  std::map<uint64_t, Eigen::Affine3d> poses;
  const fs::path csv = datapath / "ground_truth" / seq / (robot + "_gt_odom.csv");
 
  io::CSVReader<8> in(csv.string());
  in.read_header(io::ignore_extra_column, "#timestamp_kf", "x", "y", "z",
                  "qw", "qx", "qy", "qz"); // n.b., these must match csv file.
  uint64_t t_ns;
  double x, y, z, qw, qx, qy, qz;
  while (in.read_row(t_ns, x, y, z, qw, qx, qy, qz)) {
    Eigen::Affine3d T_wb = Eigen::Affine3d::Identity();
    T_wb.translation() = Eigen::Vector3d(x, y, z);
    T_wb.linear() = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
    poses[t_ns] = T_wb;
  }

  return poses;
}

// ----------------------------------------------------------------------------

std::map<uint64_t, KeyframePtr> OdometryGenerator::load_vio_log(
                              const fs::path& logdir, const int robot_id)
{
  const fs::path path = logdir
                     / ("robot" + std::to_string(robot_id));

  //
  // Get imu (optical) w.r.t base_link
  //

  Eigen::Affine3d T_BI = Eigen::Affine3d::Identity();
  try {
     const geometry_msgs::TransformStamped msgT_BI = tfbuf_.lookupTransform(
                                      robot_ + "/base_link",
                                      robot_ + "/forward_imu_optical_frame",
                                      ros::Time(0), ros::Duration(1));
     T_BI = tf2::transformToEigen(msgT_BI);
  } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ROS_WARN("Could not determine imu (optical) w.r.t base_link --> VIO"
               " odometry estimate will not be in correct frame!");
  }

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
        keyframes[kf->t_ns]->T_viodom_baselink = T_viodom_submap * kf->T_submap_kf * T_BI.inverse();
      }
    }
  }

  return keyframes;
}

// ----------------------------------------------------------------------------

void OdometryGenerator::register_viodom_and_odom()
{
  // use this to align odom frame (as defined by base_link start pose
  // in the LOCUS lidar map frame) with the viodom frame (as defined by
  // how kimera vio initialized the gravity aligned local/odom frame).
  Eigen::Affine3d T_odom_viodom = Eigen::Affine3d::Identity();

  const auto it = keyframes_.begin();
  T_odom_viodom = it->second->Tgt_odom_baselink * it->second->T_viodom_baselink.inverse();

  for (auto&& [t_ns, kf] : keyframes_) {
    kf->T_odom_baselink = T_odom_viodom * kf->T_viodom_baselink;
  }

  // let others know about the transform
  geometry_msgs::TransformStamped tfmsg = tf2::eigenToTransform(T_odom_viodom);
  tfmsg.header.stamp = ros::Time::now();
  tfmsg.header.frame_id = robot_ + "/odom";
  tfmsg.child_frame_id = robot_ + "/viodom";
  tf_static_.sendTransform(tfmsg);
}

// ----------------------------------------------------------------------------

void OdometryGenerator::set_keyframe_ground_truth_poses(
                            std::map<uint64_t, KeyframePtr>& keyframes,
                            const std::map<uint64_t, Eigen::Affine3d>& gtposes)
{
  for (auto&& [t_ns, kf] : keyframes) {
    // interpolate ground truth pose to the current keyframe time
    kf->Tgt_map_baselink = utils::find_interpolant(gtposes, t_ns);

    // express the base_link pose w.r.t odom frame
    kf->Tgt_odom_baselink = T_mo_.inverse() * kf->Tgt_map_baselink;
  }
}

// ----------------------------------------------------------------------------

void OdometryGenerator::tf_connect_map_to_odom(const Eigen::Affine3d& T_map_odom)
{
  geometry_msgs::TransformStamped tfmsg = tf2::eigenToTransform(T_map_odom);
  tfmsg.header.stamp = ros::Time::now();
  tfmsg.header.frame_id = "map";
  tfmsg.child_frame_id = robot_ + "/odom";

  tf_static_.sendTransform(tfmsg);
}

// ----------------------------------------------------------------------------

std::pair<KeyframePtr, KeyframePtr>
OdometryGenerator::get_bracketing_keyframes(uint64_t t_ns)
{
  if (keyframes_.empty()) return {nullptr, nullptr};

  const auto b = keyframes_.upper_bound(t_ns);
  if (b == keyframes_.begin()) return {nullptr, b->second};

  const auto a = std::prev(b);
  if (b == keyframes_.end()) return {a->second, nullptr};

  return {a->second, b->second};
}

// ----------------------------------------------------------------------------

void OdometryGenerator::generate_odometry(uint64_t t_ns, const Eigen::Affine3d& T_localodom)
{
  const auto [kfa, kfb] = get_bracketing_keyframes(t_ns);

  const bool use_gt = params_.tf_odom_broadcaster == TFOdometryBroadcaster::gt;
  const bool use_violog = params_.tf_odom_broadcaster 
                                    == TFOdometryBroadcaster::violog;

  if (kfa != nullptr && kfa != current_kf_) {

    if (params_.pub_ground_truth) {
      publish_odometry(kfa->t_ns, kfa->Tgt_odom_baselink, pub_gt_odom_, pub_gt_path_,
                        msg_gt_path_, use_gt);
    }

    if (params_.pub_violog) {
      publish_odometry(kfa->t_ns, kfa->T_odom_baselink, pub_violog_odom_, pub_violog_path_,
            msg_violog_path_, use_violog);
    }

    current_kf_ = kfa;
    // TODO(plusk): this isn't correct, i.e., need to interpolate to get actual
    // localodom pose at the current keyframe.
    localodom_pose_at_current_kf_ = T_localodom;
  }

  //
  // Predict current odometry with respect to last keyframe
  //

  if (current_kf_ == nullptr || (current_kf_ != nullptr && t_ns > current_kf_->t_ns)) {
    if (params_.pub_ground_truth) {
      const Eigen::Affine3d Tgt_odom_baselink = odometry_lookahead(t_ns, T_localodom,
                                      kfa, kfb, params_.lookahead_method_gt);
      publish_odometry(t_ns, Tgt_odom_baselink, pub_gt_odom_, pub_gt_path_,
            msg_gt_path_, use_gt);
    }

    if (params_.pub_violog) {
      const Eigen::Affine3d T_odom_baselink = odometry_lookahead(t_ns, T_localodom,
                                      kfa, kfb, params_.lookahead_method_violog);
      publish_odometry(t_ns, T_odom_baselink, pub_violog_odom_, pub_violog_path_,
            msg_violog_path_, use_violog);
    }
  }
}

// ----------------------------------------------------------------------------

Eigen::Affine3d OdometryGenerator::odometry_lookahead(uint64_t t_ns,
                                            const Eigen::Affine3d& T_localodom,
                                            const KeyframePtr& kfa,
                                            const KeyframePtr& kfb,
                                            LookaheadMethod method)
{
  if (kfa == nullptr) {
    if (kfb != nullptr) {
      method = LookaheadMethod::gtNextKF;
    } else {
      // If keyframes are not available, all that can
      // be done is interpolate based on ground truth.
      method = LookaheadMethod::gtInterp;
    }
  }

  Eigen::Affine3d T_odom_baselink = Eigen::Affine3d::Identity();

  if (method == LookaheadMethod::gtInterp) {
    // interpolate ground truth pose to the current time
    const Eigen::Affine3d T_mb = utils::find_interpolant(gtposes_, t_ns);
    // we want the base_link w.r.t odom frame, not map.
    T_odom_baselink = T_mo_.inverse() * T_mb;

  } else if (method == LookaheadMethod::kfExtrap) {
    const Eigen::Affine3d T_delta = localodom_pose_at_current_kf_.inverse() * T_localodom;
    T_odom_baselink = current_kf_->T_odom_baselink * T_delta;

  } else if (method == LookaheadMethod::kfInterp) {

    const auto b = keyframes_.upper_bound(t_ns);
    const auto a = std::prev(b);

    if (b == keyframes_.begin()) {
      T_odom_baselink = b->second->T_odom_baselink; // if before any times, use first
    } else if (b == keyframes_.end()) {
      T_odom_baselink = a->second->T_odom_baselink; // if after, use last
    } else if (t_ns == a->first) {
      T_odom_baselink = a->second->T_odom_baselink; // early return
    } else {
      T_odom_baselink = utils::interpolate(t_ns,
                                    a->first, a->second->T_odom_baselink,
                                    b->first, b->second->T_odom_baselink);
    }

  } else if (method == LookaheadMethod::gtNextKF) {
    T_odom_baselink = kfb->Tgt_odom_baselink;
  }

  return T_odom_baselink;
}

// ----------------------------------------------------------------------------

void OdometryGenerator::publish_odometry(uint64_t t_ns,
                                        const Eigen::Affine3d& T_odom_baselink,
                                        ros::Publisher& pub_odom,
                                        ros::Publisher& pub_path,
                                        nav_msgs::Path& msg_path,
                                        bool broadcast_tf)
{
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp.fromNSec(t_ns);
  pose_msg.header.frame_id = robot_ + "/odom";
  tf::poseEigenToMsg(T_odom_baselink, pose_msg.pose);

  msg_path.poses.push_back(pose_msg);
  msg_path.header = pose_msg.header;
  pub_path.publish(msg_path);

  nav_msgs::Odometry odom_msg;
  odom_msg.header = pose_msg.header;
  odom_msg.child_frame_id = robot_ + "/base_link";
  odom_msg.pose.pose = pose_msg.pose;
  pub_odom.publish(odom_msg);

  if (broadcast_tf) {
    geometry_msgs::TransformStamped tfmsg = tf2::eigenToTransform(T_odom_baselink);
    tfmsg.header = pose_msg.header;
    tfmsg.child_frame_id = robot_ + "/base_link";
    tf_.sendTransform(tfmsg);
  }
}

// ----------------------------------------------------------------------------

void OdometryGenerator::pub_kf_ground_truth_poses()
{
  const uint64_t last_kf_time_ns = std::prev(keyframes_.end())->first;

  nav_msgs::Path msg;
  msg.header.stamp.fromNSec(last_kf_time_ns);
  msg.header.frame_id = "map";

  for (const auto& [t_ns, kf] : keyframes_) {
    geometry_msgs::PoseStamped posemsg;
    posemsg.header.stamp.fromNSec(t_ns);
    posemsg.header.frame_id = msg.header.frame_id;
    tf::poseEigenToMsg(kf->Tgt_map_baselink, posemsg.pose);
    msg.poses.push_back(posemsg);
  }

  pub_gt_pathall_.publish(msg);
}

// ----------------------------------------------------------------------------
// ROS Callbacks
// ----------------------------------------------------------------------------

void OdometryGenerator::pathall_cb(const ros::TimerEvent& evt)
{
  pub_kf_ground_truth_poses();
}

// ----------------------------------------------------------------------------

void OdometryGenerator::odom_cb(const nav_msgs::OdometryConstPtr& msg)
{
  const uint64_t t_ns = msg->header.stamp.toNSec();

  Eigen::Affine3d T_odom_baselink;
  tf::poseMsgToEigen(msg->pose.pose, T_odom_baselink);
  generate_odometry(t_ns, T_odom_baselink);
}

// ----------------------------------------------------------------------------

void OdometryGenerator::cinfo_cb(const sensor_msgs::CameraInfoConstPtr& msg)
{
  const uint64_t t_ns = msg->header.stamp.toNSec();

  Eigen::Affine3d T_odom_baselink = Eigen::Affine3d::Identity();
  generate_odometry(t_ns, T_odom_baselink);
}

} // ns kmd_tools
