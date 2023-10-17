/**
 * @file odometry_generator.h
 * @brief Publish and broadcast odometry
 * @author Parker Lusk <plusk@mit.edu>
 * @date 5 May 2023
 */

#pragma once

#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "kmd_tools/pose_graph_defines.h"

namespace kmd_tools {

  class OdometryGenerator
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
    OdometryGenerator(const ros::NodeHandle& nh, const ros::NodeHandle& nhp);
    ~OdometryGenerator() = default;

  private:
    ros::NodeHandle nh_, nhp_;
    ros::Subscriber sub_odom_, sub_cinfo_;
    ros::Publisher pub_gt_odom_, pub_gt_path_, pub_gt_pathall_;
    ros::Publisher pub_violog_odom_, pub_violog_path_;
    ros::Timer tim_pathall_;

    ///\brief Dataset
    int robot_id_;
    std::string robot_;
    std::string datadir_, logdir_;
    std::filesystem::path datapath_, logpath_;
    std::string sequence_;

    Params params_;

    std::map<uint64_t, Eigen::Affine3d> gtposes_;
    Eigen::Affine3d T_mo_; ///< odom w.r.t map

    std::map<uint64_t, KeyframePtr> keyframes_;

    KeyframePtr current_kf_; ///< the current odom is >= this kf time
    Eigen::Affine3d localodom_pose_at_current_kf_; ///< baselink pose w.r.t local odom source
    nav_msgs::Path msg_gt_path_, msg_violog_path_; ///< odom trajectories

    // tf broadcaster
    tf2_ros::TransformBroadcaster tf_;
    tf2_ros::StaticTransformBroadcaster tf_static_;
    tf2_ros::Buffer tfbuf_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    bool broadcast_odom_tf_ = true;

    bool get_robot_name_and_id();

    /**
     * @brief      Broadcast a tf to connect the map node to the odom node
     *
     * @param[in]  T_map_odom  Odom w.r.t map
     */
    void tf_connect_map_to_odom(const Eigen::Affine3d& T_map_odom);

    /**
     * @brief      Load ground truth odometry poses from Kimera Multi Data.
     *
     * @param[in]  datapath  The root of the Kimera Multi Data directory
     * @param[in]  seq       Sequence number, either: 1014, 1207, or 1208
     * @param[in]  robot     The robot name, either: acl_jackal, acl_jackal2,
     *                          sparkal1, sparkal2, hathor, thoth, apis, sobek
     *
     * @return     Map of poses, keyed by timestamp [ns] of pose
     */
    std::map<uint64_t, Eigen::Affine3d> load_ground_truth(
                          const std::filesystem::path& datapath,
                          const std::string& seq, const std::string& robot);

    /**
     * @brief      Load keyframes (times & poses) from kimera_distributed log.
     *
     * @param[in]  logdir    Log directory
     * @param[in]  robot_id  The ID number of the robot
     *
     * @return     Map of keyframes, keyed by timestamp [ns]
     */
    std::map<uint64_t, KeyframePtr> load_vio_log(
                      const std::filesystem::path& logdir, const int robot_id);

    void register_viodom_and_odom();

    /**
     * @brief      Add ground truth information to each keyframe. Note that
     *             this ground truth is already expressed in odom frame.
     *
     * @param[in]  keyframes  Keyframes that should be updated
     * @param      gtposes    Timestamped ground truth poses used for interp.
     */
    void set_keyframe_ground_truth_poses(
                          std::map<uint64_t, KeyframePtr>& keyframes,
                          const std::map<uint64_t, Eigen::Affine3d>& gtposes);

    /**
     * @brief      Generates the desired odometry outputs
     *
     * @param[in]  t_ns         The timestamp [ns] of desired odometry output
     * @param[in]  T_localodom  The current pose w.r.t the local odom source
     */
    void generate_odometry(uint64_t t_ns, const Eigen::Affine3d& T_localodom);

    /**
     * @brief      Generate odometry at the desired time, looking ahead of
     *             potential keyframe timestamps.
     *
     * @param[in]  t_ns         Timestamp [ns] of desired odometry output
     * @param[in]  T_localodom  The current pose w.r.t the local odom source
     * @param[in]  kfa          Keyframe whose time is <= t_ns (nullptr if n/a)
     * @param[in]  kfb          Keyframe whose time is > t_ns (nullptr if n/a)
     * @param[in]  method       Method of odometry lookahead
     *
     * @return     The pose {baselink w.r.t odom} to use for odom at t_ns
     */
    Eigen::Affine3d odometry_lookahead(uint64_t t_ns,
                  const Eigen::Affine3d& T_localodom, const KeyframePtr& kfa,
                  const KeyframePtr& kfb, LookaheadMethod method);

  /**
   * @brief      Find keyframes on either side of the time t_ns.
   *
   * @param[in]  t_ns  Timestamp [ns] which should be in [kfa, kfb)
   *
   * @return     kfa and kfb
   */
    std::pair<KeyframePtr, KeyframePtr> get_bracketing_keyframes(
                                                                uint64_t t_ns);

    /**
     * @brief      Publish odometry information
     *
     * @param[in]  t_ns             Timestamp [ns] of published odometry
     * @param[in]  T_odom_baselink  Pose of robot baselink w.r.t odom frame
     * @param      pub_odom         Odometry publisher
     * @param      pub_path         Path publisher
     * @param      msg_path         Path message state
     * @param[in]  broadcast_tf     Should broadcast odom --> base_link tf?
     */
    void publish_odometry(uint64_t t_ns,
        const Eigen::Affine3d& T_odom_baselink, ros::Publisher& pub_odom,
        ros::Publisher& pub_path, nav_msgs::Path& msg_path, bool broadcast_tf);

    /**
     * @brief      Publishes a path of ground truth keyframe poses
     */
    void pub_kf_ground_truth_poses();

    void pathall_cb(const ros::TimerEvent& evt);

    void odom_cb(const nav_msgs::OdometryConstPtr& msg);


    void cinfo_cb(const sensor_msgs::CameraInfoConstPtr& msg);
    
  };

} // ns kmd_tools
