/**
 * @file loc_publisher.h
 * @brief All ROS I/O for the map-localization module, in one place.
 *
 * Kept out of ROSWrapper on purpose: /loc topics, the map display, the status
 * message and the /initialpose conversion are one coherent concern, and folding
 * them into the LIO wrapper would grow an already large class for no gain.
 * ROSWrapper gains exactly one hook (an IMU-rate callback) so this class can
 * publish at IMU rate without owning the IMU subscription.
 *
 * Frame conventions, matching what the frontend already publishes:
 *   - the IMU pose is the primary output;
 *   - the robot/base rotation is R_imu * g_odom_robo.R_, and the base position
 *     is the IMU position, which is what pub_odom() does today;
 *   - TF publishes map -> <odom_frame> (dynamic). Do NOT also publish a static
 *     map->world as relocation.py does; the two would fight.
 *
 * Thread affinity: the frontend/executor thread. Every method is called from
 * T0 or from a node timer, never from the backend thread.
 */
#ifndef LOC_PUBLISHER_H_
#define LOC_PUBLISHER_H_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "super_lio/msg/loc_status.hpp"

#include "basic/alias.h"
#include "basic/Manifold.h"
#include "common/ds.h"
#include "loc/keyframe_map.h"
#include "loc/loc_params.h"
#include "loc/map_localizer.h"

namespace LI2Sup {

/// Reads the lio.maploc.* parameters into a LocParams. The only place that
/// knows those names.
loc::LocParams LoadLocParamFromRos(rclcpp::Node& node);

class LocPublisher {
public:
  using InitCallback = std::function<void(const loc::InitRequest&)>;

  LocPublisher(rclcpp::Node* node, const loc::LocParams& params);

  /// Invoked when a human (or a future GPS source) supplies a pose guess.
  void setInitCallback(InitCallback cb) { init_cb_ = std::move(cb); }

  /// Seeds the initial guess from lio.maploc.init_pose when the source is yaml.
  /// Returns false when the configured source is not yaml.
  bool initFromParams() const;
  /// One line naming the active initialization mode, for the startup log.
  const char* initModeDescription() const;
  loc::InitRequest paramsInitRequest() const;

  /// 10 Hz output: /loc/odom, /loc/robo/odom and TF map -> odom_frame.
  ///
  /// T_M_O is needed for more than the TF: the frontend reports velocity in the
  /// ODOM frame, and the drift rotation between odom and map is NOT small (it
  /// carries the robot's whole heading offset from where it booted -- tens of
  /// degrees is routine). Publishing an odom-frame vector under frame_id "map"
  /// was wrong by exactly that angle.
  void publishPose(const BASIC::SE3& T_M_B, const BASIC::SE3& T_M_O,
                   const DynamicState& state);

  /// IMU-rate output: /loc/imu/odom, /loc/imu/robo/odom. No TF (the 10 Hz path
  /// owns that) and no extra filtering: the same applied drift is reused --
  /// which is also why the drift has to be passed in, for the twist.
  void publishImuRatePose(const BASIC::SE3& T_M_B, const BASIC::SE3& T_M_O,
                          const DynamicState& imu_state,
                          const builtin_interfaces::msg::Time& stamp);

  /// /loc/odom_raw: the estimator's pose with no rate limiting applied.
  /// Carries the same twist as /loc/odom so the T10 comparison stays total.
  void publishRawPose(const BASIC::SE3& T_M_B, const BASIC::SE3& T_M_O,
                      const DynamicState& state);

  /// /loc/odom_smooth: the optional smoothing layer's output, published
  /// alongside (never instead of) /loc/odom. Pose only: the smoothing layer
  /// applies a drift of its own, and pairing it with the unsmoothed twist
  /// would describe a motion neither one performs.
  void publishSmoothPose(const BASIC::SE3& T_M_B, const DynamicState& state);

  void publishStatus(const loc::LocStatus& status, bool jump);

 private:
  /// Fill the map-frame twist into either or both messages (pass nullptr to
  /// skip one). One place, so /loc/odom, /loc/robo/odom and the IMU-rate pair
  /// cannot drift apart in either the frame or the lever arm.
  void fillMapTwist(nav_msgs::msg::Odometry* imu_odom,
                    nav_msgs::msg::Odometry* base_odom,
                    const BASIC::SE3& T_M_O, const DynamicState& state);

 public:

  /// /loc/submap, only when the submap actually changed.
  void publishSubmap(const loc::CloudXYZ::Ptr& cloud, double stamp);

  /// /loc/map, on a decaying timer like set_global_map does.
  void setGlobalMap(const loc::CloudXYZ::ConstPtr& cloud);

  /// /loc/keyframes, once.
  void publishKeyframes(const std::vector<loc::Keyframe>& keyframes);

  /// TUM-format trajectory, accumulated in memory and written by save().
  void recordTrajectory(double stamp, const BASIC::SE3& T_M_B);
  void saveTrajectory() const;

  bool publishImuRate() const { return params_.output.publish_imu_rate; }

private:
  /// map-frame base pose from the map-frame IMU pose.
  BASIC::SE3 toBase(const BASIC::SE3& T_M_B) const;
  void handleInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  rclcpp::Node* node_;
  loc::LocParams params_;
  InitCallback init_cb_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_, pub_robo_odom_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_raw_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_smooth_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_imu_odom_, pub_imu_robo_odom_;
  rclcpp::Publisher<super_lio::msg::LocStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_submap_, pub_map_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_keyframes_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reinit_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_;

  sensor_msgs::msg::PointCloud2 map_msg_;
  rclcpp::TimerBase::SharedPtr map_timer_;

  struct TumRow { double t; double x, y, z, qx, qy, qz, qw; };
  std::vector<TumRow> trajectory_;
};

}  // namespace LI2Sup

#endif  // LOC_PUBLISHER_H_
