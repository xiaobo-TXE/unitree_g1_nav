#include "ros/loc_publisher.h"

#include <cctype>
#include <cmath>
#include <fstream>

#include <pcl_conversions/pcl_conversions.h>

#include "basic/logs.h"
#include "lio/params.h"

namespace LI2Sup {

namespace {

rclcpp::Time toRosTime(rclcpp::Node* node, double seconds) {
  if (seconds <= 0.0) return node->now();
  const int32_t sec = static_cast<int32_t>(seconds);
  const uint32_t nsec = static_cast<uint32_t>((seconds - sec) * 1e9);
  return rclcpp::Time(sec, nsec, RCL_ROS_TIME);
}

void fillPose(geometry_msgs::msg::Pose* out, const BASIC::SE3& T) {
  out->position.x = T.t_.x();
  out->position.y = T.t_.y();
  out->position.z = T.t_.z();
  const BASIC::Quat q = T.quaternion();
  out->orientation.x = q.x();
  out->orientation.y = q.y();
  out->orientation.z = q.z();
  out->orientation.w = q.w();
}

}  // namespace

loc::LocParams LoadLocParamFromRos(rclcpp::Node& node) {
  loc::LocParams p = loc::LocParams::Defaults();

  auto get_s = [&node](const std::string& name, const std::string& def) {
    node.declare_parameter<std::string>(name, def);
    std::string v; node.get_parameter(name, v); return v;
  };
  auto get_d = [&node](const std::string& name, double def) {
    node.declare_parameter<double>(name, def);
    double v; node.get_parameter(name, v); return v;
  };
  auto get_i = [&node](const std::string& name, int def) {
    node.declare_parameter<int>(name, def);
    int v; node.get_parameter(name, v); return v;
  };
  auto get_b = [&node](const std::string& name, bool def) {
    node.declare_parameter<bool>(name, def);
    bool v; node.get_parameter(name, v); return v;
  };
  const double kDeg = M_PI / 180.0;

  p.map.map_dir      = get_s("lio.maploc.map_dir", "");
  p.map.preload_all  = get_b("lio.maploc.preload_all", false);
  p.map.lru_capacity = get_i("lio.maploc.lru_capacity", 300);

  p.submap.radius       = get_d("lio.maploc.submap_radius", 15.0);
  p.submap.max_k        = get_i("lio.maploc.submap_max_k", 20);
  p.submap.refresh_dist = get_d("lio.maploc.submap_refresh_dist", 3.0);
  p.submap.prefetch_max_files = get_i("lio.maploc.submap_prefetch_max_files", 20);

  p.scan.accum_frames = get_i("lio.maploc.accum_frames", 5);
  p.scan.source_voxel = get_d("lio.maploc.source_voxel", 0.25);

  const std::string type = get_s("lio.maploc.registration_type", "VGICP");
  p.track.type = type == "GICP"     ? loc::RegistrationType::GICP
               : type == "PCL_GICP" ? loc::RegistrationType::PCL_GICP
                                    : loc::RegistrationType::VGICP;
  p.coarse.type = p.track.type;

  p.track.target_voxel  = get_d("lio.maploc.target_voxel", 0.5);
  p.track.source_voxel  = p.scan.source_voxel;
  p.track.max_corr_dist = get_d("lio.maploc.max_corr_dist", 1.0);
  p.track.max_iter      = get_i("lio.maploc.max_iter", 20);
  p.track.num_threads   = get_i("lio.maploc.num_threads", 2);
  p.track.cov_neighbors = get_i("lio.maploc.cov_neighbors", 10);
  p.track.cov_floor_t   = get_d("lio.maploc.cov_floor_t", 0.03);
  p.track.cov_floor_r   = get_d("lio.maploc.cov_floor_r", 0.3) * kDeg;
  p.track.degenerate_eig_ratio = get_d("lio.maploc.degenerate_eig_ratio", 1e-3);

  // The coarse pass shares the quality settings but searches far more widely.
  // Measured on this map: tracking parameters only converge from about
  // 0.3 m / 5 deg, so anything larger MUST come through the coarse pass first.
  p.coarse.target_voxel  = get_d("lio.maploc.init_target_voxel", 2.0);
  p.coarse.source_voxel  = get_d("lio.maploc.init_source_voxel", 0.5);
  p.coarse.max_corr_dist = get_d("lio.maploc.init_max_corr_dist", 3.0);
  p.coarse.max_iter      = get_i("lio.maploc.init_max_iter", 30);
  p.coarse.num_threads   = p.track.num_threads;
  p.coarse.cov_neighbors = p.track.cov_neighbors;
  p.coarse.cov_floor_t   = p.track.cov_floor_t;
  p.coarse.cov_floor_r   = p.track.cov_floor_r;

  p.trigger.match_min_dist     = get_d("lio.maploc.match_min_dist", 0.5);
  p.trigger.match_max_interval = get_d("lio.maploc.match_max_interval", 2.0);

  p.est.track_inlier_min = get_d("lio.maploc.track_inlier_min", 0.5);
  p.est.track_rmse_max   = get_d("lio.maploc.track_rmse_max", 0.3);
  p.est.gate_body_t      = get_d("lio.maploc.gate_body_t", 1.0);
  p.est.gate_body_r      = get_d("lio.maploc.gate_body_r", 5.0) * kDeg;
  p.est.chi2_gate        = get_d("lio.maploc.chi2_gate", 16.8);
  p.est.sigma_t_per_m    = get_d("lio.maploc.sigma_t_per_m", 0.05);
  p.est.sigma_r_per_m    = get_d("lio.maploc.sigma_r_per_m", 0.002);
  p.est.sigma_r_per_tilt = get_d("lio.maploc.sigma_r_per_tilt", 0.10);
  p.est.sigma_t_per_s    = get_d("lio.maploc.sigma_t_per_s", 0.02);
  p.est.sigma_r_per_s    = get_d("lio.maploc.sigma_r_per_s", 0.0005);
  p.est.estimate_roll_pitch = get_b("lio.maploc.estimate_roll_pitch", true);
  p.est.rp_pseudo_sigma  = get_d("lio.maploc.rp_pseudo_sigma", 2.0) * kDeg;
  p.est.rp_anchor_rate   = get_d("lio.maploc.rp_anchor_rate", 0.1) * kDeg;
  p.est.max_degenerate_t_dims = get_i("lio.maploc.max_degenerate_t_dims", 1);
  p.est.max_degenerate_r_dims = get_i("lio.maploc.max_degenerate_r_dims", 0);
  p.est.rate_gate_enable = get_b("lio.maploc.rate_gate_enable", true);
  p.est.chi2_confirm_enable = get_b("lio.maploc.chi2_confirm_enable", true);
  p.est.rate_gate_t = get_d("lio.maploc.rate_gate_t", 0.6);
  p.est.rate_gate_r = get_d("lio.maploc.rate_gate_r", 0.05);
  p.est.rate_gate_floor_t = get_d("lio.maploc.rate_gate_floor_t", 0.13);
  // Degrees at the parameter boundary, radians inside, like every other
  // rotational threshold here (gate_body_r, cov_floor_r, snap_r, ...).
  p.est.rate_gate_floor_r = get_d("lio.maploc.rate_gate_floor_r", 1.3) * kDeg;
  p.est.unconverged_cov_scale = get_d("lio.maploc.unconverged_cov_scale", 4.0);

  p.watch.enable       = get_b("lio.maploc.watch_enable", true);
  p.watch.max_speed    = get_d("lio.maploc.watch_max_speed", 5.0);
  p.watch.speed_frames = get_i("lio.maploc.watch_speed_frames", 3);
  p.watch.max_gap_s    = get_d("lio.maploc.watch_max_gap", 1.0);
  p.watch.odom_radius_margin = get_d("lio.maploc.watch_odom_radius_margin", 50.0);
  p.watch.recover_frames = get_i("lio.maploc.watch_recover_frames", 20);

  p.composer.v_max  = get_d("lio.maploc.v_max", 0.2);
  p.composer.w_max  = get_d("lio.maploc.w_max", 5.0) * kDeg;
  p.composer.tau    = get_d("lio.maploc.tau", 1.0);
  p.composer.snap_t = get_d("lio.maploc.snap_t", 1.0);
  p.composer.snap_r = get_d("lio.maploc.snap_r", 10.0) * kDeg;

  std::vector<double> init_pose(6, 0.0);
  node.declare_parameter<std::vector<double>>("lio.maploc.init_pose", init_pose);
  node.get_parameter("lio.maploc.init_pose", init_pose);
  for (int i = 0; i < 6 && i < static_cast<int>(init_pose.size()); ++i) {
    p.init.init_pose[i] = init_pose[i];
  }
  // Trimmed and lower-cased: this selects between "seed yourself at startup"
  // and "sit still until someone sends a pose", and an exact string compare
  // meant a stray trailing space in the yaml silently chose the other one.
  {
    std::string src = get_s("lio.maploc.init_pose_source", "yaml");
    const auto b = src.find_first_not_of(" \t\r\n");
    const auto e = src.find_last_not_of(" \t\r\n");
    src = (b == std::string::npos) ? std::string() : src.substr(b, e - b + 1);
    for (char& c : src) c = static_cast<char>(std::tolower(c));
    if (src != "yaml" && src != "topic" && src != "gps") {
      LOG(WARNING) << YELLOW << " ---> [MapLoc] unknown init_pose_source '" << src
                   << "', falling back to 'topic' (wait for /initialpose)" << RESET;
      src = "topic";
    }
    p.init.source = src;
  }
  p.init.yaw_known     = get_b("lio.maploc.init_yaw_known", true);
  p.init.yaw_range     = get_d("lio.maploc.init_yaw_range", 45.0) * kDeg;
  p.init.yaw_step      = get_d("lio.maploc.init_yaw_step", 15.0) * kDeg;
  p.init.xy_step       = get_d("lio.maploc.init_xy_step", 1.0);
  p.init.z_from_map    = get_b("lio.maploc.init_z_from_map", true);
  p.init.submap_radius = get_d("lio.maploc.init_submap_radius", 25.0);
  p.init.submap_max_k  = get_i("lio.maploc.init_submap_max_k", 40);
  p.init.inlier_min    = get_d("lio.maploc.init_inlier_min", 0.6);
  p.init.rmse_max      = get_d("lio.maploc.init_rmse_max", 0.2);
  p.init.max_attempts  = get_i("lio.maploc.init_max_attempts", 25);
  p.init.eager_attempts = get_i("lio.maploc.init_eager_attempts", 3);
  p.init.attempt_min_dist     = get_d("lio.maploc.init_attempt_min_dist", 0.5);
  p.init.attempt_min_interval = get_d("lio.maploc.init_attempt_min_interval", 1.0);
  p.init.refine_top_k  = get_i("lio.maploc.init_refine_top_k", 3);
  p.init.consistency_t = get_d("lio.maploc.init_consistency_t", 0.3);
  p.init.consistency_r = get_d("lio.maploc.init_consistency_r", 3.0) * kDeg;

  p.lost.reject_count = get_i("lio.maploc.lost_reject_count", 6);
  p.lost.timeout      = get_d("lio.maploc.lost_timeout", 10.0);
  p.lost.yaw_range    = get_d("lio.maploc.lost_yaw_range", 30.0) * kDeg;
  p.lost.xy_range     = get_d("lio.maploc.lost_xy_range", 3.0);
  p.lost.attempt_min_dist     = get_d("lio.maploc.lost_attempt_min_dist", 0.2);
  p.lost.attempt_min_interval = get_d("lio.maploc.lost_attempt_min_interval", 0.3);

  p.output.publish_imu_rate = get_b("lio.maploc.publish_imu_rate", true);
  p.output.publish_submap   = get_b("lio.maploc.publish_submap", true);
  p.output.publish_when_invalid = get_b("lio.maploc.publish_when_invalid", false);
  p.output.save_trajectory  = get_s("lio.maploc.save_trajectory", "");
  p.output.match_log        = get_s("lio.maploc.match_log", "");
  p.output.publish_odom_raw = get_b("lio.maploc.publish_odom_raw", false);
  p.output.smooth_output    = get_b("lio.maploc.smooth_output", false);
  p.output.odom_frame       = get_s("lio.maploc.odom_frame", "world");
  p.output.map_frame        = get_s("lio.maploc.map_frame", "map");

  p.log.debug           = get_b("lio.maploc.debug", false);
  p.log.heartbeat_s     = get_d("lio.maploc.log_heartbeat_s", 5.0);
  p.log.summary_on_exit = get_b("lio.maploc.log_summary_on_exit", true);
  p.log.warn_rmse       = get_d("lio.maploc.log_warn_rmse", 0.15);
  p.log.warn_inlier     = get_d("lio.maploc.log_warn_inlier", 0.75);
  p.log.warn_corr_t     = get_d("lio.maploc.log_warn_corr_t", 0.10);
  p.log.warn_corr_r     = get_d("lio.maploc.log_warn_corr_r", 1.0) * kDeg;
  p.log.warn_match_ms   = get_d("lio.maploc.log_warn_match_ms", 50.0);
  p.log.warn_submap_ms  = get_d("lio.maploc.log_warn_submap_ms", 150.0);
  p.log.warn_every_n    = get_i("lio.maploc.log_warn_every_n", 10);
  p.log.expected_frame_hz = get_d("lio.maploc.expected_frame_hz", 0.0);

  // H1: dump every EFFECTIVE value. The C++ defaults and the yaml are two
  // sources for the same knob, and a yaml that silently re-stated an old value
  // once meant the node ran with pre-fix parameters while every offline test
  // (which uses LocParams::Defaults()) reported the fix working. Printing what
  // actually took effect is the cheapest way to never repeat that.
  const double kRad2Deg = 180.0 / M_PI;
  LOG(INFO) << GREEN << " ---> [MapLoc] effective parameters:" << RESET
    << "\n  map_dir=" << p.map.map_dir
    << " preload_all=" << p.map.preload_all << " lru=" << p.map.lru_capacity
    << "\n  submap radius=" << p.submap.radius << " max_k=" << p.submap.max_k
    << " refresh=" << p.submap.refresh_dist << " prefetch_max=" << p.submap.prefetch_max_files
    << "\n  scan accum=" << p.scan.accum_frames << " source_voxel=" << p.scan.source_voxel
    << "\n  track: voxel=" << p.track.target_voxel << " corr=" << p.track.max_corr_dist
    << " iter=" << p.track.max_iter << " threads=" << p.track.num_threads
    << " cov_nn=" << p.track.cov_neighbors
    << " floor_t=" << p.track.cov_floor_t << " floor_r_deg=" << p.track.cov_floor_r * kRad2Deg
    << "\n  coarse: voxel=" << p.coarse.target_voxel << " src_voxel=" << p.coarse.source_voxel
    << " corr=" << p.coarse.max_corr_dist << " iter=" << p.coarse.max_iter
    << "\n  trigger: min_dist=" << p.trigger.match_min_dist
    << " max_interval=" << p.trigger.match_max_interval
    << "\n  gates: inlier>=" << p.est.track_inlier_min << " rmse<=" << p.est.track_rmse_max
    << " body_t=" << p.est.gate_body_t << " body_r_deg=" << p.est.gate_body_r * kRad2Deg
    << " chi2=" << p.est.chi2_gate << " max_deg_t=" << p.est.max_degenerate_t_dims
    << " max_deg_r=" << p.est.max_degenerate_r_dims
    << " rate_gate=" << p.est.rate_gate_enable
    << " chi2_confirm=" << p.est.chi2_confirm_enable
    << " rate_t=" << p.est.rate_gate_t << " rate_r=" << p.est.rate_gate_r
    << " rate_floor_t=" << p.est.rate_gate_floor_t
    << " rate_floor_r_deg=" << p.est.rate_gate_floor_r * kRad2Deg
    << " unconv_scale=" << p.est.unconverged_cov_scale
    << "\n  frontend watch: enable=" << p.watch.enable
    << " max_speed=" << p.watch.max_speed << " frames=" << p.watch.speed_frames
    << " max_gap=" << p.watch.max_gap_s
    << " odom_radius_margin=" << p.watch.odom_radius_margin
    << " recover_frames=" << p.watch.recover_frames
    << "\n  process noise: t/m=" << p.est.sigma_t_per_m << " r/m=" << p.est.sigma_r_per_m
    << " r/tilt=" << p.est.sigma_r_per_tilt
    << " t/s=" << p.est.sigma_t_per_s << " r/s=" << p.est.sigma_r_per_s
    << " est_rp=" << p.est.estimate_roll_pitch
    << " rp_sigma_deg=" << p.est.rp_pseudo_sigma * kRad2Deg
    << " rp_anchor_rate_deg_s=" << p.est.rp_anchor_rate * kRad2Deg
    << "\n  composer: v_max=" << p.composer.v_max << " w_max_deg=" << p.composer.w_max * kRad2Deg
    << " tau=" << p.composer.tau << " snap_t=" << p.composer.snap_t
    << " snap_r_deg=" << p.composer.snap_r * kRad2Deg
    << "\n  init: source=" << p.init.source << " yaw_known=" << p.init.yaw_known
    << " yaw_range_deg=" << p.init.yaw_range * kRad2Deg
    << " yaw_step_deg=" << p.init.yaw_step * kRad2Deg << " xy_step=" << p.init.xy_step
    << " z_from_map=" << p.init.z_from_map
    << " radius=" << p.init.submap_radius << " max_k=" << p.init.submap_max_k
    << " inlier_min=" << p.init.inlier_min << " rmse_max=" << p.init.rmse_max
    << " max_attempts=" << p.init.max_attempts << " eager=" << p.init.eager_attempts
    << " attempt_dist=" << p.init.attempt_min_dist
    << " attempt_interval=" << p.init.attempt_min_interval
    << " top_k=" << p.init.refine_top_k
    << "\n  lost: rejects=" << p.lost.reject_count << " timeout=" << p.lost.timeout
    << " yaw_range_deg=" << p.lost.yaw_range * kRad2Deg << " xy_range=" << p.lost.xy_range
    << " attempt_dist=" << p.lost.attempt_min_dist
    << " attempt_interval=" << p.lost.attempt_min_interval
    << "\n  output: imu_rate=" << p.output.publish_imu_rate
    << " submap=" << p.output.publish_submap
    << " odom_raw=" << p.output.publish_odom_raw
    << " smooth=" << p.output.smooth_output
    << " traj='" << p.output.save_trajectory << "'"
    << " match_log='" << p.output.match_log << "'"
    << "\n  log: debug=" << p.log.debug << " heartbeat_s=" << p.log.heartbeat_s
    << " summary=" << p.log.summary_on_exit
    << " warn: rmse=" << p.log.warn_rmse << " inlier=" << p.log.warn_inlier
    << " corr_t=" << p.log.warn_corr_t << " corr_r_deg=" << p.log.warn_corr_r * kRad2Deg
    << " match_ms=" << p.log.warn_match_ms << " submap_ms=" << p.log.warn_submap_ms
    << " every_n=" << p.log.warn_every_n
    << " expected_frame_hz=" << p.log.expected_frame_hz;
  return p;
}

LocPublisher::LocPublisher(rclcpp::Node* node, const loc::LocParams& params)
    : node_(node), params_(params) {
  pub_odom_      = node_->create_publisher<nav_msgs::msg::Odometry>("/loc/odom", 100);
  pub_robo_odom_ = node_->create_publisher<nav_msgs::msg::Odometry>("/loc/robo/odom", 100);
  pub_status_    = node_->create_publisher<super_lio::msg::LocStatus>("/loc/status", 10);
  if (params_.output.smooth_output) {
    pub_odom_smooth_ = node_->create_publisher<nav_msgs::msg::Odometry>(
        "/loc/odom_smooth", 100);
  }
  if (params_.output.publish_odom_raw) {
    // The estimator's pose before the output limiter. Same stamp as /loc/odom,
    // so subtracting the two isolates exactly what the limiter costs.
    pub_odom_raw_ = node_->create_publisher<nav_msgs::msg::Odometry>("/loc/odom_raw", 100);
  }
  pub_submap_    = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/loc/submap", 2);

  // The prior map and the keyframe poses are published once (or on a decaying
  // timer) and never change, while RViz is almost always started after the
  // node. Latch them so a late subscriber still gets the last message instead
  // of an empty display -- with volatile durability, /loc/keyframes was simply
  // never seen by anyone.
  const auto latched = rclcpp::QoS(1).transient_local().reliable();
  pub_map_       = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/loc/map", latched);
  pub_keyframes_ = node_->create_publisher<geometry_msgs::msg::PoseArray>("/loc/keyframes", latched);
  if (params_.output.publish_imu_rate) {
    pub_imu_odom_ = node_->create_publisher<nav_msgs::msg::Odometry>("/loc/imu/odom", 10);
    pub_imu_robo_odom_ =
        node_->create_publisher<nav_msgs::msg::Odometry>("/loc/imu/robo/odom", 10);
  }
  tf_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  sub_initialpose_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 1,
      std::bind(&LocPublisher::handleInitialPose, this, std::placeholders::_1));

  srv_reinit_ = node_->create_service<std_srvs::srv::Trigger>(
      "/loc/reinit",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        if (!initFromParams()) {
          res->success = false;
          res->message = "init_pose_source is not 'yaml'; publish /initialpose instead";
          return;
        }
        if (init_cb_) init_cb_(paramsInitRequest());
        res->success = true;
        res->message = "re-initializing from lio.maploc.init_pose";
      });
}

bool LocPublisher::initFromParams() const { return params_.init.source == "yaml"; }

const char* LocPublisher::initModeDescription() const {
  return params_.init.source == "yaml"
             ? "yaml: seeding the initial guess now from lio.maploc.init_pose"
             : "topic: staying in WAIT_INIT until /initialpose (or /loc/reinit) arrives";
}

loc::InitRequest LocPublisher::paramsInitRequest() const {
  // init_pose is the BASE pose in the map frame (x y z roll pitch yaw, degrees),
  // which is what a human reads off a map; convert it to the IMU pose the
  // backend works in.
  const double kDeg = M_PI / 180.0;
  const auto& ip = params_.init.init_pose;
  const Eigen::Matrix3d R_base =
      (Eigen::AngleAxisd(ip[5] * kDeg, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(ip[4] * kDeg, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(ip[3] * kDeg, Eigen::Vector3d::UnitX())).toRotationMatrix();

  const BASIC::M3 Rb = R_base.cast<BASIC::scalar>();
  const BASIC::SE3 T_M_robot(BASIC::SO3(Rb), BASIC::V3(ip[0], ip[1], ip[2]));

  loc::InitRequest req;
  req.T_M_B = robotToImu(T_M_robot);
  req.yaw_known = params_.init.yaw_known;
  req.use_map_z = params_.init.z_from_map;
  return req;
}

void LocPublisher::handleInitialPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  // Same base -> IMU conversion as above. Note there is deliberately no +0.79 z
  // fudge here (see the frontend's set_initial_data): z comes from the map.
  const auto& q = msg->pose.pose.orientation;
  const BASIC::Quat qb(static_cast<BASIC::scalar>(q.w), static_cast<BASIC::scalar>(q.x),
                       static_cast<BASIC::scalar>(q.y), static_cast<BASIC::scalar>(q.z));
  const BASIC::M3 R_base = qb.normalized().toRotationMatrix();
  const BASIC::V3 p_base(msg->pose.pose.position.x,
                         msg->pose.pose.position.y,
                         msg->pose.pose.position.z);

  loc::InitRequest req;
  req.T_M_B = robotToImu(BASIC::SE3(BASIC::SO3(R_base), p_base));
  req.yaw_known = true;   // a human pointing at a map knows which way it faces
  req.use_map_z = params_.init.z_from_map;

  LOG(INFO) << YELLOW << " ---> [MapLoc] /initialpose: base ("
            << p_base.transpose() << ") -> imu (" << req.T_M_B.t_.transpose() << ")"
            << RESET;
  if (init_cb_) init_cb_(req);
}

BASIC::SE3 LocPublisher::toBase(const BASIC::SE3& T_M_B) const {
  // The robot centre. imuToRobot() is a full SE(3) composition, so the lever
  // arm is included -- the old version returned the IMU position with the
  // robot's orientation, which is only the robot centre when the sensor sits
  // exactly at the robot origin.
  return imuToRobot(T_M_B);
}

void LocPublisher::fillMapTwist(nav_msgs::msg::Odometry* imu_odom,
                                nav_msgs::msg::Odometry* base_odom,
                                const BASIC::SE3& T_M_O,
                                const DynamicState& state) {
  // mapTwist() owns the frame reasoning and the lever arm; this function is
  // only the copy into message fields, so the math is unit-tested without ROS.
  const MapTwist tw = mapTwist(T_M_O, state.R, state.v, state.w);

  if (imu_odom) {
    // The IMU origin's own velocity: no lever arm, it IS the reference point.
    imu_odom->twist.twist.linear.x = tw.v_imu.x();
    imu_odom->twist.twist.linear.y = tw.v_imu.y();
    imu_odom->twist.twist.linear.z = tw.v_imu.z();
    imu_odom->twist.twist.angular.x = tw.w.x();
    imu_odom->twist.twist.angular.y = tw.w.y();
    imu_odom->twist.twist.angular.z = tw.w.z();
  }
  if (base_odom) {
    // The base origin's, which differs by omega x r whenever the robot turns.
    base_odom->twist.twist.linear.x = tw.v_base.x();
    base_odom->twist.twist.linear.y = tw.v_base.y();
    base_odom->twist.twist.linear.z = tw.v_base.z();
    base_odom->twist.twist.angular.x = tw.w.x();
    base_odom->twist.twist.angular.y = tw.w.y();
    base_odom->twist.twist.angular.z = tw.w.z();
  }
}

void LocPublisher::publishPose(const BASIC::SE3& T_M_B, const BASIC::SE3& T_M_O,
                               const DynamicState& state) {
  const auto stamp = toRosTime(node_, state.time);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = params_.output.map_frame;
  odom.child_frame_id = "imu";
  fillPose(&odom.pose.pose, T_M_B);

  const BASIC::SE3 T_M_base = toBase(T_M_B);
  nav_msgs::msg::Odometry robo = odom;
  robo.child_frame_id = "base";
  fillPose(&robo.pose.pose, T_M_base);

  fillMapTwist(&odom, &robo, T_M_O, state);
  pub_odom_->publish(odom);
  pub_robo_odom_->publish(robo);

  // map -> odom_frame, dynamic. The frontend already broadcasts
  // odom_frame -> imu / base, so the chain completes without us duplicating it.
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = params_.output.map_frame;
  tf.child_frame_id = params_.output.odom_frame;
  tf.transform.translation.x = T_M_O.t_.x();
  tf.transform.translation.y = T_M_O.t_.y();
  tf.transform.translation.z = T_M_O.t_.z();
  const BASIC::Quat q = T_M_O.quaternion();
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();
  tf_->sendTransform(tf);

  if (!params_.output.save_trajectory.empty()) {
    // The IMU pose, not the base pose: this file is compared against
    // alidarState.txt, which is the IMU trajectory in the map frame. Logging
    // base poses made the ATE rotation error meaningless (they differ by
    // g_odom_robo, a 180 deg roll in the current config).
    recordTrajectory(state.time, T_M_B);
  }
}

void LocPublisher::publishImuRatePose(const BASIC::SE3& T_M_B,
                                      const BASIC::SE3& T_M_O,
                                      const DynamicState& imu_state,
                                      const builtin_interfaces::msg::Time& stamp) {
  if (!pub_imu_odom_) return;

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = params_.output.map_frame;
  odom.child_frame_id = "imu";
  fillPose(&odom.pose.pose, T_M_B);

  nav_msgs::msg::Odometry robo = odom;
  robo.child_frame_id = "base";
  fillPose(&robo.pose.pose, toBase(T_M_B));

  // Same formula as the 10 Hz path, so the two rates cannot disagree. The
  // angular term used to go out as the raw body-frame gyro next to a
  // world-frame linear velocity -- two frames in one twist.
  fillMapTwist(&odom, &robo, T_M_O, imu_state);
  pub_imu_odom_->publish(odom);
  pub_imu_robo_odom_->publish(robo);
}

void LocPublisher::publishRawPose(const BASIC::SE3& T_M_B, const BASIC::SE3& T_M_O,
                                  const DynamicState& state) {
  if (!pub_odom_raw_) return;
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = toRosTime(node_, state.time);
  odom.header.frame_id = params_.output.map_frame;
  odom.child_frame_id = "imu";
  fillPose(&odom.pose.pose, T_M_B);
  fillMapTwist(&odom, nullptr, T_M_O, state);
  pub_odom_raw_->publish(odom);
}

void LocPublisher::publishSmoothPose(const BASIC::SE3& T_M_B, const DynamicState& state) {
  if (!pub_odom_smooth_) return;
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = toRosTime(node_, state.time);
  odom.header.frame_id = params_.output.map_frame;
  odom.child_frame_id = "imu";
  fillPose(&odom.pose.pose, T_M_B);
  pub_odom_smooth_->publish(odom);
}

void LocPublisher::publishStatus(const loc::LocStatus& s, bool jump) {
  super_lio::msg::LocStatus msg;
  msg.header.stamp = toRosTime(node_, s.stamp);
  msg.header.frame_id = params_.output.map_frame;
  msg.state = static_cast<uint8_t>(s.state);
  msg.valid = s.valid;
  msg.jump = jump;
  fillPose(&msg.t_map_odom, s.T_M_O);
  msg.inlier_ratio = static_cast<float>(s.inlier_ratio);
  msg.rmse = static_cast<float>(s.rmse);
  msg.correction_t = static_cast<float>(s.correction_t);
  msg.correction_r = static_cast<float>(s.correction_r * 180.0 / M_PI);
  msg.last_match_age = std::isfinite(s.last_match_age)
                           ? static_cast<float>(s.last_match_age)
                           : -1.0f;
  msg.match_time_ms = static_cast<float>(s.match_time_ms);
  msg.submap_build_ms = static_cast<float>(s.submap_build_ms);
  msg.consecutive_rejects = s.consecutive_rejects;
  msg.jump_seq = s.jump_seq;
  msg.last_reject = s.last_reject;
  msg.submap_ids = s.submap_ids;
  msg.queue_dropped = static_cast<int32_t>(s.queue_dropped);
  pub_status_->publish(msg);
}

void LocPublisher::publishSubmap(const loc::CloudXYZ::Ptr& cloud, double stamp) {
  if (!params_.output.publish_submap || !cloud || cloud->empty()) return;
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  msg.header.frame_id = params_.output.map_frame;
  msg.header.stamp = toRosTime(node_, stamp);
  pub_submap_->publish(msg);
}

void LocPublisher::setGlobalMap(const loc::CloudXYZ::ConstPtr& cloud) {
  if (!cloud || cloud->empty()) return;
  pcl::toROSMsg(*cloud, map_msg_);
  map_msg_.header.frame_id = params_.output.map_frame;

  // Publish once straight away. The topic is latched, so this alone serves
  // every future subscriber; waiting for the first timer tick left a one second
  // window in which a subscriber could connect and receive nothing.
  map_msg_.header.stamp = node_->now();
  pub_map_->publish(map_msg_);

  // Same decaying cadence as ROSWrapper::set_global_map: latecomers still get
  // the map, without paying for it forever.
  map_timer_ = node_->create_wall_timer(std::chrono::seconds(1), [this]() {
    static int count = -1;
    static int interval = 1;
    if (++count % interval != 0) return;
    count = 0;
    if (++interval > 10) interval = 10;
    map_msg_.header.stamp = node_->now();
    pub_map_->publish(map_msg_);
  });
}

void LocPublisher::publishKeyframes(const std::vector<loc::Keyframe>& keyframes) {
  geometry_msgs::msg::PoseArray msg;
  msg.header.frame_id = params_.output.map_frame;
  msg.header.stamp = node_->now();
  msg.poses.reserve(keyframes.size());
  for (const auto& k : keyframes) {
    geometry_msgs::msg::Pose p;
    fillPose(&p, k.T_M_B);
    msg.poses.push_back(p);
  }
  pub_keyframes_->publish(msg);
}

void LocPublisher::recordTrajectory(double stamp, const BASIC::SE3& T_M_B) {
  const BASIC::Quat q = T_M_B.quaternion();
  trajectory_.push_back({stamp, T_M_B.t_.x(), T_M_B.t_.y(), T_M_B.t_.z(),
                         q.x(), q.y(), q.z(), q.w()});
}

void LocPublisher::saveTrajectory() const {
  if (params_.output.save_trajectory.empty() || trajectory_.empty()) return;
  std::ofstream f(params_.output.save_trajectory);
  if (!f.is_open()) {
    LOG(ERROR) << " ---> [MapLoc] cannot write " << params_.output.save_trajectory;
    return;
  }
  f.precision(9);
  f << std::fixed;
  for (const auto& r : trajectory_) {
    f << r.t << " " << r.x << " " << r.y << " " << r.z << " "
      << r.qx << " " << r.qy << " " << r.qz << " " << r.qw << "\n";
  }
  LOG(INFO) << " ---> [MapLoc] wrote " << trajectory_.size() << " poses (TUM) to "
            << params_.output.save_trajectory;
}

}  // namespace LI2Sup
