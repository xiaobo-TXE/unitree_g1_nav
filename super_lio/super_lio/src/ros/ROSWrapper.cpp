
#include "ros/ROSWrapper.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include <algorithm>
#include <chrono>
#include <sstream>


using namespace BASIC;

namespace LI2Sup{

void LoadParamFromRos(rclcpp::Node& node)
{
  node.declare_parameter<bool>("lio.map.save_map", false);
  node.get_parameter("lio.map.save_map", g_save_map);

  LOG(INFO) << GREEN << " ---> [Param] map/save_map: "
            << (g_save_map ? "true" : "false") << RESET;

  node.declare_parameter<bool>("lio.eva.timer", false);
  node.get_parameter("lio.eva.timer", g_time_eva);

  node.declare_parameter<bool>("lio.map.if_filter", false);
  node.get_parameter("lio.map.if_filter", g_if_filter);

  node.declare_parameter<std::string>("lio.map.save_map_dir", "");
  node.get_parameter("lio.map.save_map_dir", g_save_map_dir);
  g_save_map_dir = g_root_dir + g_save_map_dir;

  node.declare_parameter<std::string>("lio.map.map_name", "default");
  node.get_parameter("lio.map.map_name", g_map_name);

  node.declare_parameter<double>("lio.map.ds_size", 0.5);
  node.get_parameter("lio.map.ds_size", g_map_ds_size);

  node.declare_parameter<int>("lio.map.save_interval", 1);
  node.get_parameter("lio.map.save_interval", g_pcd_save_interval);

  node.declare_parameter<std::string>("lio.ros.lidar_topic", "/lidar");
  node.get_parameter("lio.ros.lidar_topic", g_lidar_topic);

  node.declare_parameter<std::string>("lio.ros.imu_topic", "/imu");
  node.get_parameter("lio.ros.imu_topic", g_imu_topic);

  node.declare_parameter<int>("lio.sensor.lidar_type", 0);
  node.get_parameter("lio.sensor.lidar_type", g_lidar_type);

  double temp_range_dis;
  node.declare_parameter<double>("lio.sensor.blind", 0.0);
  node.get_parameter("lio.sensor.blind", temp_range_dis);
  g_blind2 = temp_range_dis * temp_range_dis;

  node.declare_parameter<double>("lio.sensor.maxrange", 100.0);
  node.get_parameter("lio.sensor.maxrange", temp_range_dis);
  g_maxrange2 = temp_range_dis * temp_range_dis;

  node.declare_parameter<int>("lio.sensor.filter_rate", 1);
  node.get_parameter("lio.sensor.filter_rate", g_filter_rate);

  node.declare_parameter<bool>("lio.sensor.enable_downsample", false);
  node.get_parameter("lio.sensor.enable_downsample", g_enable_downsample);

  node.declare_parameter<double>("lio.sensor.voxel_fliter_size", 0.2);
  node.get_parameter("lio.sensor.voxel_fliter_size", g_voxel_fliter_size);

  node.declare_parameter<double>("lio.sensor.gravity_norm", 9.81);
  node.get_parameter("lio.sensor.gravity_norm", g_gravity_norm);

  node.declare_parameter<int>("lio.sensor.imu_type", 0);
  node.get_parameter("lio.sensor.imu_type", g_imu_type);

  node.declare_parameter<double>("lio.sensor.imu_na", 0.0);
  node.get_parameter("lio.sensor.imu_na", g_imu_na);

  node.declare_parameter<double>("lio.sensor.imu_ng", 0.0);
  node.get_parameter("lio.sensor.imu_ng", g_imu_ng);

  node.declare_parameter<double>("lio.sensor.imu_nba", 0.0);
  node.get_parameter("lio.sensor.imu_nba", g_imu_nba);

  node.declare_parameter<double>("lio.sensor.imu_nbg", 0.0);
  node.get_parameter("lio.sensor.imu_nbg", g_imu_nbg);

  node.declare_parameter<double>("lio.sensor.imu_max_dt", 0.05);
  node.get_parameter("lio.sensor.imu_max_dt", g_imu_max_dt);

  // ================= extrinsic =================
  std::vector<double> extrinsic_lidar_imu;
  node.declare_parameter<std::vector<double>>(
      "lio.extrinsic.lidar_imu", std::vector<double>(12, 0.0));
  node.get_parameter("lio.extrinsic.lidar_imu", extrinsic_lidar_imu);

  V3 __t(extrinsic_lidar_imu[0],
         extrinsic_lidar_imu[1],
         extrinsic_lidar_imu[2]);
  std::vector<scalar> r_data(9);
  for (int i = 0; i < 9; ++i) {
    r_data[i] = static_cast<scalar>(extrinsic_lidar_imu[3 + i]);
  }
  M3 __R(r_data.data());
  g_lidar_imu = SE3(__R, __t);

  // ---- lidar mounting -------------------------------------------------
  // `base_lidar` is the LIDAR pose in the ROBOT BASE frame (x y z roll pitch
  // yaw, metres and degrees, ZYX) -- the direction robot vendors document, so
  // nothing has to be inverted by hand.
  std::vector<double> base_lidar(6, 0.0);
  node.declare_parameter<std::vector<double>>("lio.extrinsic.base_lidar", base_lidar);
  node.get_parameter("lio.extrinsic.base_lidar", base_lidar);

  node.declare_parameter<bool>("lio.extrinsic.base_lidar_auto_rp", false);
  node.get_parameter("lio.extrinsic.base_lidar_auto_rp", g_base_lidar_auto_rp);

  // Migration: the old `odom_robo` meant the ROBOT frame seen from the LIDAR,
  // i.e. the inverse of this one. Honour it if it is set and base_lidar is not,
  // so existing configs keep working, but say so.
  std::vector<double> odom_robo(6, 0.0);
  node.declare_parameter<std::vector<double>>("lio.extrinsic.odom_robo", odom_robo);
  node.get_parameter("lio.extrinsic.odom_robo", odom_robo);

  auto allZero = [](const std::vector<double>& v) {
    for (double x : v) if (std::abs(x) > 1e-12) return false;
    return true;
  };
  auto rpyToR = [](double roll_deg, double pitch_deg, double yaw_deg) {
    return (Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch_deg * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll_deg * M_PI / 180.0, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
  };

  if (allZero(base_lidar) && !allZero(odom_robo)) {
    LOG(WARNING) << YELLOW
        << " ---> [Extrinsic] lio.extrinsic.odom_robo is deprecated. It means the"
           " ROBOT frame seen from the LIDAR; lio.extrinsic.base_lidar means the"
           " LIDAR seen from the BASE, which is what a robot datasheet gives."
           " Converting automatically -- please migrate the config." << RESET;
    const SE3 T_lidar_base(M3(rpyToR(odom_robo[3], odom_robo[4], odom_robo[5]).cast<scalar>()),
                           V3(odom_robo[0], odom_robo[1], odom_robo[2]));
    g_base_lidar = T_lidar_base.inverse();
  } else {
    g_base_lidar = SE3(M3(rpyToR(base_lidar[3], base_lidar[4], base_lidar[5]).cast<scalar>()),
                       V3(base_lidar[0], base_lidar[1], base_lidar[2]));
  }

  // Gravity can never supply yaw, so keep the configured one for the auto path.
  g_base_lidar_yaw = std::atan2(g_base_lidar.R_(1, 0), g_base_lidar.R_(0, 0));
  RefreshImuRobo();

  {
    const double kDeg = 180.0 / M_PI;
    const M3 R = g_base_lidar.R_;
    LOG(INFO) << GREEN << " ---> [Extrinsic] lidar in base: t=("
              << g_base_lidar.t_.transpose() << ") rpy=("
              << std::atan2(R(2, 1), R(2, 2)) * kDeg << ", "
              << std::asin(std::max(-1.0f, std::min(1.0f, -R(2, 0)))) * kDeg << ", "
              << g_base_lidar_yaw * kDeg << ") deg"
              << (g_base_lidar_auto_rp ? "  [roll/pitch will be re-estimated from gravity]" : "")
              << RESET;
  }

  const std::vector<double>& extrinsic_odom_robo = odom_robo;
  auto temp_R_yaw =
      Eigen::AngleAxisd(extrinsic_odom_robo[5] * M_PI / 180.0,
                        Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  g_lidar_robo_yaw = temp_R_yaw.cast<scalar>();

  // ================= hash map =================
  node.declare_parameter<int>("lio.hash_map.hash_capacity", 1000000);
  node.get_parameter("lio.hash_map.hash_capacity", g_ivox_capacity);

  node.declare_parameter<double>("lio.hash_map.vox_resolution", 0.5);
  node.get_parameter("lio.hash_map.vox_resolution", g_ivox_resolution);

  // kf
  node.declare_parameter<int>("lio.kf.kf_type", 0);
  node.get_parameter("lio.kf.kf_type", g_kf_type);

  node.declare_parameter<int>("lio.kf.kf_max_iterations", 0);
  node.get_parameter("lio.kf.kf_max_iterations", g_kf_max_iterations);

  node.declare_parameter<bool>("lio.kf.kf_align_gravity", false);
  node.get_parameter("lio.kf.kf_align_gravity", g_kf_align_gravity);

  node.declare_parameter<double>("lio.kf.kf_quit_eps", 0.0);
  node.get_parameter("lio.kf.kf_quit_eps", g_kf_quit_eps);

  // submaps
  node.declare_parameter<double>("lio.submap.submap_resolution", 0.0);
  node.get_parameter("lio.submap.submap_resolution", g_submap_resolution);

  node.declare_parameter<int>("lio.submap.submap_capacity", 0);
  node.get_parameter("lio.submap.submap_capacity", g_submap_capacity);

  // visual
  node.declare_parameter<bool>("lio.output.robot", false);
  node.get_parameter("lio.output.robot", g_2_robot);

  node.declare_parameter<bool>("lio.output.planner", false);
  node.get_parameter("lio.output.planner", g_planner_enable);

  node.declare_parameter<bool>("lio.output.plan_env_world", false);
  node.get_parameter("lio.output.plan_env_world", g_2_plan_env_world);

  node.declare_parameter<bool>("lio.output.plan_env_body", false);
  node.get_parameter("lio.output.plan_env_body", g_2_plan_env_body);

  node.declare_parameter<bool>("lio.output.ml_map", false);
  node.get_parameter("lio.output.ml_map", g_2_ml_map);

  node.declare_parameter<bool>("lio.output.map", false);
  node.get_parameter("lio.output.map", g_visual_map);

  node.declare_parameter<bool>("lio.output.dense", false);
  node.get_parameter("lio.output.dense", g_visual_dense);

  node.declare_parameter<int>("lio.output.pub_step", 0);
  node.get_parameter("lio.output.pub_step", g_pub_step);

  // ================= relocation =================
  node.declare_parameter<bool>("lio.relocation.update_map", false);
  node.get_parameter("lio.relocation.update_map", g_update_map);

  node.declare_parameter<double>("lio.relocation.local_map_radius", 10.0);
  node.get_parameter("lio.relocation.local_map_radius", g_reloc_local_map_radius);

  std::vector<double> init_pose;
  node.declare_parameter<std::vector<double>>(
      "lio.relocation.init_pose", std::vector<double>(6, 0.0));
  node.get_parameter("lio.relocation.init_pose", init_pose);

  g_init_px    = init_pose[0];
  g_init_py    = init_pose[1];
  g_init_pz    = init_pose[2];
  g_init_roll  = init_pose[3];
  g_init_pitch = init_pose[4];
  g_init_yaw   = init_pose[5];

  LOG(INFO) << GREEN << " ---> [Params]: Load from ROS2 parameter server."
            << RESET;
}


#ifdef HAVE_LIVOX_ROS_DRIVER2
void livox2pcl(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg, CloudPtr& point_cloud){
  point_cloud->clear();
  CloudPtr cloud_full(new PointCloudType());
  int plsize = msg->point_num;
  cloud_full->resize(plsize);
  point_cloud->reserve(plsize);
  std::vector<bool> is_valid_pt(plsize, false);
  std::vector<std::size_t> index(plsize - 1);
  std::iota(std::begin(index), std::end(index), 1);

  std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const uint &i) {
    if((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)
    {
      // if (i % g_filter_rate == 0) 
      {
        cloud_full->at(i).x = msg->points[i].x;
        cloud_full->at(i).y = msg->points[i].y;
        cloud_full->at(i).z = msg->points[i].z;
        cloud_full->at(i).intensity = msg->points[i].reflectivity;

        if ((abs(cloud_full->at(i).x - cloud_full->at(i - 1).x) > 1e-7) ||
            (abs(cloud_full->at(i).y - cloud_full->at(i - 1).y) > 1e-7) ||
            (abs(cloud_full->at(i).z - cloud_full->at(i - 1).z) > 1e-7))
        {
          double normal_dis = cloud_full->at(i).x * cloud_full->at(i).x + 
                              cloud_full->at(i).y * cloud_full->at(i).y +
                              cloud_full->at(i).z * cloud_full->at(i).z;
          if(normal_dis > g_blind2 and normal_dis < g_maxrange2){
            is_valid_pt[i] = true;
          }
        }
      }
    }
  });

  for (int i = 1; i < plsize; i++) {
    if (is_valid_pt[i]) {
      point_cloud->points.push_back(cloud_full->at(i));
    }
  }
}
#endif  // HAVE_LIVOX_ROS_DRIVER2


std::string lidarTypeToString(int type) {
  if (type <= 0 || type >= static_cast<int>(LID_TYPE_NAMES.size())) return "UNKNOWN";
  return LID_TYPE_NAMES[type];
}


inline bool validPoint(double x, double y, double z)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
    return false;

  double d2 = x * x + y * y + z * z;
  return (d2 > g_blind2 && d2 < g_maxrange2);
}


inline double stampToSec(const builtin_interfaces::msg::Time& t)
{
  return static_cast<double>(t.sec) +
         static_cast<double>(t.nanosec) * 1e-9;
}


inline builtin_interfaces::msg::Time toRosTime(double t_sec)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(std::floor(t_sec));
  t.nanosec = static_cast<uint32_t>((t_sec - t.sec) * 1e9);
  return t;
}


ROSWrapper::ROSWrapper(const rclcpp::NodeOptions& options)
: rclcpp::Node("super_lio", options)
{
  LoadParamFromRos(*this);
  LOG(INFO) << GREEN << " ---> Using Lidar type: " << lidarTypeToString(g_lidar_type)
            << " | lidar '" << g_lidar_topic << "' as "
            << (g_lidar_type == LID_TYPE::LIVOX ? "livox_ros_driver2/CustomMsg"
                                                : "sensor_msgs/PointCloud2")
            << " | imu '" << g_imu_topic << "'" << RESET;

  msg2uav_.header.frame_id = "world";
  path_.header.frame_id = "world";

  setupIO();
}


void ROSWrapper::setupIO(){
  //// input ======================================
  cb_sensor_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions sub_opt;
  sub_opt.callback_group = cb_sensor_;

  auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(500))
                 .best_effort()
                 .durability_volatile();

  auto lidar_qos = rclcpp::QoS(rclcpp::KeepLast(20))
                   .best_effort()
                   .durability_volatile();

  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      g_imu_topic,
      imu_qos,
      std::bind(&ROSWrapper::imuHandler, this, std::placeholders::_1),
      sub_opt);

  if (g_lidar_type == LID_TYPE::LIVOX) {
#ifdef HAVE_LIVOX_ROS_DRIVER2
    sub_lidar_ =
        this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            g_lidar_topic,
            lidar_qos,
            std::bind(&ROSWrapper::livoxHandler, this, std::placeholders::_1),
            sub_opt);
#else
    // Built without the Livox interfaces, so livox_ros_driver2/CustomMsg cannot
    // be deserialized at all. Say so and stop: subscribing to the topic with any
    // other type would match nothing and leave a node that is alive and silent.
    LOG(FATAL) << RED
               << " ---> [Lidar] lio.sensor.lidar_type=LIVOX, but this build has no"
                  " livox_ros_driver2. Source the Livox driver workspace and rebuild"
                  " super_lio, or set lidar_type to the PointCloud2 type this rig"
                  " actually publishes." << RESET;
#endif
  } else {
    sub_lidar_std_ =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
            g_lidar_topic,
            lidar_qos,
            std::bind(&ROSWrapper::stdMsgHandler, this, std::placeholders::_1),
            sub_opt);
  }

  input_watchdog_ = this->create_wall_timer(
      std::chrono::seconds(3), std::bind(&ROSWrapper::checkInputs, this));

  /// output ======================================
  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/lio/odom", 100);

  pub_imu_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/lio/imu/odom", 10);

  pub_robo_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/lio/robo/odom", 10);

  pub_path_ = this->create_publisher<nav_msgs::msg::Path>(
      "/lio/path", 10);

  pub_cloud_world_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lio/cloud_world", 10);

  tf_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(this);
}


void ROSWrapper::imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg){
  ++n_imu_msgs_;
  IMUData data;
  data.secs = stampToSec(msg->header.stamp);
  data.acc  = V3(msg->linear_acceleration.x,
                 msg->linear_acceleration.y,
                 msg->linear_acceleration.z);
  data.gyr  = V3(msg->angular_velocity.x,
                 msg->angular_velocity.y,
                 msg->angular_velocity.z);

  if (data.secs < last_timestamp_imu_) {
    LOG(WARNING) << "imu loop back, clear buffer";
    imu_buffer_.clear();
    imu_buffer_.push_back(data);
    last_timestamp_imu_ = data.secs;
    // eskf_->Reset();   // todo:
    return;
  }

  imu_buffer_.push_back(data);
  last_timestamp_imu_ = data.secs;

  DynamicState imu_state, robo_state;
  if(eskf_->Predict(data, imu_state, robo_state)){
    nav_msgs::msg::Odometry odom_imu, odom_robo;

    {
      odom_imu.pose.pose.position.x = imu_state.p(0);
      odom_imu.pose.pose.position.y = imu_state.p(1);
      odom_imu.pose.pose.position.z = imu_state.p(2);

      Quat q(imu_state.R);
      q.normalize();

      odom_imu.pose.pose.orientation.x = q.x();
      odom_imu.pose.pose.orientation.y = q.y();
      odom_imu.pose.pose.orientation.z = q.z();
      odom_imu.pose.pose.orientation.w = q.w();

      odom_imu.twist.twist.linear.x = imu_state.v(0);
      odom_imu.twist.twist.linear.y = imu_state.v(1);
      odom_imu.twist.twist.linear.z = imu_state.v(2);

      odom_imu.twist.twist.angular.x = imu_state.w(0);
      odom_imu.twist.twist.angular.y = imu_state.w(1);
      odom_imu.twist.twist.angular.z = imu_state.w(2);
    }

    {
      odom_robo.pose.pose.position.x = robo_state.p(0);
      odom_robo.pose.pose.position.y = robo_state.p(1);
      odom_robo.pose.pose.position.z = robo_state.p(2);

      Quat q(robo_state.R);
      q.normalize();

      odom_robo.pose.pose.orientation.x = q.x();
      odom_robo.pose.pose.orientation.y = q.y();
      odom_robo.pose.pose.orientation.z = q.z();
      odom_robo.pose.pose.orientation.w = q.w();

      // Twist of the BASE origin, expressed in the ODOM frame -- deliberately
      // not the REP-105 child-frame twist. Two reasons, both about staying
      // consistent with what already exists rather than being locally right:
      //   * /lio/odom and /lio/imu/odom have always published a world-frame
      //     linear velocity under child_frame_id "imu";
      //   * the navigation consumer of the robot odometry reads the twist as
      //     world-frame and rotates it into the base frame itself
      //     (rl_nav_controller's convert_vel_frame). Publishing base-frame here
      //     would make it rotate twice and silently corrupt the policy input.
      // If the convention is ever flipped, flip all the odometry topics
      // together and fix that consumer in the same change.
      odom_robo.twist.twist.linear.x = robo_state.v(0);
      odom_robo.twist.twist.linear.y = robo_state.v(1);
      odom_robo.twist.twist.linear.z = robo_state.v(2);

      odom_robo.twist.twist.angular.x = robo_state.w(0);
      odom_robo.twist.twist.angular.y = robo_state.w(1);
      odom_robo.twist.twist.angular.z = robo_state.w(2);
    }

    odom_imu.header.stamp = msg->header.stamp;
    odom_robo.header.stamp = msg->header.stamp;
    odom_imu.header.frame_id = "world";
    odom_robo.header.frame_id = "world";
    // These were never set, so both messages went out with an empty
    // child_frame_id -- which is exactly the field that tells a consumer which
    // body the pose and twist describe.
    odom_imu.child_frame_id = "imu";
    odom_robo.child_frame_id = "base";
    pub_imu_odom_->publish(odom_imu);
    pub_robo_odom_->publish(odom_robo);

    // Map localization, when running, turns the same propagated state into
    // /loc/imu/* using the drift it has already applied at 10 Hz.
    if (imu_rate_hook_) imu_rate_hook_(imu_state, msg->header.stamp);
  }
}


#ifdef HAVE_LIVOX_ROS_DRIVER2
void ROSWrapper::livoxHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg){
  if(msg->point_num < 10) return;
  ++n_lidar_msgs_;
  announceFirstLidar(msg->point_num);
  LidarData lidar_data;
  std::size_t ptsize = msg->point_num;
  lidar_data.pc.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());
  lidar_data.pc->reserve(ptsize / g_filter_rate + 1);

  double offset_time = 0.0;
  for(std::size_t _i = 0; _i < ptsize; _i += g_filter_rate){
    auto& pt = msg->points[_i];
    auto tag = pt.tag & 0x30;
    if (tag == 0x10 || tag == 0x00){
      auto dis = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
      if(dis > g_blind2 && dis < g_maxrange2){
        offset_time = pt.offset_time * 1e-9;
        lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, pt.reflectivity, offset_time);
      }
    }
  }
  lidar_data.start_time = stampToSec(msg->header.stamp);
  lidar_data.end_time   = lidar_data.start_time + offset_time;
  lidar_buffer_.push_back(lidar_data);
}
#endif  // HAVE_LIVOX_ROS_DRIVER2


void ROSWrapper::announceFirstLidar(std::size_t points) {
  if (announced_lidar_) return;
  announced_lidar_ = true;
  LOG(INFO) << GREEN << " ---> [Lidar] first message on " << g_lidar_topic << ": "
            << points << " points, type " << lidarTypeToString(g_lidar_type) << RESET;
}

void ROSWrapper::announceFirstLidar(const sensor_msgs::msg::PointCloud2& msg) {
  if (announced_lidar_) return;
  announced_lidar_ = true;

  // The field list is the single most useful line of a board bring-up. Every
  // non-Livox branch below casts the message into a point struct with
  // pcl::fromROSMsg, which matches BY FIELD NAME and silently leaves anything
  // missing at zero rather than failing. When the cloud comes from a fusion or
  // relay node rather than the vendor driver, `timestamp` and `ring` are
  // exactly the fields that get dropped -- and an all-zero `timestamp` means no
  // de-skewing and a scan time at the epoch, with nothing in the log to say so.
  // Printing what actually arrived separates "wrong topic" from "right topic,
  // wrong fields" in one line.
  std::ostringstream fields;
  for (const auto& f : msg.fields) fields << f.name << " ";
  LOG(INFO) << GREEN << " ---> [Lidar] first message on " << g_lidar_topic << ": "
            << msg.width * msg.height << " points, type "
            << lidarTypeToString(g_lidar_type) << ", fields: " << fields.str() << RESET;

  const bool has_time = std::any_of(
      msg.fields.begin(), msg.fields.end(),
      [](const sensor_msgs::msg::PointField& f) {
        return f.name == "timestamp" || f.name == "time" || f.name == "t" ||
               f.name == "offset_time";
      });
  if (!has_time) {
    LOG(WARNING) << RED
                 << " ---> [Lidar] no per-point time field in this cloud."
                    " Undistortion will be a no-op and the scan time will be wrong."
                    " Subscribe to the driver's own topic, or make the upstream node"
                    " preserve the per-point timestamp." << RESET;
  }
}

void ROSWrapper::checkInputs() {
  // Subscribing to the wrong topic, or to the right topic with the wrong
  // lidar_type, produces a node that is alive, silent and doing nothing. There
  // is no error to see because nothing failed -- the subscription simply never
  // matches. So say it out loud, and say it with the values that were actually
  // in effect, because the usual cause is a config edit that never reached the
  // running node.
  const bool no_lidar = (n_lidar_msgs_ == 0);
  const bool no_imu   = (n_imu_msgs_ == 0);
  if (!no_lidar && !no_imu) {
    silent_reports_ = 0;
    return;
  }
  // Loud at first, then every ~30 s, so a genuinely idle rig does not scroll.
  ++silent_reports_;
  if (silent_reports_ > 3 && (silent_reports_ % 10) != 0) return;

  LOG(WARNING) << RED << " ---> [Lidar] no data yet:"
               << (no_lidar ? std::string(" lidar '") + g_lidar_topic + "' (" +
                                  lidarTypeToString(g_lidar_type) + ", expecting " +
                                  (g_lidar_type == LID_TYPE::LIVOX
                                       ? "livox_ros_driver2/CustomMsg"
                                       : "sensor_msgs/PointCloud2") + ")"
                            : std::string())
               << (no_imu ? std::string(" imu '") + g_imu_topic + "'" : std::string())
               << " -- check `ros2 topic list`, the message TYPE (set by"
                  " lio.sensor.lidar_type), and that the config you edited is the"
                  " one this node loaded." << RESET;
}

void ROSWrapper::stdMsgHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
  if(msg->data.size() < 10) return;
  ++n_lidar_msgs_;
  announceFirstLidar(*msg);

  LidarData lidar_data;
  lidar_data.pc.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());

  double offset_time = 0.0;
  double dis = 0.0;

  switch (g_lidar_type) {

  case LID_TYPE::HESAI16:
  {
    pcl::PointCloud<hesai_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    if (pl_orig.empty()) return;   // points[0] below is otherwise out of bounds
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    const double time_begin = pl_orig.points[0].timestamp;
    // A Hesai timestamp is absolute wall-clock seconds. Anything near zero means
    // the field was not in the message and fromROSMsg zero-filled it, which
    // would hand the frontend a scan starting at the epoch.
    if (time_begin < 1e8) {
      LOG_EVERY_N(WARNING, 100) << RED
          << " ---> [Lidar] Hesai per-point timestamp is " << time_begin
          << ", not wall-clock seconds; the `timestamp` field is missing or zeroed"
             " upstream. Scan times and undistortion are both wrong." << RESET;
    }
    lidar_data.start_time = time_begin;
    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate)
    {
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      offset_time = pt.timestamp - time_begin;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, pt.intensity, offset_time);
    }
    lidar_data.end_time = time_begin + offset_time;
    break;
  }
  case LID_TYPE::VEL_NCLT:
  {
    pcl::PointCloud<NCLT::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = stampToSec(msg->header.stamp);
    
    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate){
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      offset_time = pt.time * 1e-6;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, 1.0, offset_time);
    }
    lidar_data.end_time = lidar_data.start_time + offset_time;
    break;
  }
  case LID_TYPE::VELO16:
  case LID_TYPE::VELO32:
  {
    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = stampToSec(msg->header.stamp);

    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate){
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, pt.intensity, pt.time);
    }
    lidar_data.end_time = lidar_data.start_time + lidar_data.pc->points.back().offset_time;
    break;
  }
  case OUSTER:
  {
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = stampToSec(msg->header.stamp);

    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate){
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      offset_time = pt.t * 1e-9;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, pt.intensity, offset_time);
    }
    lidar_data.end_time = lidar_data.start_time + offset_time;
    break;
  }
  default:
    return;
  }
  
  lidar_buffer_.push_back(lidar_data);
}


bool ROSWrapper::sync_measure(MeasureGroup& meas){
  if (lidar_buffer_.empty() || imu_buffer_.empty()) {
    return false;
  }

  if (!lidar_pushed_) {
    meas.lidar = lidar_buffer_.front();
    lidar_pushed_ = true;
  }

  if(last_timestamp_lidar_ > meas.lidar.end_time){
    lidar_buffer_.pop_front();
    lidar_pushed_ = false;
    return false;
  }

  if (last_timestamp_imu_ < meas.lidar.end_time) {
    return false;
  }

  double imu_time = imu_buffer_.front().secs;
  meas.imu.clear();
  while ((!imu_buffer_.empty()) && (imu_time < meas.lidar.end_time)) {
    imu_time = imu_buffer_.front().secs;
    if (imu_time > meas.lidar.end_time) break;
    meas.imu.push_back(imu_buffer_.front());
    imu_buffer_.pop_front();
  }

  last_timestamp_lidar_ = meas.lidar.end_time;
  lidar_buffer_.pop_front();
  lidar_pushed_ = false;
  return true;
}


void ROSWrapper::pub_odom(const NavState& state){
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "world";
  odom.child_frame_id = "imu";

  odom.header.stamp = toRosTime(state.timestamp);
  odom.pose.pose.position.x = state.p[0];
  odom.pose.pose.position.y = state.p[1];
  odom.pose.pose.position.z = state.p[2];

  V4 temp_q = state.R.coeffs();
  odom.pose.pose.orientation.x = temp_q[0];
  odom.pose.pose.orientation.y = temp_q[1];
  odom.pose.pose.orientation.z = temp_q[2];
  odom.pose.pose.orientation.w = temp_q[3];

  odom.twist.twist.linear.x = state.v[0];
  odom.twist.twist.linear.y = state.v[1];
  odom.twist.twist.linear.z = state.v[2];

  pub_odom_->publish(odom);    // imu frame -> lidar frequency

  V3 robo_position = state.p;

  // if(g_2_robot){
  //   static auto pub_msg2uav_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
  //       "/mavros/vision_pose/pose", 10);
  //   M3 robo_rotation = state.R.R_ * g_odom_robo.R_;
  //   msg2uav_.header.stamp = odom.header.stamp;
  //   msg2uav_.pose.position.x = robo_position[0];
  //   msg2uav_.pose.position.y = robo_position[1];
  //   msg2uav_.pose.position.z = robo_position[2];
  //   Quat robo_quat(robo_rotation);
  //   msg2uav_.pose.orientation.w = robo_quat.w();
  //   msg2uav_.pose.orientation.x = robo_quat.x();
  //   msg2uav_.pose.orientation.y = robo_quat.y();
  //   msg2uav_.pose.orientation.z = robo_quat.z();
  //   pub_msg2uav_->publish(msg2uav_);
  // }

  if (g_2_robot) {
    static auto pub_robo_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/dlio/odom_node/odom", 10);
    
    // 2. 声明 Odometry 消息
    nav_msgs::msg::Odometry odom_msg;

    // 计算旋转
    // The robot centre, not the IMU position with the robot's orientation.
    // Downstream planning consumes this pose, so the lever arm matters as soon
    // as the sensor is not mounted at the robot origin.
    const SE3 T_robot = imuToRobot(state.GetSE3());
    M3 robo_rotation = T_robot.R_;
    robo_position = T_robot.t_;
    Quat robo_quat(robo_rotation);

    // 3. 填充 Header
    odom_msg.header.stamp = odom.header.stamp;
    odom_msg.header.frame_id = "world";       // 通常里程计坐标系为 odom
    odom_msg.child_frame_id = "base";   // 

    // 4. 填充 Pose (位置)
    odom_msg.pose.pose.position.x = robo_position[0];
    odom_msg.pose.pose.position.y = robo_position[1];
    odom_msg.pose.pose.position.z = robo_position[2];

    // 5. 填充 Pose (姿态/四元数)
    odom_msg.pose.pose.orientation.w = robo_quat.w();
    odom_msg.pose.pose.orientation.x = robo_quat.x();
    odom_msg.pose.pose.orientation.y = robo_quat.y();
    odom_msg.pose.pose.orientation.z = robo_quat.z();

    // 5b. 填充 Twist. This topic is what the navigation policy subscribes to,
    // and it used to go out with an all-zero twist -- i.e. the controller has
    // been reading "the robot is not moving" the whole time.
    //
    // Same convention as the IMU-rate /lio/robo/odom: the BASE origin's
    // velocity, expressed in the ODOM frame, because the consumer rotates it
    // into the base frame itself. NavState carries no angular rate, so take the
    // filter's latest bias-corrected gyro; at 10 Hz that is the rate at the end
    // of the scan, which is the instant this pose belongs to.
    if (eskf_) {
      // state.v is already in the odom frame, so the output frame here IS the
      // odom frame and R_out_imu is just the attitude.
      const BaseTwist tw = baseTwist(state.R.R(), state.v,
                                     eskf_->GetDynamicState().w);

      odom_msg.twist.twist.linear.x = tw.v[0];
      odom_msg.twist.twist.linear.y = tw.v[1];
      odom_msg.twist.twist.linear.z = tw.v[2];

      odom_msg.twist.twist.angular.x = tw.w[0];
      odom_msg.twist.twist.angular.y = tw.w[1];
      odom_msg.twist.twist.angular.z = tw.w[2];
    }

    // 6. 发布消息
    pub_robo_odom_->publish(odom_msg);

    geometry_msgs::msg::TransformStamped tf_msg_base;

    tf_msg_base.header.stamp = odom.header.stamp;
    tf_msg_base.header.frame_id = "world";
    tf_msg_base.child_frame_id = "base";

    tf_msg_base.transform.translation.x = robo_position[0];
    tf_msg_base.transform.translation.y = robo_position[1];
    tf_msg_base.transform.translation.z = robo_position[2];

    tf_msg_base.transform.rotation.x = robo_quat.x();
    tf_msg_base.transform.rotation.y = robo_quat.y();
    tf_msg_base.transform.rotation.z = robo_quat.z();
    tf_msg_base.transform.rotation.w = robo_quat.w();

    tf_broadcaster_->sendTransform(tf_msg_base);
  }

  if((last_path_point_ - robo_position).norm() > 0.1)
  {
    path_.header.stamp = odom.header.stamp;
    geometry_msgs::msg::PoseStamped point;
    point.pose = odom.pose.pose;
    path_.poses.push_back(point);
    pub_path_->publish(path_);
    last_path_point_ = robo_position;
  }

  geometry_msgs::msg::TransformStamped tf_msg;

  tf_msg.header.stamp = odom.header.stamp;
  tf_msg.header.frame_id = "world";
  tf_msg.child_frame_id = "imu";

  tf_msg.transform.translation.x = state.p[0];
  tf_msg.transform.translation.y = state.p[1];
  tf_msg.transform.translation.z = state.p[2];

  tf_msg.transform.rotation.x = temp_q.x();
  tf_msg.transform.rotation.y = temp_q.y();
  tf_msg.transform.rotation.z = temp_q.z();
  tf_msg.transform.rotation.w = temp_q.w();

  tf_broadcaster_->sendTransform(tf_msg);

  // tf_msg.child_frame_id = "god";
  // tf_msg.transform.rotation.x = 0.0;
  // tf_msg.transform.rotation.y = 0.0;
  // tf_msg.transform.rotation.z = 0.0;
  // tf_msg.transform.rotation.w = 1.0;
  // tf_broadcaster_->sendTransform(tf_msg);

}


void ROSWrapper::pub_cloud_world(const CloudPtr& pc, double time){
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = "world";
  cloud.header.stamp = toRosTime(time);
  pub_cloud_world_->publish(cloud);
}


void ROSWrapper::pub_cloud2planner(const CloudPtr& pc, double time){
  static auto pub_cloud2robot_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lio/robo/cloud_world", 10);
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = "world";
  cloud.header.stamp = toRosTime(time);
  pub_cloud2robot_->publish(cloud);
}


void ROSWrapper::pub_cloud_body_pose(const CloudPtr& pc, 
  const NavState& state)
{
  static auto pub_cloud_body_pose_ =
    this->create_publisher<super_lio::msg::CloudPose>(
        "/lio/body/cloud_pose", 10);
  super_lio::msg::CloudPose cloud_pose;
  pcl::toROSMsg(*pc, cloud_pose.cloud);
  cloud_pose.cloud.header.stamp = toRosTime(state.timestamp); 
  cloud_pose.pose.position.x = state.p[0];
  cloud_pose.pose.position.y = state.p[1];
  cloud_pose.pose.position.z = state.p[2];
  V4 temp_q = state.R.coeffs();
  cloud_pose.pose.orientation.x = temp_q[0];
  cloud_pose.pose.orientation.y = temp_q[1];
  cloud_pose.pose.orientation.z = temp_q[2];
  cloud_pose.pose.orientation.w = temp_q[3];

  pub_cloud_body_pose_->publish(cloud_pose);
}


void ROSWrapper::pub_cloud_world_pose(const CloudPtr& pc, 
   const NavState& state)
{
  static auto pub_cloud_world_pose_ =
    this->create_publisher<super_lio::msg::CloudPose>(
        "/lio/world/cloud_pose", 10);
  super_lio::msg::CloudPose cloud_pose;
  pcl::toROSMsg(*pc, cloud_pose.cloud);
  cloud_pose.cloud.header.stamp = toRosTime(state.timestamp);  
  cloud_pose.pose.position.x = state.p[0];
  cloud_pose.pose.position.y = state.p[1];
  cloud_pose.pose.position.z = state.p[2];
  V4 temp_q = state.R.coeffs();
  cloud_pose.pose.orientation.x = temp_q[0];
  cloud_pose.pose.orientation.y = temp_q[1];
  cloud_pose.pose.orientation.z = temp_q[2];
  cloud_pose.pose.orientation.w = temp_q[3];
  pub_cloud_world_pose_->publish(cloud_pose);
}


void ROSWrapper::pub_processing_time(double time, 
  double current_time, double mean_time, double std_time)
{
  static auto pub_processing_time_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/lio/processing_time", 10);
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = toRosTime(time);
  msg.pose.position.x = current_time;
  msg.pose.position.y = mean_time;
  msg.pose.position.z = std_time;
  pub_processing_time_->publish(msg);
}


void ROSWrapper::set_global_map(const BASIC::CloudPtr& global_map){
  pcl::toROSMsg(*global_map, global_map_msg_);
  global_map_msg_.header.frame_id = "world";

  static auto global_map_pub =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/lio/global_map", 10);

  static auto global_map_timer =
    this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        static int count = -1;
        static int publish_interval = 1;

        count++;
        if (count % publish_interval != 0) {
          return;
        }

        count = 0;
        publish_interval++;
        if (publish_interval > 10) {
          publish_interval = 10;
        }
        global_map_msg_.header.stamp = this->now();
        global_map_pub->publish(global_map_msg_);
      });
}


void ROSWrapper::set_initial_data(BASIC::SE3& init_pose, bool& flg_get_init_guess, bool flg_finish_init)
{
  static auto init_pose_sub =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1,
        [this, &init_pose, &flg_get_init_guess](
          const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) 
        {
          // A clicked pose is a ROBOT pose; the filter wants the IMU pose.
          // robotToImu() is the exact inverse of imuToRobot(), which the
          // hand-written version here was not (wrong sign on the lever arm).
          double qx = msg->pose.pose.orientation.x;
          double qy = msg->pose.pose.orientation.y;
          double qz = msg->pose.pose.orientation.z;
          double qw = msg->pose.pose.orientation.w;

          Quat robot_q(qw, qx, qy, qz);
          M3 R_robot = robot_q.toRotationMatrix().cast<scalar>();

          V3 p_robot;
          p_robot << (scalar)msg->pose.pose.position.x,
                     (scalar)msg->pose.pose.position.y,
                     (scalar)msg->pose.pose.position.z + 0.79;

          const SE3 T_imu = robotToImu(SE3(SO3(R_robot), p_robot));
          const M3 R_lidar = T_imu.R_;
          const V3 p_lidar = T_imu.t_;

          init_pose = T_imu;

          flg_get_init_guess = true;

          LOG(INFO) << YELLOW
                  << " ---> GET Initial guess (robot frame): "
                  << "xyz: " << p_robot.transpose()
                  << " euler(RPY): "
                  << R_robot.eulerAngles(0, 1, 2).transpose()
                  << RESET;
          LOG(INFO) << YELLOW
                  << " ---> GET Initial guess (lidar frame): "
                  << "xyz: " << p_lidar.transpose()
                  << " euler(RPY): "
                  << R_lidar.eulerAngles(0, 1, 2).transpose()
                  << RESET;
        });

  if (flg_finish_init) {
    init_pose_sub.reset();
  }
}


} // namespace END.
