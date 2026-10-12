

#include "lio/params.h"

#include "basic/logs.h"

using namespace std;
using namespace BASIC;

namespace LI2Sup{

  const std::string g_root_dir = std::string(ROOT);
  std::atomic<bool> g_flag_run = true; 
  bool g_flg_map_init = true;

  /// evaluation
  bool g_time_eva = false;

  bool   g_save_map;
  bool   g_if_filter; 
  string g_save_map_dir;
  string g_map_name;
  float  g_map_ds_size;
  int    g_pcd_save_interval;
  
  string g_imu_topic;
  string g_lidar_topic;

  int    g_lidar_type;
  float  g_blind2;
  float  g_maxrange2;
  int    g_filter_rate;
  bool   g_enable_downsample;
  float  g_voxel_fliter_size;

  int    g_imu_type;
  double g_gravity_norm = 9.7946;
  double g_imu_na;
  double g_imu_ng;
  double g_imu_nba;
  double g_imu_nbg;
  double g_imu_max_dt = 0.05;

  SE3 g_lidar_imu;
  SE3 g_base_lidar;
  SE3 g_imu_robo;
  bool g_base_lidar_auto_rp = false;
  double g_base_lidar_yaw = 0.0;
  M3  g_lidar_robo_yaw;

  /// hash_map
  std::size_t g_ivox_capacity = 100000;
  float       g_ivox_resolution = 0.5;

  /// kf
  int g_kf_type = 1;                // 1: ESKF, 2: InESKF
  int g_kf_max_iterations = 4;
  bool g_kf_align_gravity = true;
  double g_kf_quit_eps;

  /// submap 
  double g_submap_resolution;
  int    g_submap_capacity;

  /// output
  bool g_2_robot    = false;
  bool g_2_plan_env_world = false; 
  bool g_2_plan_env_body  = false;
  bool g_2_ml_map = false;
  bool g_visual_map = true;
  bool g_visual_dense = false;
  int  g_pub_step;

  /// for planner
  bool g_planner_enable;

  ResidualType g_residual_type = PROB;

  /// for relocation
  bool g_update_map = false;
  double g_init_px, g_init_py, g_init_pz, g_init_roll, g_init_pitch, g_init_yaw;
  double g_reloc_local_map_radius = 60.0;


  void RefreshImuRobo() {
    // T_base_imu = T_base_lidar * T_lidar_imu, and g_lidar_imu is T_B_L, so
    // T_lidar_imu is its inverse. g_imu_robo is what consumers want: the base
    // frame seen from the IMU.
    g_imu_robo = (g_base_lidar * g_lidar_imu.inverse()).inverse();
  }

  void CalibrateBaseLidarFromGravity(const BASIC::SE3& T_O_B) {
    if (!g_base_lidar_auto_rp) return;

    // The odometry frame is gravity aligned (kf_init averages a static
    // accelerometer window and zeroes yaw), so the roll/pitch of an early
    // stationary pose IS the sensor's tilt against level. Strip whatever yaw
    // has crept in and keep only the tilt.
    const Eigen::Matrix3d R = T_O_B.R_.cast<double>();
    const double yaw = std::atan2(R(1, 0), R(0, 0));
    const Eigen::Matrix3d R_tilt =
        Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() * R;

    // The IMU's attitude in the base frame: measured tilt, yaw from the yaml
    // (gravity says nothing about yaw).
    const Eigen::Matrix3d R_base_imu =
        Eigen::AngleAxisd(g_base_lidar_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
        R_tilt;
    // Back out to the lidar, since g_base_lidar is what we store.
    const Eigen::Matrix3d R_base_lidar =
        R_base_imu * g_lidar_imu.R_.cast<double>();

    g_base_lidar = SE3(M3(R_base_lidar.cast<scalar>()), g_base_lidar.t_);
    RefreshImuRobo();

    const double kDeg = 180.0 / M_PI;
    const double roll  = std::atan2(R_tilt(2, 1), R_tilt(2, 2)) * kDeg;
    const double pitch = std::asin(std::max(-1.0, std::min(1.0, -R_tilt(2, 0)))) * kDeg;
    LOG(INFO) << GREEN << " ---> [Extrinsic] gravity-calibrated lidar tilt: roll "
              << roll << " deg, pitch " << pitch << " deg (yaw "
              << g_base_lidar_yaw * kDeg << " deg from yaml)" << RESET;
  }
}
