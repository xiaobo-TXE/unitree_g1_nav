/**
 * @file parameters.hpp
 * @author WangLiansheng (lswang@mail.ecust.edu.cn)
 * @date 2023-03-14
 * @copyright Copyright (c) 2023
 */


#ifndef PARAMETERS_HPP_
#define PARAMETERS_HPP_


#include <atomic>
#include <string>
#include <vector>
#include <mutex>
#include <shared_mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "basic/alias.h"
#include "basic/Manifold.h"


namespace LI2Sup{
  
  extern const std::string g_root_dir;
  extern std::atomic<bool> g_flag_run;
  extern bool g_flg_map_init;

  /// evaluation
  extern bool g_time_eva;

  extern bool g_save_map;
  extern bool g_if_filter;
  extern std::string g_map_name;
  extern std::string g_save_map_dir;
  extern float g_map_ds_size;
  extern int   g_pcd_save_interval;
  
  extern std::string g_imu_topic;
  extern std::string g_lidar_topic;

  extern int   g_lidar_type;       // 1: mid360, 2: hesai16, 3: velo16, 4: velo32, 5: vel_nclt, 6: ls16 
  extern float g_blind2;
  extern float g_maxrange2;
  extern int   g_filter_rate;
  extern bool  g_enable_downsample;
  extern float g_voxel_fliter_size;

  extern int    g_imu_type;
  extern double g_gravity_norm;
  extern double g_imu_na;
  extern double g_imu_ng;
  extern double g_imu_nba;
  extern double g_imu_nbg;

  /// [s] Longest IMU step the ESKF will integrate blind. See ESKF::Options.
  extern double g_imu_max_dt;

  extern BASIC::SE3 g_lidar_imu;      // T_B_L: lidar frame expressed in the IMU frame

  /// T_base_L: the LIDAR pose expressed in the ROBOT BASE frame.
  ///
  /// This is `lio.extrinsic.base_lidar`, and it is deliberately the direction a
  /// robot vendor documents: "the lidar sits at (x, y, z) with attitude
  /// (roll, pitch, yaw) on the robot". Nothing has to be inverted by hand.
  ///
  /// Base frame is the usual robot convention (REP-103): x forward, y left,
  /// z up.
  extern BASIC::SE3 g_base_lidar;

  /// T_B_robot: the base frame expressed in the IMU frame. Derived; this is
  /// what every consumer actually needs, because the filter state is the IMU
  /// pose.
  ///
  ///   T_base_imu = g_base_lidar * g_lidar_imu^-1
  ///   g_imu_robo = T_base_imu^-1
  extern BASIC::SE3 g_imu_robo;

  /// Estimate the lidar's roll/pitch from gravity instead of trusting the yaml.
  ///
  /// Roll and pitch are exactly the part of a mounting that is easy to get
  /// wrong and easy to measure: with the robot standing level and still, the
  /// accelerometer already says which way is down, so the sensor's tilt falls
  /// out of the same static window the ESKF uses for its own gravity
  /// alignment. Yaw cannot be recovered this way (gravity is rotationally
  /// symmetric about it), so yaw always comes from the yaml.
  extern bool g_base_lidar_auto_rp;

  /// Yaw of the lidar in the base frame, kept separately because the auto
  /// path rebuilds the rotation and needs it.
  extern double g_base_lidar_yaw;

  /// Recompute g_imu_robo once the frontend has a gravity-aligned attitude.
  ///
  /// `T_O_B` must be an early, stationary frontend pose: the odometry frame is
  /// gravity aligned by kf_init(), so the roll/pitch of that pose IS the
  /// sensor's tilt relative to level. Assumes the robot is standing level and
  /// still at startup -- the same assumption kf_init() already makes.
  /// No-op unless g_base_lidar_auto_rp.
  void CalibrateBaseLidarFromGravity(const BASIC::SE3& T_O_B);

  /// Rebuild g_imu_robo from g_base_lidar and g_lidar_imu. Call after changing
  /// either.
  void RefreshImuRobo();

  extern BASIC::M3  g_lidar_robo_yaw; // yaw-only part, for planner conventions

  /// Robot pose from IMU pose, in whatever frame the input is expressed in.
  inline BASIC::SE3 imuToRobot(const BASIC::SE3& T_x_imu) {
    return T_x_imu * g_imu_robo;
  }

  /// The exact inverse. An operator clicking a robot pose in RViz, or a planner
  /// plugin handing over (x, y, z, yaw), gives a ROBOT pose; the estimator works
  /// in IMU poses, and the two must round-trip exactly or the initial guess is
  /// silently displaced.
  inline BASIC::SE3 robotToImu(const BASIC::SE3& T_x_robot) {
    return T_x_robot * g_imu_robo.inverse();
  }

  /// The base origin's twist, in whatever frame `R_out_imu` maps the IMU into.
  struct BaseTwist {
    BASIC::V3 v;   ///< base origin velocity, in the output frame
    BASIC::V3 w;   ///< angular velocity, in the output frame
  };

  /// The base origin is a different point on the same rigid body as the IMU, so
  /// turning sweeps it around the sensor:
  ///
  ///   v_base = v_imu + R_out_imu * (omega_body x r),   r = g_imu_robo.t_
  ///
  /// Dropping that term is only correct when the sensor sits exactly at the
  /// robot origin -- and since every shipped config has had a ZERO lever arm,
  /// that is exactly the mistake this codebase cannot catch by inspection. It
  /// is how four disagreeing copies of the pose conversion survived for so
  /// long (see imuToRobot above and the design record's extrinsic round).
  ///
  /// Angular velocity is a free vector: the whole rigid body shares it, so only
  /// the frame it is expressed in changes.
  ///
  /// This lives next to imuToRobot() for the same reason it exists: one
  /// concept, one entry point. The frontend needs it in the odom frame
  /// (/lio/robo/odom) and the map localizer needs the identical formula in the
  /// map frame (/loc/robo/odom); parameterizing the output rotation is what
  /// keeps that from becoming two drifting copies.
  ///
  /// @param R_out_imu   IMU -> output frame, e.g. R_odom_imu, or
  ///                    R_map_odom * R_odom_imu for the drift-corrected output
  /// @param v_imu_out   IMU origin velocity, ALREADY rotated into the output
  ///                    frame (the caller owns that rotation, because only it
  ///                    knows whether the drift has been applied yet)
  /// @param w_imu_body  bias-corrected gyro, in the IMU frame
  inline BaseTwist baseTwist(const BASIC::M3& R_out_imu,
                             const BASIC::V3& v_imu_out,
                             const BASIC::V3& w_imu_body) {
    BaseTwist out;
    out.w = R_out_imu * w_imu_body;
    out.v = v_imu_out + R_out_imu * w_imu_body.cross(g_imu_robo.t_);
    return out;
  }

  /// Everything the /loc/* odometry messages need, in the MAP frame.
  struct MapTwist {
    BASIC::V3 v_imu;    ///< IMU origin velocity  -- /loc/odom, /loc/imu/odom
    BASIC::V3 v_base;   ///< base origin velocity -- /loc/robo/odom
    BASIC::V3 w;        ///< angular velocity, shared by both
  };

  /// The frontend reports v in the ODOM frame and w in the IMU body frame, but
  /// the /loc/* messages are stamped frame_id "map". The drift rotation between
  /// the two is NOT small: T_M_O carries the robot's whole heading offset from
  /// wherever it booted, and yaw values in the tens of degrees are routine, so
  /// publishing the odom-frame vector under "map" is wrong by that whole angle.
  ///
  /// Map frame, not the REP-105 child frame, on purpose: the navigation
  /// consumer rotates the twist into the base frame itself using the
  /// orientation carried by this same message, so a map-frame vector is what
  /// makes that rotation correct.
  ///
  /// @param T_M_O   drift actually applied to the pose in this message
  /// @param R_O_I   frontend attitude, R_odom_imu
  /// @param v_I_O   IMU origin velocity in the odom frame (the ESKF's v)
  /// @param w_I     bias-corrected gyro, IMU body frame
  inline MapTwist mapTwist(const BASIC::SE3& T_M_O, const BASIC::M3& R_O_I,
                           const BASIC::V3& v_I_O, const BASIC::V3& w_I) {
    const BASIC::M3 R_M_I = T_M_O.R_ * R_O_I;
    const BASIC::V3 v_imu_map = T_M_O.R_ * v_I_O;
    const BaseTwist base = baseTwist(R_M_I, v_imu_map, w_I);
    return MapTwist{v_imu_map, base.v, base.w};
  }

  /// hash_map
  extern std::size_t g_ivox_capacity;
  extern float       g_ivox_resolution;
  
  /// kf
  extern int g_kf_type;            // 1: ESKF, 2: InESKF.
  extern int g_kf_max_iterations;
  extern bool g_kf_align_gravity;
  extern double g_kf_quit_eps;

  /// submaps
  extern double g_submap_resolution;
  extern int    g_submap_capacity;
  
  /// output  
  extern bool g_2_robot;
  extern bool g_2_plan_env_world;
  extern bool g_2_plan_env_body;
  extern bool g_2_ml_map;
  extern bool g_visual_map;
  extern bool g_visual_dense;
  extern int  g_pub_step;

  /// for planner
  extern bool g_planner_enable;

  /// Define the hybrid residual formulation.
  enum ResidualType{
    PROB = 1,     // Probabilistic residual
    P2P  = 2,     // Point-to-plane residual
    MIX  = 3      // Hybrid residual (probabilistic + point-to-plane)
  };
  extern ResidualType g_residual_type;


  /// for relocation
  extern bool g_update_map;
  extern double g_init_px, g_init_py, g_init_pz, g_init_roll, g_init_pitch, g_init_yaw;
  extern double g_reloc_local_map_radius;

}

#endif
