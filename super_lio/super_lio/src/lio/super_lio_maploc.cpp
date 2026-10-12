#include "lio/super_lio_maploc.h"

#include "loc/se3_math.h"

#include <cstdlib>
#include <fstream>
#include <iomanip>

#include "basic/logs.h"

namespace LI2Sup {

SuperLIOMapLoc::~SuperLIOMapLoc() {
  if (localizer_) localizer_->stop();
}

void SuperLIOMapLoc::init() {
  SuperLIO::init();

  if (params_.map.map_dir.empty()) {
    // Name both ways it can be empty. The second one bit us on the board: a
    // launch argument left unset used to write an empty string OVER the yaml's
    // path, because ROS 2 applies params files in order and the last one wins.
    LOG(FATAL) << RED
               << " ---> [MapLoc] lio.maploc.map_dir is empty; nothing to localize against."
                  "\n      Set it in config/map_localization.yaml, or pass"
                  " map_dir:=/path/to/loc_map on the launch command line."
                  "\n      If the yaml does have a path, check that nothing is overriding"
                  " it with an empty value: a later --params-file wins over an earlier one."
               << RESET;
    return;
  }

  map_ = std::make_shared<loc::KeyframeMap>();
  if (!map_->load(params_.map.map_dir, params_.map.preload_all, params_.map.lru_capacity)) {
    LOG(FATAL) << " ---> [MapLoc] failed to load " << params_.map.map_dir;
    return;
  }

  // The map clouds already have the mapping-time extrinsic baked in and the
  // frontend's scans use lio.extrinsic.lidar_imu; if the two differ, every
  // alignment carries a constant offset that no amount of filtering removes.
  if (map_->meta().has_extrinsic) {
    const BASIC::V3 dt = map_->meta().T_B_L.t_ - g_lidar_imu.t_;
    const BASIC::M3 dR = map_->meta().T_B_L.R_.transpose() * g_lidar_imu.R_;
    const double dr = Eigen::AngleAxis<BASIC::scalar>(dR).angle();
    if (dt.norm() > 1e-3 || dr > 1e-3) {
      LOG(WARNING) << RED << " ---> [MapLoc] extrinsic mismatch: map was built with t=("
                   << map_->meta().T_B_L.t_.transpose() << "), this node uses t=("
                   << g_lidar_imu.t_.transpose() << "); expect a constant bias of "
                   << dt.norm() << " m" << RESET;
    }
  }

  publisher_ = std::make_unique<LocPublisher>(data_wrapper_.get(), params_);
  localizer_ = std::make_unique<loc::MapLocalizer>(map_, params_);

  publisher_->setInitCallback(
      [this](const loc::InitRequest& req) { localizer_->requestInit(req); });

  // Display assets, published once the map is up.
  publisher_->setGlobalMap(map_->globalDisplayCloud());
  publisher_->publishKeyframes(map_->keyframes());

  if (params_.output.publish_imu_rate) {
    data_wrapper_->setImuRateHook(
        [this](const DynamicState& imu_state, const builtin_interfaces::msg::Time& stamp) {
          if (!have_applied_) return;
          // The same estimate the 10 Hz path publishes, so /loc/imu/odom and
          // /loc/odom agree by construction rather than by tuning.
          const BASIC::SE3 T_O_B(BASIC::M3(imu_state.R), imu_state.p);
          publisher_->publishImuRatePose(applied_drift_ * T_O_B, applied_drift_,
                                         imu_state, stamp);
        });
  }

  localizer_->start();

  LOG(INFO) << GREEN << " ---> [MapLoc] init mode -- "
            << publisher_->initModeDescription() << RESET;
  if (publisher_->initFromParams()) {
    localizer_->requestInit(publisher_->paramsInitRequest());
  }
}

void SuperLIOMapLoc::finish() {
  if (localizer_) localizer_->stop();
  if (publisher_) publisher_->saveTrajectory();
}

void SuperLIOMapLoc::Output() {
  SuperLIO::Output();
  if (!localizer_) return;

  // DynamicState, not NavState: the outputs need the angular rate too, and
  // both accessors read the same current_time_, so stamps are unchanged.
  const auto state = kf_->GetDynamicState();
  const BASIC::SE3 T_O_B = kf_->GetSE3();

  // Calibrate the sensor tilt from gravity, once, on the first frame after the
  // frontend has finished its static init. The odometry frame is gravity
  // aligned by then, so this pose's roll/pitch is the mounting tilt.
  //
  // Deliberately NOT taken from the map: the map records whichever robot built
  // it, and a second robot with a different mounting would be told its own
  // correct extrinsic is wrong. Gravity is local to this robot.
  if (!mounting_calibrated_) {
    mounting_calibrated_ = true;
    CalibrateBaseLidarFromGravity(T_O_B);
    reportMounting(T_O_B);
  }

  // 1. Hand the backend this frame. The cloud MUST be copied: the frontend
  //    resizes scan_undistort_full_ in place on the next iteration.
  if (scan_undistort_full_ && !scan_undistort_full_->empty()) {
    loc::FrontendFrame frame;
    frame.t = state.time;
    frame.T_O_B = T_O_B;
    frame.gravity_O = kf_->GetGravity();
    const auto imu_timing = kf_->GetImuTiming();
    frame.imu.refused   = imu_timing.refused();
    frame.imu.max_gap_s = imu_timing.max_gap_s;
    frame.imu.period_s  = imu_timing.period();
    frame.cloud_B = loc::CloudXYZ::Ptr(new loc::CloudXYZ());
    frame.cloud_B->reserve(scan_undistort_full_->size());
    for (const auto& p : *scan_undistort_full_) {
      // Intensity is dropped on purpose: voxel-slam wrote zeros into every map
      // keyframe, so it carries no information either side of the match.
      frame.cloud_B->emplace_back(p.x, p.y, p.z);
    }
    localizer_->pushFrame(std::move(frame));
  }

  // 2. Publish. /loc/odom carries the ESTIMATE directly.
  //
  //    Round 2 measured the old arrangement, where a rate limiter sat between
  //    the estimate and the output: ATE RMSE 0.601 vs 0.190 m, rotation
  //    1.637 vs 0.159 deg, and the limiter itself produced the largest
  //    per-frame output jump in the run (4.86 m) because its own state was
  //    diverging. It was worse at both of the things it existed to trade
  //    between, so it is no longer in the path. Smoothing is now an opt-in
  //    side channel (/loc/odom_smooth) that has to earn its place on numbers.
  BASIC::SE3 T_M_O_est;
  bool jumped = false;
  const auto backend_status = localizer_->status();
  const bool backend_jumped = backend_status.jump_seq != last_jump_seq_;
  if (backend_jumped) {
    last_jump_seq_ = backend_status.jump_seq;
    composer_.requestSnap();
  }
  static std::ofstream* clog = [] {
    const char* p = std::getenv("MAPLOC_COMPOSER_LOG");
    if (!p) return static_cast<std::ofstream*>(nullptr);
    auto* f = new std::ofstream(p);
    // Default ostream precision is 6 SIGNIFICANT digits, which turns a UNIX
    // timestamp into 1.78874e+09 and makes the column useless for aligning
    // against anything.
    *f << std::fixed << std::setprecision(9);
    *f << "t,det_odom,orth_odom,det_est,orth_est,lever\n";
    return f;
  }();

  if (localizer_->drift(&T_M_O_est)) {
    const BASIC::SE3 T_M_B = T_M_O_est * T_O_B;
    applied_drift_ = T_M_O_est;
    have_applied_ = true;
    // A confirmed (re)initialization is still a discontinuity worth flagging,
    // even though nothing is being ramped.
    jumped = backend_jumped;

    publisher_->publishPose(T_M_B, T_M_O_est, state);
    publisher_->publishRawPose(T_M_B, T_M_O_est, state);  // T10: == /loc/odom

    // T9a: how far the frontend's rotation and the estimate have drifted from
    // orthogonality. T_O_B here is the ESKF's raw float matrix, before the
    // backend scrubs it.
    if (clog) {
      double d_o, f_o, d_e, f_e;
      loc::orthogonalityOf(T_O_B.R_, &d_o, &f_o);
      loc::orthogonalityOf(T_M_O_est.R_, &d_e, &f_e);
      *clog << state.time << ',' << d_o << ',' << f_o << ','
            << d_e << ',' << f_e << ',' << T_O_B.t_.norm() << std::endl;
    }

    if (params_.output.smooth_output) {
      const auto sm = composer_.compose(T_M_O_est, T_O_B, state.time);
      publisher_->publishSmoothPose(sm.T_M_B, state);
    }
  } else if (params_.output.publish_when_invalid &&
             backend_status.state != loc::LocState::kFailed) {
    // publish_when_invalid exists so a run can be watched before the first fix,
    // where T_O_B is at least a consistent odometry pose. In FAILED it is not:
    // publishing it would put the flying pose on /loc/odom under the map frame,
    // which is the one thing this state exists to prevent.
    publisher_->publishPose(T_O_B, BASIC::SE3(), state);
  }

  // Stop the IMU-rate path too. It republishes `applied_drift_` between 10 Hz
  // frames and has its own latch, so leaving it alone would keep /loc/imu/odom
  // streaming the last good drift times a diverging T_O_B at IMU rate --
  // faster, and just as wrong, as the path this state already closed.
  if (backend_status.state == loc::LocState::kFailed) have_applied_ = false;

  // 3. Submap display, only when the backend actually rebuilt one.
  loc::CloudXYZ::Ptr submap;
  std::uint64_t version = 0;
  if (localizer_->takeSubmapForDisplay(&submap, &version)) {
    publisher_->publishSubmap(submap, state.time);
  }

  publishStatusIfDue(state.time, jumped);
}

void SuperLIOMapLoc::reportMounting(const BASIC::SE3& T_O_B) const {
  // Whether or not the auto path ran, say what the tilt measures against what
  // the config claims. A silent mismatch here does not degrade accuracy -- it
  // makes initialization fail outright, with nothing in the logs pointing here.
  const double kDeg = 180.0 / M_PI;
  const Eigen::Matrix3d R = T_O_B.R_.cast<double>();
  const double meas_roll  = std::atan2(R(2, 1), R(2, 2)) * kDeg;
  const double meas_pitch = std::asin(std::max(-1.0, std::min(1.0, -R(2, 0)))) * kDeg;

  const Eigen::Matrix3d C = g_base_lidar.R_.cast<double>();
  const double cfg_roll  = std::atan2(C(2, 1), C(2, 2)) * kDeg;
  const double cfg_pitch = std::asin(std::max(-1.0, std::min(1.0, -C(2, 0)))) * kDeg;

  LOG(INFO) << " ---> [MapLoc] mounting: gravity measures the lidar at roll "
            << meas_roll << " deg, pitch " << meas_pitch
            << " deg; base_lidar says roll " << cfg_roll << " deg, pitch "
            << cfg_pitch << " deg";

  if (g_base_lidar_auto_rp) return;   // the config was overridden anyway
  const double d_roll  = std::abs(meas_roll - cfg_roll);
  const double d_pitch = std::abs(meas_pitch - cfg_pitch);
  if (d_roll > 10.0 || d_pitch > 10.0) {
    LOG(WARNING) << RED
        << " ---> [MapLoc] lio.extrinsic.base_lidar disagrees with gravity by "
        << d_roll << " deg roll / " << d_pitch << " deg pitch. Every robot-frame"
           " output and every clicked initial pose is rotated by that much, and"
           " initialization will fail if it exceeds init_yaw_range. Either fix"
           " the value (roll " << meas_roll << ", pitch " << meas_pitch
        << ") or set lio.extrinsic.base_lidar_auto_rp: true." << RESET;
  }
}

void SuperLIOMapLoc::publishStatusIfDue(double stamp, bool jumped) {
  const auto status = localizer_->status();
  // Publish on every state change or match, and otherwise at 1 Hz so a silent
  // backend still shows up on the topic.
  const bool due = last_status_pub_ < 0.0 || (stamp - last_status_pub_) >= 1.0 ||
                   status.stamp > last_status_pub_ || jumped != last_jump_;
  if (!due) return;
  publisher_->publishStatus(status, jumped);
  last_status_pub_ = stamp;
  last_jump_ = jumped;
}

}  // namespace LI2Sup
