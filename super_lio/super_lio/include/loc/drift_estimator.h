/**
 * @file drift_estimator.h
 * @brief The whole backend state: one SE(3) drift transform T_M_O and its 6x6
 *        covariance, plus the gate chain that decides what is allowed to touch it.
 *
 * Why the state is the drift and not the pose: T_M_O changes slowly (it is the
 * accumulated error of the frontend), so a registration that takes 80 ms still
 * produces a correction that is valid "now". The observation is built from the
 * odometry pose snapshotted with the scan, so alignment latency never turns
 * into pose latency. Nothing here needs to catch up with the frontend.
 *
 * Conventions: right perturbation, T <- T * Exp(delta), delta = [omega; rho]
 * (rotation first), matching BASIC::hat6/vee6 and small_gicp. P and every
 * covariance in this file use that same ordering.
 *
 * The interface is deliberately narrow: predict, update, reset, and read. The
 * gate chain, the chi-square test, the adjoint transport of the measurement
 * covariance and the roll/pitch prior are all implementation.
 *
 * Testability: no ROS, no clock of its own (times are passed in), no point
 * clouds. Synthetic observations are enough to exercise every branch.
 */
#ifndef LOC_DRIFT_ESTIMATOR_H_
#define LOC_DRIFT_ESTIMATOR_H_

#include <string>

#include <Eigen/Core>

#include "basic/alias.h"
#include "basic/Manifold.h"
#include "loc/loc_params.h"
#include "loc/registrar.h"
#include "loc/se3_math.h"

namespace LI2Sup {
namespace loc {

/// Why an observation was refused. Ordered as the gates are applied.
enum class RejectReason {
  kAccepted = 0,
  kNotConverged,
  kLowInlier,
  kHighRmse,
  kDegenerate,
  kBodyGate,     ///< the correction would yank the robot too far
  kChi2,         ///< inconsistent with the current estimate and its covariance
  kRateGate,     ///< drift moved faster than physically plausible; held for confirmation
  kBadCovariance
};

const char* toString(RejectReason r);

/// What update() decided and why. Returned rather than logged, so tests and the
/// status topic see exactly what the estimator saw.
struct UpdateReport {
  bool accepted = false;
  RejectReason reason = RejectReason::kAccepted;

  /// Correction as the robot experiences it: e = Log((T_hat*T_O_Bk)^-1 * T_M_Bk).
  /// This, not the T_M_O delta, is the quantity the gates use and the number
  /// worth showing a human — a yaw change in T_M_O turns into metres of body
  /// motion through the lever arm to the odometry origin.
  double body_correction_t = 0.0;   ///< [m]
  double body_correction_r = 0.0;   ///< [rad]

  double chi2 = 0.0;
  double innovation_t = 0.0;        ///< [m], norm of the T_M_O innovation
  double innovation_r = 0.0;        ///< [rad]
};

class DriftEstimator {
public:
  explicit DriftEstimator(const EstimatorParams& params);

  /// Inflate the covariance for the odometry travelled and the time elapsed
  /// since the last call. T_M_O itself is treated as constant.
  /// Both terms matter: distance dominates while walking, but a stationary
  /// robot must still lose confidence or the chi-square gate hardens into a
  /// hair trigger against the next real correction.
  /// @param tilt_change_rad  how far the FRONTEND's attitude tilted since the
  ///        last match (angle between the body z axis then and now, so it is
  ///        yaw-invariant). Scaled by sigma_r_per_tilt; see loc_params.h.
  void predict(double distance_m, double dt_s = 0.0, double tilt_change_rad = 0.0);

  /// Fold one registration result in, after the gate chain.
  /// @param reg      alignment of the scan, T_M_Bk plus its covariance in Bk
  /// @param T_O_Bk   odometry pose snapshotted with that scan
  /// @param stamp    scan time, used only for the "time since last accept" clock
  UpdateReport update(const RegResult& reg, const BASIC::SE3& T_O_Bk, double stamp);

  /// Jump straight to a known drift (initialization, or an accepted relocalization).
  void reset(const BASIC::SE3& T_M_O, const Eigen::Matrix<double, 6, 6>& cov, double stamp);

  /// Fold in a position-only observation of the body in the map frame -- the
  /// GPS seam of design doc 13.1/13.2. h(T_M_O) = T_M_O * p_O_B, a 3-row update
  /// against the same state and the same chi-square gate, so a fix constrains
  /// the drift translation and (through the lever arm) its yaw. That is exactly
  /// the direction LiDAR loses in open ground, which is why the two complement
  /// each other.
  /// @param p_M      measured body position in the map frame
  /// @param cov      3x3 covariance of that measurement
  /// @param T_O_Bk   odometry pose snapshotted with the measurement
  UpdateReport addPositionMeasurement(const Eigen::Vector3d& p_M,
                                      const Eigen::Matrix3d& cov,
                                      const BASIC::SE3& T_O_Bk, double stamp);

  bool initialized() const { return initialized_; }
  const BASIC::SE3& drift() const { return T_M_O_; }
  const Eigen::Matrix<double, 6, 6>& covariance() const { return P_; }

  int    consecutiveRejects() const { return consecutive_rejects_; }
  double lastAcceptStamp() const { return last_accept_stamp_; }
  /// Seconds since the last accepted observation; infinite before the first one.
  double ageOfLastAccept(double now) const;

  /// Body pose implied by the current drift.
  BASIC::SE3 mapPose(const BASIC::SE3& T_O_B) const { return T_M_O_ * T_O_B; }

private:
  /// Weak prior on the roll/pitch of T_M_O, anchored rather than zeroed: it
  /// bounds how fast the tilt may change, not how large it may be. The anchor
  /// follows the estimate at params_.rp_anchor_rate, so a slow real tilt is
  /// eventually adopted while a fast one is held back. See the rationale on
  /// EstimatorParams::rp_pseudo_sigma for why zeroing it was wrong.
  void applyRollPitchPrior(double stamp);

  /// World-frame x/y of Log(R_M_O) — the tilt, as the prior measures it.
  Eigen::Vector2d currentTilt() const;

  /// May two observations taken `travelled` metres apart disagree by `step`?
  ///
  /// The one place the rate gate's budget is defined; both the gate and the
  /// confirmation check go through here so they cannot drift apart. `step` is
  /// a body-frame Vec6d (rotation first, translation last), and the budget is
  /// affine in `travelled` -- see rate_gate_floor_t in loc_params.h.
  bool rateWithinBudget(const Vec6d& step, double travelled) const;

  /// Does the held observation say the same thing as this one?
  ///
  /// Two independent registrations agreeing on a jump is what separates a real
  /// discontinuity -- the frontend slipped -- from one bad alignment. Compared
  /// in the BODY frame at the SAME odometry pose, for the same reason the rate
  /// gate is: comparing two drifts directly makes the answer a function of how
  /// far the robot has walked from the odometry origin.
  bool pendingConfirms(const BASIC::SE3& Z, const BASIC::SE3& T_O_Bk) const;

  /// Hold this observation back for a second opinion on the next match.
  void holdPending(const BASIC::SE3& Z, const BASIC::SE3& T_O_Bk);

  EstimatorParams params_;
  BASIC::SE3 T_M_O_;
  Eigen::Matrix<double, 6, 6> P_ = Eigen::Matrix<double, 6, 6>::Identity();

  bool   initialized_ = false;
  int    consecutive_rejects_ = 0;
  double last_accept_stamp_ = -1.0;

  /// Previous ACCEPTED observation and the odometry position it was taken at,
  /// which is what the drift-rate gate measures against.
  bool       have_last_obs_ = false;
  BASIC::SE3 last_obs_Z_;
  BASIC::V3  last_obs_odom_pos_ = BASIC::V3::Zero();

  /// An observation held back by the rate gate OR the chi-square gate, waiting
  /// for a second opinion. One slot serves both: either gate refusing means the
  /// same thing operationally -- this looks wrong, show me it again.
  bool       have_pending_ = false;
  BASIC::SE3 pending_Z_;
  BASIC::V3  pending_odom_pos_ = BASIC::V3::Zero();

  /// Tilt the roll/pitch prior pulls toward, and when it last moved. Seeded
  /// from the pose handed to reset() -- an initialization or relocalization is
  /// a direct observation of T_M_O, so the tilt it found is the right place to
  /// start rather than something to be walked toward at the rate limit.
  Eigen::Vector2d rp_anchor_ = Eigen::Vector2d::Zero();
  double          rp_anchor_stamp_ = -1.0;
};

}  // namespace loc
}  // namespace LI2Sup

#endif  // LOC_DRIFT_ESTIMATOR_H_
