#include "loc/drift_estimator.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Dense>

#include "basic/logs.h"
#include "loc/se3_math.h"

namespace LI2Sup {
namespace loc {

const char* toString(RejectReason r) {
  switch (r) {
    case RejectReason::kAccepted:      return "accepted";
    case RejectReason::kNotConverged:  return "not_converged";
    case RejectReason::kLowInlier:     return "low_inlier";
    case RejectReason::kHighRmse:      return "high_rmse";
    case RejectReason::kDegenerate:    return "degenerate";
    case RejectReason::kBodyGate:      return "body_gate";
    case RejectReason::kChi2:          return "chi2";
    case RejectReason::kRateGate:      return "rate_gate";
    case RejectReason::kBadCovariance: return "bad_covariance";
  }
  return "unknown";
}

DriftEstimator::DriftEstimator(const EstimatorParams& params) : params_(params) {}

void DriftEstimator::predict(double distance_m, double dt_s, double tilt_change_rad) {
  if (!initialized_) return;
  const double d = std::max(0.0, distance_m);
  const double t = std::max(0.0, dt_s);
  const double a = std::max(0.0, tilt_change_rad);
  if (d <= 0.0 && t <= 0.0 && a <= 0.0) return;

  // Variances add: the two mechanisms (travelling, and simply elapsing) are
  // independent contributions to how far the drift can have wandered.
  const double qt2 = params_.sigma_t_per_m * params_.sigma_t_per_m * d * d +
                     params_.sigma_t_per_s * params_.sigma_t_per_s * t * t;
  // The third term is the attitude-activity allowance: the distance and time
  // terms were calibrated on flat ground and do not cover what the frontend's
  // attitude does on stairs or broken terrain.
  const double qr2 = params_.sigma_r_per_m * params_.sigma_r_per_m * d * d +
                     params_.sigma_r_per_s * params_.sigma_r_per_s * t * t +
                     params_.sigma_r_per_tilt * params_.sigma_r_per_tilt * a * a;
  P_.topLeftCorner<3, 3>()     += Eigen::Matrix3d::Identity() * qr2;
  P_.bottomRightCorner<3, 3>() += Eigen::Matrix3d::Identity() * qt2;
}

double DriftEstimator::ageOfLastAccept(double now) const {
  if (last_accept_stamp_ < 0.0) return std::numeric_limits<double>::infinity();
  return now - last_accept_stamp_;
}

void DriftEstimator::reset(const BASIC::SE3& T_M_O,
                           const Eigen::Matrix<double, 6, 6>& cov,
                           double stamp) {
  T_M_O_ = orthonormalized(T_M_O);
  P_ = cov;
  initialized_ = true;
  consecutive_rejects_ = 0;
  last_accept_stamp_ = stamp;
  // An initialization or relocalization is a direct, confirmed observation of
  // T_M_O, so the tilt it carries is where the prior should already be anchored
  // -- making it slew there from zero would have the prior fighting a known
  // good answer for the first minute after every recovery.
  rp_anchor_ = currentTilt();
  rp_anchor_stamp_ = stamp;
  // A reset is a sanctioned discontinuity (init / relocalization), so the rate
  // gate must not measure against the pre-jump observation.
  have_last_obs_ = false;
  have_pending_ = false;
}

bool DriftEstimator::rateWithinBudget(const Vec6d& step, double travelled) const {
  const double d = std::max(travelled, 0.0);
  return step.tail<3>().norm() <= params_.rate_gate_t * d + params_.rate_gate_floor_t &&
         step.head<3>().norm() <= params_.rate_gate_r * d + params_.rate_gate_floor_r;
}

bool DriftEstimator::pendingConfirms(const BASIC::SE3& Z,
                                     const BASIC::SE3& T_O_Bk) const {
  if (!have_pending_) return false;
  const Vec6d agree = logSE3((pending_Z_ * T_O_Bk).inverse() * (Z * T_O_Bk));
  return rateWithinBudget(agree, (T_O_Bk.t_ - pending_odom_pos_).norm());
}

void DriftEstimator::holdPending(const BASIC::SE3& Z, const BASIC::SE3& T_O_Bk) {
  pending_Z_ = Z;
  pending_odom_pos_ = T_O_Bk.t_;
  have_pending_ = true;
}

UpdateReport DriftEstimator::update(const RegResult& reg,
                                    const BASIC::SE3& T_O_Bk,
                                    double stamp) {
  UpdateReport rep;

  // The body-frame correction is computed first: it is the number the gates and
  // the operator both care about, and it is meaningful even for a rejection.
  const BASIC::SE3 predicted_body = T_M_O_ * T_O_Bk;
  const Eigen::Matrix<double, 6, 1> e = logSE3(predicted_body.inverse() * reg.T);
  rep.body_correction_r = e.head<3>().norm();
  rep.body_correction_t = e.tail<3>().norm();

  auto reject = [&](RejectReason why) {
    rep.accepted = false;
    rep.reason = why;
    ++consecutive_rejects_;
    return rep;
  };

  // Gate 1: did the alignment produce a pose at all, and does it explain enough
  // of the scan?
  //
  // kNotConverged fires only when there is NO pose. An alignment that ran out
  // of iterations still carries a usable estimate -- measured on this map, the
  // registrar returns correct-to-centimetres poses with converged=false often
  // enough that hard-rejecting them ran the consecutive-reject counter up and
  // dropped a healthy system into LOST. Such an observation is trusted less
  // instead: its covariance is inflated below, so the EKF takes a smaller step
  // and the chi-square gate stays meaningful.
  if (!reg.hasPose()) return reject(RejectReason::kNotConverged);
  if (reg.inlier_ratio < params_.track_inlier_min) return reject(RejectReason::kLowInlier);
  if (!(reg.rmse <= params_.track_rmse_max)) return reject(RejectReason::kHighRmse);

  // Gate 2: observability. A corridor constrains nothing along its axis; one
  // unobservable translation direction is survivable because the covariance
  // inflates along it and the EKF gain goes to zero there, two is not.
  if (reg.degenerate_t_dims > params_.max_degenerate_t_dims ||
      reg.degenerate_r_dims > params_.max_degenerate_r_dims) {
    return reject(RejectReason::kDegenerate);
  }

  // Observation: Z = T_M_Bk * T_O_Bk^-1, a direct measurement of T_M_O.
  // Its covariance moves from a right perturbation in Bk to one in O by the
  // adjoint of T_O_Bk: Z_true = Z_est * Exp(Ad(T_O_Bk) * delta_Bk).
  const BASIC::SE3 Z = reg.T * T_O_Bk.inverse();
  const Eigen::Matrix<double, 6, 6> Ad = adjointSE3(T_O_Bk);
  Eigen::Matrix<double, 6, 6> R_z = Ad * reg.cov * Ad.transpose();
  // An alignment that stopped on the iteration limit rather than on its own
  // convergence test is still usable, but its reported precision is not
  // trustworthy: widen it so the EKF leans on it proportionally less.
  if (!reg.converged) R_z *= params_.unconverged_cov_scale;
  R_z = 0.5 * (R_z + R_z.transpose());
  if (!R_z.allFinite()) return reject(RejectReason::kBadCovariance);

  // The remaining gates ask "does this agree with what we already believe?",
  // which is meaningless before there is anything to believe. So the bootstrap
  // comes first: an uninitialized estimator adopts the first observation that
  // clears the quality gates above. In the normal flow MapLocalizer has already
  // called reset() after a confirmed initialization and this never fires; it
  // matters for direct users of the estimator, and it keeps the gate ordering
  // honest rather than leaving an unreachable branch below.
  if (!initialized_) {
    reset(Z, R_z, stamp);
    last_obs_Z_ = Z;
    last_obs_odom_pos_ = T_O_Bk.t_;
    have_last_obs_ = true;
    rep.accepted = true;
    rep.reason = RejectReason::kAccepted;
    return rep;
  }

  // Gate 3: how hard would this yank the robot?
  if (rep.body_correction_t > params_.gate_body_t ||
      rep.body_correction_r > params_.gate_body_r) {
    return reject(RejectReason::kBodyGate);
  }

  // Gate 3b: the drift-rate gate. Independent of P, so it keeps working when
  // the process noise is large enough to make the chi-square test permissive.
  if (params_.rate_gate_enable && have_last_obs_) {
    const double d = (T_O_Bk.t_ - last_obs_odom_pos_).norm();
    // Measured in the BODY frame, evaluated at the SAME odometry pose -- not on
    // the raw T_M_O step. Comparing two drifts directly makes the gate a
    // function of where the odometry origin happens to be: a yaw wobble in the
    // drift turns into metres of apparent translation through the lever arm to
    // that origin, and the lever arm grows without bound as the robot walks.
    //
    // Measured on the board, 117 m out: a 0.23 deg wobble between two
    // consecutive observations became 0.48 m of "drift motion" over a 0.55 m
    // step -- rate 0.87 against a 0.6 limit -- while the correction the robot
    // would actually have felt was 46 mm. Six such refusals in a row declare
    // LOST, which is what happened three times, always far from the start and
    // never near it; the identical wobble 5 m from the origin scores 0.037.
    // Evaluating both drifts at T_O_Bk cancels the lever arm, so the gate now
    // measures what it always claimed to: how far the correction the robot
    // experiences may move, given how far it travelled. `given how far it
    // travelled` is affine, not proportional -- see rate_gate_floor_t in
    // loc_params.h for why, and for what a pure rate did to a parked robot.
    const Vec6d step = logSE3((last_obs_Z_ * T_O_Bk).inverse() * (Z * T_O_Bk));

    if (!rateWithinBudget(step, d)) {
      // Too fast to be real drift. If a held observation says the same thing,
      // the jump is genuine and we take it; otherwise hold this one and wait.
      if (!pendingConfirms(Z, T_O_Bk)) {
        holdPending(Z, T_O_Bk);
        return reject(RejectReason::kRateGate);
      }
    }
  }

  const Eigen::Matrix<double, 6, 1> nu = logSE3(T_M_O_.inverse() * Z);
  rep.innovation_r = nu.head<3>().norm();
  rep.innovation_t = nu.tail<3>().norm();

  Eigen::Matrix<double, 6, 6> S = P_ + R_z;
  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(S);
  if (ldlt.info() != Eigen::Success) return reject(RejectReason::kBadCovariance);

  const Eigen::Matrix<double, 6, 1> S_inv_nu = ldlt.solve(nu);
  rep.chi2 = nu.dot(S_inv_nu);
  if (!std::isfinite(rep.chi2)) return reject(RejectReason::kBadCovariance);

  // Gate 4: consistency with the current estimate and its uncertainty.
  //
  // Hold-and-confirm, exactly as the rate gate does. Without it a real
  // discontinuity that is too small to trip the rate gate but too large for
  // chi-square has no way in at all: it is refused over and over until the
  // process noise has grown P enough to admit it, and the output carries the
  // whole error until then. Measured on the mapping bag at 2.3 m/s, a 365 mm
  // frontend slip scored 0.34 m/m against the 0.6 rate limit -- so the rate
  // gate never saw it -- and took five refusals, 1.5 s, to get in.
  //
  // A second registration agreeing to within the rate-gate tolerance is real
  // evidence: a bad alignment does not repeat itself in the same direction.
  // The body gate above still caps how far any single accepted observation can
  // pull the robot, so confirmation cannot wave through a large yank.
  if (rep.chi2 > params_.chi2_gate) {
    if (!params_.chi2_confirm_enable || !pendingConfirms(Z, T_O_Bk)) {
      holdPending(Z, T_O_Bk);
      return reject(RejectReason::kChi2);
    }
  }

  // Accepted: nothing is being held any more.
  have_pending_ = false;

  // Identity observation model, so K = P S^-1 and the update is a right-composition.
  const Eigen::Matrix<double, 6, 6> K =
      ldlt.solve(P_.transpose()).transpose();  // P * S^-1, solved rather than inverted
  // Renormalize after every state update. The estimator never conjugates, so
  // it degrades far more slowly than the output stage did, but 4700 float
  // products over a run is no reason to let it drift at all.
  T_M_O_ = orthonormalized(T_M_O_ * expSE3(K * nu));
  P_ = (Eigen::Matrix<double, 6, 6>::Identity() - K) * P_;
  P_ = 0.5 * (P_ + P_.transpose());

  if (params_.estimate_roll_pitch) {
    applyRollPitchPrior(stamp);
  } else {
    // 4-DoF mode: hold roll/pitch at exactly zero and keep the EKF from
    // spending any uncertainty on them.
    const Eigen::Matrix<double, 6, 1> w = logSE3(T_M_O_);
    Eigen::Matrix<double, 6, 1> flat = Eigen::Matrix<double, 6, 1>::Zero();
    flat(2) = w(2);
    flat.tail<3>() = w.tail<3>();
    T_M_O_ = expSE3(flat);
    P_.row(0).setZero(); P_.col(0).setZero();
    P_.row(1).setZero(); P_.col(1).setZero();
    P_(0, 0) = P_(1, 1) = 1e-12;
  }

  consecutive_rejects_ = 0;
  last_accept_stamp_ = stamp;
  last_obs_Z_ = Z;
  last_obs_odom_pos_ = T_O_Bk.t_;
  have_last_obs_ = true;
  rep.accepted = true;
  rep.reason = RejectReason::kAccepted;
  return rep;
}

UpdateReport DriftEstimator::addPositionMeasurement(const Eigen::Vector3d& p_M,
                                                    const Eigen::Matrix3d& cov,
                                                    const BASIC::SE3& T_O_Bk,
                                                    double stamp) {
  UpdateReport rep;
  if (!initialized_) {
    // A position alone cannot fix yaw, so it cannot bootstrap the state.
    rep.reason = RejectReason::kBadCovariance;
    return rep;
  }

  const Eigen::Vector3d p_O = T_O_Bk.t_.template cast<double>();
  const Eigen::Matrix3d R = T_M_O_.R_.template cast<double>();
  const Eigen::Vector3d predicted = R * p_O + T_M_O_.t_.template cast<double>();
  const Eigen::Vector3d residual = p_M - predicted;

  // Right perturbation, [omega; rho]: d(R p + t) = -R [p]^ domega + R drho.
  Eigen::Matrix<double, 3, 6> H;
  H.leftCols<3>()  = -R * skew3(p_O);
  H.rightCols<3>() = R;

  const Eigen::Matrix3d S = H * P_ * H.transpose() + cov;
  Eigen::LDLT<Eigen::Matrix3d> ldlt(S);
  if (ldlt.info() != Eigen::Success) {
    rep.reason = RejectReason::kBadCovariance;
    ++consecutive_rejects_;
    return rep;
  }

  rep.chi2 = residual.dot(ldlt.solve(residual));
  rep.innovation_t = residual.norm();
  // 3 dof at 99% rather than the 6-dof gate used for pose observations.
  if (rep.chi2 > 11.35) {
    rep.reason = RejectReason::kChi2;
    ++consecutive_rejects_;
    return rep;
  }

  const Eigen::Matrix<double, 6, 3> K =
      P_ * H.transpose() * ldlt.solve(Eigen::Matrix3d::Identity());
  T_M_O_ = orthonormalized(T_M_O_ * expSE3(K * residual));
  P_ = (Mat6d::Identity() - K * H) * P_;
  P_ = 0.5 * (P_ + P_.transpose());
  if (params_.estimate_roll_pitch) applyRollPitchPrior(stamp);

  consecutive_rejects_ = 0;
  last_accept_stamp_ = stamp;
  rep.accepted = true;
  rep.reason = RejectReason::kAccepted;
  return rep;
}

Eigen::Vector2d DriftEstimator::currentTilt() const {
  // Tilt of T_M_O as a world-frame rotation vector. The frontend's gravity
  // alignment drifts a few degrees over a run, so this is not the sub-milli-
  // radian quantity the zeroing prior assumed, but it is still small enough
  // that the x/y components of the log are roll and pitch.
  const Eigen::Matrix3d R = T_M_O_.R_.template cast<double>();
  const Eigen::AngleAxisd aa(Eigen::Quaterniond(R).normalized());
  const Eigen::Vector3d w = aa.axis() * aa.angle();
  return Eigen::Vector2d(w.x(), w.y());
}

void DriftEstimator::applyRollPitchPrior(double stamp) {
  const Eigen::Vector2d tilt = currentTilt();

  // Let the anchor follow, at a bounded rate. It chases the estimate as the
  // measurement has just left it -- the EKF update runs before this -- so a
  // tilt the registrations keep asking for is adopted within tens of seconds,
  // while one that appears and disappears inside a few matches never moves the
  // anchor far enough to be taken.
  //
  // dt is clamped because a backend that was starved (submap reload, a long
  // LOST) would otherwise cash in the whole outage as one slew step, which is
  // exactly the jump the rate limit exists to refuse.
  constexpr double kMaxSlewDt = 5.0;
  if (rp_anchor_stamp_ >= 0.0) {
    const double dt = std::clamp(stamp - rp_anchor_stamp_, 0.0, kMaxSlewDt);
    const double max_step = params_.rp_anchor_rate * dt;
    const Eigen::Vector2d err = tilt - rp_anchor_;
    const double n = err.norm();
    if (n > max_step) {
      if (max_step > 0.0) rp_anchor_ += err * (max_step / n);
    } else {
      rp_anchor_ = tilt;
    }
  }
  rp_anchor_stamp_ = stamp;

  const Eigen::Vector2d residual = rp_anchor_ - tilt;

  // A right perturbation of the body maps to a world rotation change by R:
  // delta_w_world ~= R * delta_omega. Take the first two rows.
  const Eigen::Matrix3d R = T_M_O_.R_.template cast<double>();
  Eigen::Matrix<double, 2, 6> H = Eigen::Matrix<double, 2, 6>::Zero();
  H.block<2, 3>(0, 0) = R.topRows<2>();

  const double var = params_.rp_pseudo_sigma * params_.rp_pseudo_sigma;
  Eigen::Matrix2d S = H * P_ * H.transpose() + Eigen::Matrix2d::Identity() * var;

  Eigen::LDLT<Eigen::Matrix2d> ldlt(S);
  if (ldlt.info() != Eigen::Success) return;

  const Eigen::Matrix<double, 6, 2> K = P_ * H.transpose() * ldlt.solve(Eigen::Matrix2d::Identity());
  T_M_O_ = orthonormalized(T_M_O_ * expSE3(K * residual));
  P_ = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * P_;
  P_ = 0.5 * (P_ + P_.transpose());
}

}  // namespace loc
}  // namespace LI2Sup
