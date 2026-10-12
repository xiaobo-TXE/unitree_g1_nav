/**
 * @file se3_math.h
 * @brief Double-precision SE(3) log/exp/adjoint for the localization backend.
 *
 * BASIC::scalar is float, and BASIC::SE3 stores its members at that precision.
 * That is fine for the frontend, but this module estimates a drift correction
 * whose interesting magnitude is millimetres over a 200 m map: a float log of a
 * near-identity transform throws away exactly the quantity being estimated, and
 * the EKF covariance arithmetic is worse still. So every module below does its
 * algebra in double and converts back only at the interface.
 *
 * Ordering is [omega(3); rho(3)] — rotation first — matching BASIC::hat6/vee6,
 * BASIC::SE3::adjoint() and small_gicp's se3_exp. Keeping one ordering
 * everywhere is what lets a small_gicp Hessian be used as an EKF covariance
 * with no reshuffling.
 */
#ifndef LOC_SE3_MATH_H_
#define LOC_SE3_MATH_H_

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "basic/alias.h"
#include "basic/Manifold.h"

namespace LI2Sup {
namespace loc {

using Vec6d = Eigen::Matrix<double, 6, 1>;
using Mat6d = Eigen::Matrix<double, 6, 6>;

inline Eigen::Matrix3d skew3(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m <<     0, -v.z(),  v.y(),
       v.z(),      0, -v.x(),
      -v.y(),  v.x(),      0;
  return m;
}

/// Rotation vector of an SE(3)/SO(3) rotation, in double.
inline Eigen::Vector3d logSO3(const Eigen::Matrix3d& R) {
  const Eigen::AngleAxisd aa(Eigen::Quaterniond(R).normalized());
  return aa.axis() * aa.angle();
}

inline Vec6d logSE3(const BASIC::SE3& T) {
  const Eigen::Matrix3d R = T.R_.template cast<double>();
  const Eigen::Vector3d t = T.t_.template cast<double>();

  const Eigen::Vector3d omega = logSO3(R);
  const double theta = omega.norm();

  Eigen::Matrix3d Vinv = Eigen::Matrix3d::Identity();
  if (theta > 1e-10) {
    const Eigen::Matrix3d W = skew3(omega / theta);
    const double half = 0.5 * theta;
    // V^-1 = I - (theta/2) W + (1 - (theta/2) cot(theta/2)) W^2
    Vinv = Eigen::Matrix3d::Identity() - half * W +
           (1.0 - half * std::cos(half) / std::sin(half)) * W * W;
  }

  Vec6d xi;
  xi.head<3>() = omega;
  xi.tail<3>() = Vinv * t;
  return xi;
}

inline BASIC::SE3 expSE3(const Vec6d& xi) {
  const Eigen::Vector3d omega = xi.head<3>();
  const Eigen::Vector3d rho   = xi.tail<3>();
  const double theta = omega.norm();
  const Eigen::Matrix3d W = skew3(omega);

  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d V = Eigen::Matrix3d::Identity();
  if (theta > 1e-10) {
    const double t2 = theta * theta;
    R += (std::sin(theta) / theta) * W + ((1.0 - std::cos(theta)) / t2) * W * W;
    V += ((1.0 - std::cos(theta)) / t2) * W + ((theta - std::sin(theta)) / (t2 * theta)) * W * W;
  } else {
    R += W;
    V += 0.5 * W;
  }

  return BASIC::SE3(BASIC::M3(R.template cast<BASIC::scalar>()),
                    BASIC::V3((V * rho).template cast<BASIC::scalar>()));
}

/// [[R, 0], [t^ R, R]] under the [omega; rho] ordering.
inline Mat6d adjointSE3(const BASIC::SE3& T) {
  const Eigen::Matrix3d R = T.R_.template cast<double>();
  const Eigen::Vector3d t = T.t_.template cast<double>();

  Mat6d Ad = Mat6d::Zero();
  Ad.topLeftCorner<3, 3>()     = R;
  Ad.bottomRightCorner<3, 3>() = R;
  Ad.bottomLeftCorner<3, 3>()  = skew3(t) * R;
  return Ad;
}

/// Re-orthonormalize a pose's rotation block via a quaternion round trip.
///
/// Nothing in basic/ or the ESKF ever does this: SO3::operator* and
/// SO3::update are plain float 3x3 products, and ESKF::R_ is multiplied once
/// per IMU sample (~470k times over a 39 minute run), so R'R - I only ever
/// grows. Published quaternions hide it because quaternion() normalizes on the
/// way out, but the matrix handed to this module does not.
///
/// It matters here because SE3::inverse() uses the transpose as the inverse.
/// For R = k*Q with Q orthogonal, R * R^T = k^2 * I, so the composer's
/// conjugation T_out * T_O_B * Exp(d) * T_O_B^-1 multiplies its own rotation
/// block by k^2 EVERY frame -- measured: k = 1 + 1e-5 drives det(R_out) to 1.41
/// within 6k frames, which is what made the output stage diverge in section
/// 20.5 while the estimator (no conjugation) stayed clean.
///
/// One quaternion round trip per frame, well under a microsecond.
inline BASIC::SE3 orthonormalized(const BASIC::SE3& T) {
  const Eigen::Quaterniond q(T.R_.template cast<double>());
  const Eigen::Matrix3d R = q.normalized().toRotationMatrix();
  return BASIC::SE3(BASIC::M3(R.template cast<BASIC::scalar>()), T.t_);
}

/// How far a rotation has drifted from orthogonality. For diagnostics (T9a).
inline void orthogonalityOf(const BASIC::M3& R_in, double* det, double* frob) {
  const Eigen::Matrix3d R = R_in.template cast<double>();
  if (det) *det = R.determinant();
  if (frob) *frob = (R.transpose() * R - Eigen::Matrix3d::Identity()).norm();
}

/// Yaw of a rotation, matching BASIC::SO3::yaw() but in double.
inline double yawOf(const Eigen::Matrix3d& R) {
  const double sy = std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
  if (sy < 1e-9) return 0.0;
  return std::atan2(R(1, 0), R(0, 0));
}

}  // namespace loc
}  // namespace LI2Sup

#endif  // LOC_SE3_MATH_H_
