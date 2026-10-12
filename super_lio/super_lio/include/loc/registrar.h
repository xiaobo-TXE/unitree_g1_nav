/**
 * @file registrar.h
 * @brief The registration seam: scan-to-submap alignment behind one interface.
 *
 * Two adapters live behind it — SmallGicpRegistrar (the working default) and
 * PclGicpRegistrar (reference/fallback for A-B comparison) — so this is a real
 * seam, not a hypothetical one.
 *
 * The interface hides more than "call align": the registrar owns the target
 * representation. Callers hand it the stitched submap cloud plus a version and
 * it decides what to build (Gaussian voxel map, kd-tree, per-point covariances)
 * and when to rebuild. That keeps small_gicp types out of every other module's
 * interface and means switching backends touches only this file.
 *
 * It also returns decision-ready quality: a covariance already scaled and
 * floored, a metric RMSE, and degeneracy counts. DriftEstimator never has to
 * know what a Hessian is.
 *
 * Conventions (verified against small_gicp, and matching BASIC::SE3):
 *  - RegResult::T is T_M_Bk, the body pose of the source scan in the map frame.
 *  - H and cov are expressed under the RIGHT perturbation T <- T * Exp(delta),
 *    with delta ordered [omega(3); rho(3)] — rotation first. This is exactly
 *    small_gicp's se3_exp ordering and exactly BASIC::hat6/vee6/updateRhs/
 *    adjoint(), so no reordering is needed anywhere in this module.
 *
 * Thread affinity: backend thread (T1). align() runs inside the registrar's own
 * tbb::task_arena so it cannot starve the frontend's parallel_for.
 */
#ifndef LOC_REGISTRAR_H_
#define LOC_REGISTRAR_H_

#include <memory>
#include <cstdint>

#include <Eigen/Core>

#include "basic/alias.h"
#include "basic/Manifold.h"
#include "loc/keyframe_map.h"
#include "loc/loc_params.h"

namespace LI2Sup {
namespace loc {

struct RegResult {
  bool   converged = false;
  int    iterations = 0;
  BASIC::SE3 T;                 ///< T_M_Bk, the aligned source pose

  /// Information and covariance under the right perturbation in Bk, [omega; rho].
  Eigen::Matrix<double, 6, 6> H   = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Identity();

  double inlier_ratio = 0.0;    ///< source points with a correspondence
  double rmse = std::numeric_limits<double>::infinity();  ///< metric, inliers only [m]
  int    num_inliers = 0;
  int    num_source  = 0;
  int    num_target  = 0;   ///< points in the installed target, for the match log

  /// Observability, from the eigenvalues of the H blocks.
  double eig_min_t = 0.0, eig_max_t = 0.0;
  double eig_min_r = 0.0, eig_max_r = 0.0;
  int    degenerate_t_dims = 0, degenerate_r_dims = 0;

  /// False for backends that cannot produce a real information matrix; the
  /// estimator then treats `cov` as a conservative stand-in rather than a
  /// measurement of this particular alignment.
  bool   information_is_real = true;

  double time_ms = 0.0;

  /// A usable pose came back: correspondences were found and `T` is the
  /// optimizer's best estimate.
  ///
  /// This is deliberately NOT the same question as `converged`, and there is no
  /// combined ok() helper, because conflating the two is a bug we already paid
  /// for. small_gicp sets `converged` only when the final step falls under
  /// ~1 mm / 0.1 deg within the iteration budget (an exhausted LM inner loop
  /// also exits unconverged). Measured on this map: the coarse pass at
  /// max_iter=10 returned four poses that were correct to <0.1 m / <1 deg while
  /// reporting converged=false, and dropping those made initialization fail at
  /// random. Callers must decide explicitly: use hasPose() to ask "is there a
  /// pose to consider", `converged` to ask "how much should I trust its
  /// precision".
  bool hasPose() const { return num_inliers > 0; }
};

class IRegistrar {
public:
  virtual ~IRegistrar() = default;

  /// Install a new target. A repeat call with the same `version` is a no-op, so
  /// callers may pass the current submap on every match without thinking.
  virtual void setTarget(const CloudXYZ::ConstPtr& cloud_M, std::uint64_t version) = 0;

  virtual std::uint64_t targetVersion() const = 0;
  virtual bool hasTarget() const = 0;

  /// Align `source_B` (body frame of the scan) onto the installed target,
  /// starting from `T_init_M_B`. Returns a result with converged=false when
  /// there is no usable target.
  virtual RegResult align(const CloudXYZ& source_B, const BASIC::SE3& T_init_M_B) = 0;

  virtual const char* name() const = 0;

  /// Builds the adapter named by params.type.
  static std::unique_ptr<IRegistrar> Create(const RegParams& params);
};

}  // namespace loc
}  // namespace LI2Sup

#endif  // LOC_REGISTRAR_H_
