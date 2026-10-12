/**
 * @file pose_composer.h
 * @brief Turns the backend's drift estimate into an output pose that never jumps.
 *
 * Two things happen here and nowhere else:
 *  1. Composition: T_M_B = T_M_O_out * T_O_B. The frontend's motion passes
 *     through untouched — no filtering, no lag — because only the correction is
 *     rate limited, never the odometry.
 *  2. Rate limiting, evaluated in the BODY frame. What a planner sees jump is
 *     the robot's pose, and a small yaw change in T_M_O becomes metres of body
 *     motion through the lever arm out to the odometry origin. Limiting the
 *     T_M_O delta directly would therefore be the wrong quantity; §16.6 of the
 *     design doc is explicit about this.
 *
 * Large corrections that have actually been vetted — initialization, an accepted
 * relocalization — are applied in one step via requestSnap(), and the caller is
 * told so it can flag downstream (Output::jumped).
 *
 * Thread affinity: frontend thread (T0) only. It holds the "already applied"
 * drift, which no other thread reads or writes.
 */
#ifndef LOC_POSE_COMPOSER_H_
#define LOC_POSE_COMPOSER_H_

#include "basic/alias.h"
#include "basic/Manifold.h"
#include "loc/loc_params.h"

namespace LI2Sup {
namespace loc {

class PoseComposer {
public:
  struct Output {
    BASIC::SE3 T_M_B;         ///< the pose to publish
    BASIC::SE3 T_M_O;         ///< the drift actually applied to produce it
    bool   jumped = false;    ///< this output snapped rather than ramped
    double pending_t = 0.0;   ///< body-frame correction still to be absorbed [m]
    double pending_r = 0.0;   ///< [rad]
  };

  explicit PoseComposer(const ComposerParams& params) : params_(params) {}

  /// Advance the applied drift toward `T_M_O_est` and compose the output pose.
  /// `stamp` is the frame time; the first call only establishes the clock.
  Output compose(const BASIC::SE3& T_M_O_est, const BASIC::SE3& T_O_B, double stamp);

  /// Apply the next estimate in one step instead of ramping to it.
  void requestSnap() { snap_requested_ = true; }

  /// Adopt a drift with no ramp and no snap flag (startup).
  void reset(const BASIC::SE3& T_M_O);

  const BASIC::SE3& applied() const { return T_M_O_out_; }
  bool started() const { return started_; }

private:
  ComposerParams params_;
  BASIC::SE3 T_M_O_out_;
  bool   started_ = false;
  bool   snap_requested_ = false;
  double last_stamp_ = -1.0;
};

}  // namespace loc
}  // namespace LI2Sup

#endif  // LOC_POSE_COMPOSER_H_
