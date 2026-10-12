/**
 * @file frontend_watch.h
 * @brief Is the frontend's own output still usable?
 *
 * TRACKING and LOST are both statements about the MAP: "I can match" and "I
 * cannot match". Both take for granted that `T_O_B` itself is sound, and the
 * whole output path is built on that -- LOST deliberately keeps publishing,
 * dead-reckoning from the last good drift, because a few seconds of odometry
 * is genuinely useful.
 *
 * On 2026-09-21 that assumption broke twice on the board. In one run the
 * frontend stepped 7 m in z while the robot stood on a flat floor and stayed
 * there; in the other it left at ~14.5 m/s and passed 8 km. Nothing in the
 * backend was equipped to notice: `drift_valid_` is set once at the first fix
 * and was never cleared, so `/loc/odom`, `/loc/robo/odom`, `/loc/imu/odom` and
 * the `map -> world` transform all kept flowing, carrying `T_M_O` times a pose
 * in flight. A planner cannot defend itself against that -- the numbers are
 * confident and wrong.
 *
 * This watches the frontend pose stream alone and answers one question: has
 * the frontend produced something it physically cannot have produced? It says
 * nothing about the map. What it catches:
 *
 *   non_finite  a NaN or Inf reached the pose. Never legitimate.
 *   speed       implied body speed over a platform limit, held for several
 *               frames so one bad frame cannot trip it.
 *   odom_radius the robot is further from the odometry origin than the map is
 *               wide. It cannot be on the map, whatever the map says.
 *
 * All three are bounds no correct run approaches, not tuned thresholds: the
 * point is to be certain when it fires, because the consequence is that
 * localization output stops. A frontend that is merely drifting is not this
 * module's business -- that is what the gates and the drift estimator are for.
 */
#ifndef LOC_FRONTEND_WATCH_H_
#define LOC_FRONTEND_WATCH_H_

#include <cstdint>

#include "basic/alias.h"
#include "basic/Manifold.h"
#include "loc/loc_params.h"

namespace LI2Sup {
namespace loc {

class FrontendWatch {
public:
  /// Why the frontend was declared unusable. kNone means it has not been.
  enum class Cause : std::uint8_t { kNone = 0, kNonFinite, kSpeed, kOdomRadius };

  struct Verdict {
    bool   failed = false;   ///< the state right now, not just this frame
    bool   changed = false;  ///< this frame flipped it, so the caller logs once
    Cause  cause = Cause::kNone;
    double value = 0.0;      ///< the number that tripped it, in the cause's unit
    double limit = 0.0;      ///< what it was compared against
  };

  explicit FrontendWatch(const FrontendWatchParams& p) : params_(p) {}

  /// The bound for `odom_radius`, from the loaded map. Passing 0 disables that
  /// check, which is what a caller with no map should do.
  void setOdomRadius(double radius_m) { odom_radius_ = radius_m; }
  double odomRadius() const { return odom_radius_; }

  /// One frontend frame, in arrival order. Cheap: no allocation, a few flops.
  Verdict observe(double t, const BASIC::SE3& T_O_B);

  bool failed() const { return failed_; }
  Cause cause() const { return cause_; }

  /// Frames of plausible motion since the failure. The caller uses this to
  /// decide when the frontend may be trusted again -- and it should then
  /// demand a fresh fix rather than resuming, because a pose that has flown
  /// and settled is still in the wrong place.
  int healthyStreak() const { return healthy_streak_; }
  bool recovered() const {
    return failed_ && healthy_streak_ >= params_.recover_frames;
  }

  /// Forget everything, including the verdict. For a (re)initialization.
  void reset();

  static const char* toString(Cause c);

private:
  void fail(Cause c, double value, double limit, Verdict* v);

  FrontendWatchParams params_;
  double odom_radius_ = 0.0;

  bool   have_prev_ = false;
  double prev_t_ = 0.0;
  BASIC::V3 prev_p_ = BASIC::V3::Zero();

  int    fast_streak_ = 0;     ///< consecutive frames over the speed limit
  int    healthy_streak_ = 0;
  bool   failed_ = false;
  Cause  cause_ = Cause::kNone;
};

}  // namespace loc
}  // namespace LI2Sup

#endif  // LOC_FRONTEND_WATCH_H_
