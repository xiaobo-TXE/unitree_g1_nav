#include "loc/frontend_watch.h"

#include <cmath>

namespace LI2Sup {
namespace loc {

const char* FrontendWatch::toString(Cause c) {
  switch (c) {
    case Cause::kNone:       return "none";
    case Cause::kNonFinite:  return "non_finite";
    case Cause::kSpeed:      return "speed";
    case Cause::kOdomRadius: return "odom_radius";
  }
  return "?";
}

void FrontendWatch::reset() {
  have_prev_ = false;
  fast_streak_ = 0;
  healthy_streak_ = 0;
  failed_ = false;
  cause_ = Cause::kNone;
}

void FrontendWatch::fail(Cause c, double value, double limit, Verdict* v) {
  v->changed = !failed_;
  failed_ = true;
  cause_ = c;
  healthy_streak_ = 0;
  v->failed = true;
  v->cause = c;
  v->value = value;
  v->limit = limit;
}

FrontendWatch::Verdict FrontendWatch::observe(double t, const BASIC::SE3& T_O_B) {
  Verdict v;
  v.failed = failed_;
  v.cause = cause_;

  const BASIC::V3 p = T_O_B.t_;

  // 1. Non-finite. Checked on the rotation too: the pose is used as a rigid
  //    transform, and a NaN in R spreads into every transformed point without
  //    ever showing up in the translation the logs print.
  if (!p.allFinite() || !T_O_B.R_.allFinite()) {
    fail(Cause::kNonFinite, 0.0, 0.0, &v);
    // No prev_ update: a non-finite pose is not a position to measure from.
    return v;
  }

  // 2. Further from the odometry origin than the map is wide. The odometry
  //    origin is wherever the robot started, which is on the map, so the
  //    distance from it is bounded by the map's own extent for as long as the
  //    robot is on the map. Beyond that it is not, whatever anything says.
  const double r = static_cast<double>(p.norm());
  if (odom_radius_ > 0.0 && r > odom_radius_) {
    fail(Cause::kOdomRadius, r, odom_radius_, &v);
    return v;
  }

  // 3. Implied body speed. Needs two frames and a sane interval: the backend
  //    receives frames in batches and a long stall would otherwise read as a
  //    huge speed, which is a statement about the queue, not the frontend.
  bool speed_checked = false;
  if (have_prev_) {
    const double dt = t - prev_t_;
    if (dt > 1e-6 && dt <= params_.max_gap_s) {
      speed_checked = true;
      const double speed = static_cast<double>((p - prev_p_).norm()) / dt;
      if (speed > params_.max_speed) {
        // One frame over the limit is a timestamp glitch or a dropped frame;
        // several in a row is the frontend leaving.
        if (++fast_streak_ >= params_.speed_frames) {
          fail(Cause::kSpeed, speed, params_.max_speed, &v);
          prev_t_ = t;
          prev_p_ = p;
          return v;
        }
      } else {
        fast_streak_ = 0;
      }
    }
  }

  have_prev_ = true;
  prev_t_ = t;
  prev_p_ = p;

  // Only a frame that was actually measured counts towards recovery. Without
  // this, a frontend that stopped publishing plausibly-spaced frames would
  // count its way back purely on arrivals.
  if (failed_ && speed_checked && fast_streak_ == 0) ++healthy_streak_;
  return v;
}

}  // namespace loc
}  // namespace LI2Sup
