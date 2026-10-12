#include "loc/pose_composer.h"

#include <cmath>
#include <algorithm>

#include "loc/se3_math.h"

namespace LI2Sup {
namespace loc {

void PoseComposer::reset(const BASIC::SE3& T_M_O) {
  T_M_O_out_ = orthonormalized(T_M_O);
  started_ = true;
  snap_requested_ = false;
}

PoseComposer::Output PoseComposer::compose(const BASIC::SE3& T_M_O_est,
                                           const BASIC::SE3& T_O_B,
                                           double stamp) {
  Output out;

  if (!started_) {
    // Nothing has been applied yet: adopt the estimate outright. This is the
    // initialization jump, and the caller wants to know about it.
    T_M_O_out_ = orthonormalized(T_M_O_est);
    started_ = true;
    snap_requested_ = false;
    last_stamp_ = stamp;
    out.T_M_O  = T_M_O_out_;
    out.T_M_B  = T_M_O_out_ * T_O_B;
    out.jumped = true;
    return out;
  }

  const double dt = (last_stamp_ >= 0.0 && stamp > last_stamp_)
                        ? std::min(stamp - last_stamp_, 1.0)  // clamp after a stall
                        : 0.0;
  last_stamp_ = stamp;

  // The correction as the robot experiences it.
  const BASIC::SE3 T_est = T_M_O_est * T_O_B;
  const BASIC::SE3 T_cur = T_M_O_out_ * T_O_B;
  const Vec6d delta = logSE3(T_cur.inverse() * T_est);
  const double dr = delta.head<3>().norm();
  const double dt_norm = delta.tail<3>().norm();

  out.pending_t = dt_norm;
  out.pending_r = dr;

  const bool snap = snap_requested_ ||
                    dt_norm > params_.snap_t ||
                    dr > params_.snap_r;

  if (snap) {
    T_M_O_out_ = orthonormalized(T_M_O_est);
    snap_requested_ = false;
    out.jumped = true;
    out.pending_t = 0.0;
    out.pending_r = 0.0;
  } else if (dt > 0.0 && (dt_norm > 1e-9 || dr > 1e-9)) {
    // Fraction of the remaining correction we are allowed to absorb this frame:
    // capped by a linear speed, an angular speed, and a first-order time
    // constant so the tail does not creep forever.
    double s = 1.0 - std::exp(-dt / std::max(params_.tau, 1e-3));
    if (dt_norm > 1e-9) s = std::min(s, params_.v_max * dt / dt_norm);
    if (dr > 1e-9)      s = std::min(s, params_.w_max * dt / dr);
    s = std::clamp(s, 0.0, 1.0);

    const BASIC::SE3 T_new = T_cur * expSE3(s * delta);
    // The conjugation below is what amplified any non-orthogonality in T_O_B
    // by k^2 per frame; renormalizing here bounds it regardless of the input.
    T_M_O_out_ = orthonormalized(T_new * T_O_B.inverse());

    out.pending_t = (1.0 - s) * dt_norm;
    out.pending_r = (1.0 - s) * dr;
  }

  out.T_M_O = T_M_O_out_;
  out.T_M_B = T_M_O_out_ * T_O_B;
  return out;
}

}  // namespace loc
}  // namespace LI2Sup
