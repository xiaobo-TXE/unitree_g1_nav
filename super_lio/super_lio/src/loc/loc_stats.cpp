#include "loc/loc_stats.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include <Eigen/Dense>

namespace LI2Sup {
namespace loc {

namespace {

constexpr double kRad2Deg = 180.0 / M_PI;

std::string fixed(double v, int precision) {
  std::ostringstream o;
  o << std::fixed << std::setprecision(precision) << v;
  return o.str();
}

/// "p50 12.1  p95 19.4  max 61.2  (n 4707)" -- the shape every timing line uses.
std::string tail(const Series& s, double scale, int precision, double hi_q = 0.95) {
  if (s.empty()) return "(none)";
  std::ostringstream o;
  o << std::fixed << std::setprecision(precision)
    << "p50 " << s.quantile(0.50) * scale
    << "  p" << static_cast<int>(hi_q * 100.0) << " " << s.quantile(hi_q) * scale
    << "  max " << s.max() * scale
    << "   (n " << s.count() << ")";
  return o.str();
}

/// "0.972/0.881" -- median and tail in the compact heartbeat form.
std::string pair_of(const Series& s, std::size_t from, double scale, int precision,
                    double hi_q = 0.95) {
  if (s.mark() <= from) return "-";
  return fixed(s.quantile(0.50, from) * scale, precision) + "/" +
         fixed(s.quantile(hi_q, from) * scale, precision);
}

}  // namespace

// ---------------------------------------------------------------------------

void Series::add(double v) {
  if (!std::isfinite(v)) return;   // an infinite rmse means "no correspondences"
  if (count_ == 0) {
    min_ = max_ = v;
  } else {
    min_ = std::min(min_, v);
    max_ = std::max(max_, v);
  }
  ++count_;
  sum_ += v;
  if (v_.size() < kMaxStored) v_.push_back(v);
}

double Series::quantile(double q, std::size_t from) const {
  if (from >= v_.size()) return 0.0;
  std::vector<double> w(v_.begin() + static_cast<std::ptrdiff_t>(from), v_.end());
  if (w.empty()) return 0.0;
  const std::size_t k = static_cast<std::size_t>(
      std::llround(q * static_cast<double>(w.size() - 1)));
  std::nth_element(w.begin(), w.begin() + static_cast<std::ptrdiff_t>(k), w.end());
  return w[k];
}

std::string Series::describe(double scale, int precision) const {
  return tail(*this, scale, precision);
}

// ---------------------------------------------------------------------------

void RunStats::noteMatch(bool accepted, RejectReason reason, const RegResult& reg,
                         const UpdateReport& rep, double gap_s) {
  ++matches_;
  if (accepted) {
    ++accepted_;
  } else {
    ++rejected_;
    const auto i = static_cast<std::size_t>(reason);
    if (i < sizeof(reject_counts_) / sizeof(reject_counts_[0])) ++reject_counts_[i];
  }
  inlier_.add(reg.inlier_ratio);
  rmse_.add(reg.rmse);
  reg_ms_.add(reg.time_ms);
  corr_t_.add(rep.body_correction_t);
  corr_r_.add(rep.body_correction_r);
  chi2_.add(rep.chi2);
  if (gap_s > 0.0) gap_s_.add(gap_s);
}

void RunStats::noteSubmapRebuild(double ms, std::size_t keyframes, std::size_t points) {
  ++submap_rebuilds_;
  submap_ms_.add(ms);
  submap_keyframes_ += keyframes;
  submap_points_ += points;
}

void RunStats::noteSubmapInvalid() { ++submap_invalid_; }

void RunStats::noteFrameStamp(double t) {
  if (!(t > 0.0)) return;
  if (first_stamp_ == 0.0) {
    first_stamp_ = t;
  } else if (t > last_stamp_) {
    frame_dt_.add(t - last_stamp_);
  }
  last_stamp_ = t;
}

void RunStats::noteScanBuild(double ms) { scan_ms_.add(ms); }

void RunStats::noteSearch(bool accepted, double ms) {
  ++searches_;
  if (accepted) ++searches_ok_;
  search_ms_.add(ms);
}

void RunStats::noteLost() { ++lost_events_; }

void RunStats::noteFrontendFailed() { ++frontend_failures_; }

void RunStats::noteFix(bool relocalization, double seconds, int attempts) {
  if (relocalization) {
    ++relocs_;
  } else {
    ++fixes_;
  }
  // A relocalization has no request to measure from, so it reports nothing
  // rather than overwriting the initialization timing with a zero.
  if (seconds > 0.0) {
    fix_seconds_ = seconds;
    fix_attempts_ = attempts;
  }
}

void RunStats::notePush(double ms) {
  pushes_.fetch_add(1, std::memory_order_relaxed);
  const auto us = static_cast<std::uint64_t>(std::max(0.0, ms) * 1000.0);
  push_us_sum_.fetch_add(us, std::memory_order_relaxed);
  auto prev = push_us_max_.load(std::memory_order_relaxed);
  while (us > prev && !push_us_max_.compare_exchange_weak(prev, us,
                                                          std::memory_order_relaxed)) {
  }
}

void RunStats::noteDrop() { drops_.fetch_add(1, std::memory_order_relaxed); }

// ---------------------------------------------------------------------------

std::string RunStats::heartbeat(const Context& ctx) {
  const std::size_t d_matches = matches_ - mark_.matches;
  const std::size_t d_rejects = rejected_ - mark_.rejected;
  const std::size_t d_submaps = submap_rebuilds_ - mark_.submaps;
  const std::size_t d_frames  = ctx.frames_in - mark_.frames_in;

  // A robot standing still with the backend idle produces nothing to say.
  if (d_matches == 0 && d_frames == 0) return std::string();

  const BASIC::V3 t = ctx.drift.t_;
  const double yaw = std::atan2(static_cast<double>(ctx.drift.R_(1, 0)),
                                static_cast<double>(ctx.drift.R_(0, 0))) * kRad2Deg;

  // Three numbers that together say whether a tilted T_M_O is the frontend's
  // doing. M is gravity aligned (measured on this map: the heading-correlated
  // component of map-up in the keyframe body frames is 0.17 deg), so:
  //
  //   drift_tilt  how far T_M_O has to rotate O's z back to vertical
  //   grav_tilt   how far the frontend's own gravity estimate has left (0,0,-g),
  //               i.e. how far O stopped being gravity aligned
  //   resid       angle(R_MO * up_O_true, z_M), where up_O_true = -g/|g|
  //
  // If the frontend's gravity state explains the tilt, the first two match and
  // resid is ~0. A large resid means something OTHER than the frontend's
  // gravity is tilting the drift, and this heartbeat is how to tell them apart
  // on the next run -- the two 2026-09 runs had no gravity logging at all.
  const Eigen::Vector3d gO(static_cast<double>(ctx.gravity_O.x()),
                           static_cast<double>(ctx.gravity_O.y()),
                           static_cast<double>(ctx.gravity_O.z()));
  const Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
  const double gn = gO.norm();
  const Eigen::Vector3d up_O = gn > 1e-6 ? Eigen::Vector3d(-gO / gn) : z_axis;
  const double grav_tilt = std::acos(std::clamp(up_O.z(), -1.0, 1.0)) * kRad2Deg;

  const Eigen::Matrix3d R_MO = ctx.drift.R_.template cast<double>();
  const Eigen::Vector3d z_O_in_M = R_MO * z_axis;
  const double drift_tilt = std::acos(std::clamp(z_O_in_M.z(), -1.0, 1.0)) * kRad2Deg;
  const Eigen::Vector3d up_in_M = R_MO * up_O;
  const double grav_resid = std::acos(std::clamp(up_in_M.z(), -1.0, 1.0)) * kRad2Deg;

  std::ostringstream o;
  o << std::fixed << std::setprecision(1)
    << " ---> [MapLoc] " << ctx.state << " +" << ctx.wall_s << "s"
    << " | frames " << d_frames
    << " match " << d_matches;
  if (d_rejects) o << " REJ " << d_rejects;
  if (d_matches) {
    o << " | inlier " << pair_of(inlier_, mark_.inlier, 1.0, 2, 0.05)
      << " rmse " << pair_of(rmse_, mark_.rmse, 1.0, 3)
      << " | corr " << pair_of(corr_t_, mark_.corr_t, 1000.0, 1) << " mm "
      << pair_of(corr_r_, mark_.corr_r, kRad2Deg, 2) << " deg"
      << " | reg " << pair_of(reg_ms_, mark_.reg_ms, 1.0, 1) << " ms";
  }
  if (d_submaps) {
    o << " | submap " << d_submaps << "x " << pair_of(submap_ms_, mark_.submap_ms, 1.0, 1)
      << " ms";
  }
  o << " | q" << ctx.queue_depth << " drop" << drops_.load(std::memory_order_relaxed)
    << " | T_M_O (" << std::setprecision(3) << t.x() << ", " << t.y() << ", " << t.z()
    << ") yaw " << std::setprecision(2) << yaw << " deg"
    << " tilt " << drift_tilt << " deg"
    << " | grav (" << std::setprecision(3) << gO.x() << ", " << gO.y() << ", " << gO.z()
    << ") tilt " << std::setprecision(2) << grav_tilt << " deg"
    << " resid " << grav_resid << " deg";

  // Only when it moved. A healthy IMU stream says nothing here; the running
  // totals are in the summary either way.
  if (ctx.imu_refused > mark_.imu_refused) {
    o << " | IMU refused " << (ctx.imu_refused - mark_.imu_refused)
      << " (total " << ctx.imu_refused << ", worst gap " << std::setprecision(3)
      << ctx.imu_max_gap_s << " s)";
  }

  mark_.matches   = matches_;
  mark_.imu_refused = ctx.imu_refused;
  mark_.accepted  = accepted_;
  mark_.rejected  = rejected_;
  mark_.submaps   = submap_rebuilds_;
  mark_.frames_in = ctx.frames_in;
  mark_.wall_s    = ctx.wall_s;
  mark_.inlier    = inlier_.mark();
  mark_.rmse      = rmse_.mark();
  mark_.corr_t    = corr_t_.mark();
  mark_.corr_r    = corr_r_.mark();
  mark_.reg_ms    = reg_ms_.mark();
  mark_.submap_ms = submap_ms_.mark();
  return o.str();
}

/// "frame delivery  23414 of 23418 expected (99.98% at 10.0 Hz)".
///
/// Frames that never arrive are invisible to every other number here: the
/// backend sees a shorter stream and reports perfect health on it, and
/// `queue dropped` stays zero because a frame lost in transport was never
/// queued. Two bag replays running at once were measured delivering half the
/// lidar frames each while the summary looked clean, which turned one A/B into
/// nonsense before anyone noticed.
///
/// The nominal period is taken as the 10th percentile of the observed
/// intervals rather than a hard-coded rate: dropping frames stretches some
/// intervals but cannot shrink the shortest ones, so the low quantile still
/// recovers the true sensor period whatever the lidar runs at.
std::string RunStats::deliveryLine() const {
  if (frame_dt_.count() < 100) return {};
  // A configured rate wins: interval statistics alone cannot see uniform loss.
  const double period = expected_hz_ > 0.0 ? 1.0 / expected_hz_
                                           : frame_dt_.quantile(0.10);
  const double span   = last_stamp_ - first_stamp_;
  if (!(period > 1e-6) || !(span > 0.0)) return {};

  const double expected = span / period + 1.0;
  const double got      = static_cast<double>(frame_dt_.count()) + 1.0;
  const double pct      = 100.0 * got / expected;

  std::ostringstream o;
  o << std::fixed << std::setprecision(0)
    << "\n  frame delivery  " << got << " of " << expected << " expected ("
    << std::setprecision(1) << pct << "% at " << 1.0 / period << " Hz"
    << (expected_hz_ > 0.0 ? ", configured" : ", inferred") << ")";
  // Anything below ~98% means frames are being lost upstream of the queue.
  // Say so in the summary rather than leaving it to be derived.
  if (pct < 98.0) {
    o << "   *** " << std::setprecision(0) << (expected - got)
      << " FRAMES NEVER ARRIVED -- this run's numbers describe a truncated"
         " input, not the algorithm ***";
  }
  return o.str();
}

/// "imu input       period 4.98 ms (200.6 Hz), 0 samples refused".
///
/// The companion to deliveryLine(). Both sensors are subscribed `best_effort`,
/// so both can lose data silently, but only the lidar side was ever counted --
/// and the IMU is the side that can diverge the filter rather than merely
/// starve it. `period` is also what says whether lio.sensor.imu_max_dt is set
/// sanely for this sensor: the guard has to sit well above it.
std::string RunStats::imuLine(const Context& ctx) const {
  if (!(ctx.imu_period_s > 0.0)) return {};
  std::ostringstream o;
  o << std::fixed
    << "\n  imu input       period " << std::setprecision(2) << ctx.imu_period_s * 1000.0
    << " ms (" << std::setprecision(1) << 1.0 / ctx.imu_period_s << " Hz), "
    << ctx.imu_refused << " samples refused";
  if (ctx.imu_refused) {
    o << ", worst gap " << std::setprecision(3) << ctx.imu_max_gap_s << " s"
      << "   *** the filter did not advance across those gaps; the pose was"
         " stale by that much and the scan match had to make it up ***";
  }
  return o.str();
}

std::string RunStats::summary(const Context& ctx) const {
  const std::size_t pushes = pushes_.load(std::memory_order_relaxed);
  const double push_mean_us =
      pushes ? static_cast<double>(push_us_sum_.load(std::memory_order_relaxed)) /
                   static_cast<double>(pushes)
             : 0.0;
  const double push_max_us =
      static_cast<double>(push_us_max_.load(std::memory_order_relaxed));

  // What fraction of wall time the backend actually spent working. This is the
  // number that says whether the board keeps up, and it is the one thing no
  // topic carries.
  const double busy_s = (reg_ms_.mean() * static_cast<double>(reg_ms_.count()) +
                         submap_ms_.mean() * static_cast<double>(submap_ms_.count()) +
                         scan_ms_.mean() * static_cast<double>(scan_ms_.count()) +
                         search_ms_.mean() * static_cast<double>(search_ms_.count())) /
                        1000.0;

  const std::size_t looks = ctx.cache_hits + ctx.cache_misses;
  const double hit_rate = looks ? 100.0 * static_cast<double>(ctx.cache_hits) /
                                      static_cast<double>(looks)
                                : 0.0;

  std::ostringstream o;
  o << std::fixed;
  o << "\n===== MapLoc run summary ====================================================="
    << "\n  quantiles are p50/p95 unless named otherwise"
    << std::setprecision(1)
    << "\n  wall            " << ctx.wall_s << " s        last scan stamp "
    << std::setprecision(3) << ctx.stamp
    << "\n  state           " << ctx.state << "        registrar " << ctx.registrar
    << "\n  frames in       " << ctx.frames_in << "        scan builds " << ctx.scan_builds
    << "        queue dropped " << drops_.load(std::memory_order_relaxed)
    << deliveryLine()
    << imuLine(ctx)
    << "\n  T0 pushFrame    mean " << std::setprecision(1) << push_mean_us << " us   max "
    << push_max_us << " us   (n " << pushes << ")   [the frontend budget is 100 ms/frame]"
    << "\n  matches         " << matches_ << "        accepted " << accepted_ << " ("
    << std::setprecision(2)
    << (matches_ ? 100.0 * static_cast<double>(accepted_) / static_cast<double>(matches_)
                 : 0.0)
    << "%)        rejected " << rejected_;

  if (rejected_) {
    o << "\n    rejects       ";
    for (std::size_t i = 0; i < sizeof(reject_counts_) / sizeof(reject_counts_[0]); ++i) {
      if (!reject_counts_[i]) continue;
      o << toString(static_cast<RejectReason>(i)) << " " << reject_counts_[i] << "   ";
    }
  }

  o << std::setprecision(3) << "\n  inlier ratio    ";
  if (inlier_.empty()) {
    o << "(none)";
  } else {
    // The interesting extreme here is the LOW one -- "max 1.000" says nothing.
    o << "p50 " << inlier_.quantile(0.50) << "  p5 " << inlier_.quantile(0.05)
      << "  min " << inlier_.min() << "   (n " << inlier_.count() << ")";
  }
  o
    << "\n  rmse [m]        " << tail(rmse_, 1.0, 3)
    << "\n  body corr [mm]  " << tail(corr_t_, 1000.0, 1)
    << "\n  body corr [deg] " << tail(corr_r_, kRad2Deg, 3)
    << "\n  chi2            " << tail(chi2_, 1.0, 2)
    << "\n  match gap [s]   " << tail(gap_s_, 1.0, 2)
    << "\n  register [ms]   " << tail(reg_ms_, 1.0, 2)
    << "\n  submap bld [ms] " << tail(submap_ms_, 1.0, 2);
  if (submap_rebuilds_) {
    o << "   avg " << submap_keyframes_ / submap_rebuilds_ << " kf / "
      << submap_points_ / submap_rebuilds_ << " pts";
  }
  if (submap_invalid_) o << "   INVALID " << submap_invalid_ << "x (off the map)";
  o << "\n  scan bld [ms]   " << tail(scan_ms_, 1.0, 2);
  if (searches_) {
    o << "\n  search [ms]     " << tail(search_ms_, 1.0, 1) << "   accepted " << searches_ok_
      << "/" << searches_;
  }
  o << std::setprecision(1)
    << "\n  backend busy    " << busy_s << " s of " << ctx.wall_s << " s ("
    << std::setprecision(2)
    << (ctx.wall_s > 0.0 ? 100.0 * busy_s / ctx.wall_s : 0.0) << "%)"
    << "\n  map             " << ctx.keyframes << " keyframes, cache " << hit_rate
    << "% hit (" << ctx.cache_misses << " misses, " << ctx.prefetched << " prefetched, "
    << std::setprecision(2) << ctx.disk_read_ms / 1000.0 << " s on disk, worst read "
    << ctx.disk_read_max_ms << " ms)"
    << "\n  events          initializations " << fixes_ << "   relocalizations " << relocs_
    << "   LOST " << lost_events_;
  if (frontend_failures_) {
    o << "\n  frontend        " << frontend_failures_
      << " x FAILED   *** the frontend produced a pose the robot cannot be in;"
         " localization output was stopped for the duration ***";
  }
  if (fixes_ + relocs_) {
    o << "   (last fix " << std::setprecision(2) << fix_seconds_ << " s, " << fix_attempts_
      << " attempts)";
  }
  o << "\n=============================================================================";
  return o.str();
}

}  // namespace loc
}  // namespace LI2Sup
