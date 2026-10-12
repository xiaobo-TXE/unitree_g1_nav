/**
 * @file loc_stats.h
 * @brief What the backend learned about its own behaviour, and how it says so.
 *
 * This exists because of one constraint: on the board the only channel back is
 * a pasted log. Anything not printed did not happen. So the module keeps every
 * number a bring-up or a post-mortem needs and renders it in three places --
 * a periodic heartbeat, an anomaly line with full context, and an end-of-run
 * summary -- instead of scattering LOG() calls through the hot path.
 *
 * Percentiles, not averages. The board question is never "what is the mean
 * alignment time", it is "does the slowest 5% still fit in the budget between
 * two matches". A mean hides exactly the tail that decides that.
 *
 * Thread affinity: everything here is written on T1 (the backend thread)
 * except notePush/noteDrop, which are called on T0 and go to atomics -- T0
 * must never wait on a lock T1 holds, that is the whole safety argument of
 * this module (see map_localizer.h).
 */
#ifndef LOC_LOC_STATS_H_
#define LOC_LOC_STATS_H_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "basic/alias.h"
#include "basic/Manifold.h"
#include "loc/drift_estimator.h"
#include "loc/registrar.h"

namespace LI2Sup {
namespace loc {

/// A stream of samples that answers "typical" and "worst" at report time.
class Series {
public:
  void add(double v);

  std::size_t count() const { return count_; }
  bool   empty() const { return count_ == 0; }
  double mean() const { return count_ ? sum_ / static_cast<double>(count_) : 0.0; }
  double min() const { return count_ ? min_ : 0.0; }
  double max() const { return count_ ? max_ : 0.0; }

  /// Quantile over the samples stored since index `from`; `from` = 0 is the
  /// whole run, and a heartbeat passes the mark it took last time.
  double quantile(double q, std::size_t from = 0) const;
  /// Index one past the newest stored sample, for use as the next `from`.
  std::size_t mark() const { return v_.size(); }

  /// "p50 12.1  p95 19.4  max 61.2  (n 4707)", values multiplied by `scale`.
  std::string describe(double scale = 1.0, int precision = 3) const;

private:
  /// Samples are kept so quantiles stay exact; count/mean/min/max stay exact
  /// past the cap, only the percentiles then describe the first kMaxStored.
  /// At the match rate (~2.4 Hz) the cap is about a day of running.
  static constexpr std::size_t kMaxStored = 200000;

  std::vector<double> v_;
  std::size_t count_ = 0;
  double sum_ = 0.0, min_ = 0.0, max_ = 0.0;
};

/// Everything worth reporting about one run. Owned by MapLocalizer.
class RunStats {
public:
  RunStats() = default;
  RunStats(const RunStats&) = delete;
  RunStats& operator=(const RunStats&) = delete;

  // ---- recording (T1) -----------------------------------------------------

  void noteMatch(bool accepted, RejectReason reason, const RegResult& reg,
                 const UpdateReport& rep, double gap_s);
  void noteSubmapRebuild(double ms, std::size_t keyframes, std::size_t points);
  void noteSubmapInvalid();
  void noteScanBuild(double ms);
  /// One frame arrived from T0, with the scan time it carries.
  ///
  /// The point is not the count -- `frames_in` already has that -- but the
  /// INTERVAL between consecutive frames, which is what reveals frames that
  /// never reached the node at all. `queue dropped` cannot: it only counts
  /// frames the backend was too slow to take, and a frame lost in transport
  /// was never queued. Two parallel bag replays were measured delivering half
  /// the lidar frames while every counter in this summary read healthy.
  void noteFrameStamp(double t);

  /// Yardstick for the delivery check; 0 (default) infers it. See LogParams.
  void setExpectedFrameHz(double hz) { expected_hz_ = hz; }
  void noteSearch(bool accepted, double ms);
  void noteLost();
  /// The frontend was declared unusable. Counted separately from LOST because
  /// it is a different failure with a different consequence: LOST keeps
  /// publishing, this stops output altogether.
  void noteFrontendFailed();
  /// A confirmed (re)initialization: how long it took and how many tries.
  void noteFix(bool relocalization, double seconds, int attempts);

  // ---- recording (T0, lock-free) -----------------------------------------

  void notePush(double ms);
  void noteDrop();

  // ---- reading ------------------------------------------------------------

  std::size_t matches() const { return matches_; }
  std::size_t accepted() const { return accepted_; }
  std::size_t lostEvents() const { return lost_events_; }

  /// Context the renderers cannot get on their own.
  struct Context {
    const char* state = "?";
    double wall_s = 0.0;            ///< since start()
    double stamp = 0.0;             ///< scan time of the last processed frame
    BASIC::SE3 drift;               ///< current T_M_O
    BASIC::V3  gravity_O = BASIC::V3(0, 0, -9.8);  ///< frontend's g, in O
    /// Frontend IMU-input health, cumulative. Copied field by field rather than
    /// as FrontendFrame::ImuHealth so this module keeps its independence from
    /// the frame queue; see that struct for what the numbers mean and why they
    /// exist.
    std::uint64_t imu_refused = 0;
    double        imu_max_gap_s = 0.0;
    double        imu_period_s = 0.0;
    std::size_t frames_in = 0;
    std::size_t scan_builds = 0;
    std::size_t queue_depth = 0;
    std::size_t keyframes = 0;      ///< resident in the map
    std::size_t cache_hits = 0;
    std::size_t cache_misses = 0;
    std::size_t prefetched = 0;
    double      disk_read_ms = 0.0;     ///< total spent reading keyframe clouds
    double      disk_read_max_ms = 0.0; ///< the single worst read
    const char* registrar = "?";
  };

  /// One dense line covering the window since the previous call. Empty when
  /// nothing at all has happened, so a parked robot does not fill the log.
  std::string heartbeat(const Context& ctx);

  /// The end-of-run block: the one thing worth pasting back in full.
  std::string summary(const Context& ctx) const;

private:
  /// The "frame delivery" line of the summary; empty when too few frames.
  std::string deliveryLine() const;
  std::string imuLine(const Context& ctx) const;

  std::size_t matches_ = 0, accepted_ = 0, rejected_ = 0;
  std::size_t reject_counts_[16] = {};
  std::size_t lost_events_ = 0, fixes_ = 0, relocs_ = 0;
  std::size_t frontend_failures_ = 0;
  std::size_t searches_ = 0, searches_ok_ = 0;
  std::size_t submap_rebuilds_ = 0, submap_invalid_ = 0;
  std::size_t submap_keyframes_ = 0, submap_points_ = 0;

  double fix_seconds_ = 0.0;
  int    fix_attempts_ = 0;

  Series inlier_, rmse_, corr_t_, corr_r_, chi2_, reg_ms_, submap_ms_, scan_ms_, gap_s_,
      search_ms_;

  /// Frame arrival, for the delivery check in the summary.
  Series frame_dt_;
  double first_stamp_ = 0.0, last_stamp_ = 0.0, expected_hz_ = 0.0;

  /// T0 side. Sum in microseconds so the atomic stays integral.
  std::atomic<std::size_t>   pushes_{0};
  std::atomic<std::size_t>   drops_{0};
  std::atomic<std::uint64_t> push_us_sum_{0};
  std::atomic<std::uint64_t> push_us_max_{0};

  /// Where the previous heartbeat stopped, per series.
  struct Marks {
    std::size_t matches = 0, accepted = 0, rejected = 0, submaps = 0;
    std::size_t inlier = 0, rmse = 0, corr_t = 0, corr_r = 0, reg_ms = 0, submap_ms = 0;
    std::size_t frames_in = 0;
    std::uint64_t imu_refused = 0;
    double wall_s = 0.0;
  } mark_;
};

}  // namespace loc
}  // namespace LI2Sup

#endif  // LOC_LOC_STATS_H_
