/**
 * @file scan_accumulator.h
 * @brief The T0 -> T1 handoff, and the merge of several frontend frames into
 *        one registration source cloud.
 *
 * FrameQueue is the only data path from the frontend thread into the backend.
 * The frontend pushes a *deep copy* of scan_undistort_full_ (that buffer is
 * resized in place every frame, so a shared handle would be read as the next
 * frame's data) together with the odometry pose snapshotted at the same instant.
 * That snapshot is what makes registration latency harmless: the observation is
 * defined at t_k regardless of when the align finishes.
 *
 * ScanAccumulator merges the newest N frames into the body frame of the newest
 * one. A single mid360 sweep is ~8k points, too sparse for a stable GICP
 * covariance; five sweeps put the source density in the same range as the map
 * keyframes. The relative odometry error across 0.5 s is millimetric.
 */
#ifndef LOC_SCAN_ACCUMULATOR_H_
#define LOC_SCAN_ACCUMULATOR_H_

#include <cstdint>
#include <deque>
#include <mutex>
#include <memory>
#include <vector>
#include <condition_variable>

#include "basic/alias.h"
#include "basic/Manifold.h"
#include "loc/keyframe_map.h"
#include "loc/loc_params.h"

namespace LI2Sup {
namespace loc {

/// One frontend frame, owned outright by the backend once queued.
struct FrontendFrame {
  double        t = 0.0;     ///< measures_.lidar.end_time
  BASIC::SE3    T_O_B;       ///< kf_->GetSE3() at that instant
  CloudXYZ::Ptr cloud_B;     ///< deep copy, IMU body frame, undistorted

  /// kf_->GetGravity(): where the frontend thinks gravity points, in O.
  ///
  /// Diagnostic only, nothing consumes it but the heartbeat. The frontend
  /// carries gravity as a filter state (ESKF STATE is "R p v bg ba g", and
  /// Update() re-normalizes g_ to a fixed magnitude, leaving its DIRECTION
  /// free), so O stops being gravity aligned by exactly however far this has
  /// wandered from (0, 0, -g). The two board runs needed 4-6 deg of tilt in
  /// T_M_O against a map that is gravity aligned to 0.17 deg, and this is the
  /// number that says whether the frontend's own gravity estimate accounts for
  /// it. It was never logged, which is why the drift stayed invisible.
  BASIC::V3     gravity_O = BASIC::V3(0, 0, -9.8);

  /// What the frontend filter saw on its IMU input, cumulative since start.
  ///
  /// Diagnostic only, and deliberately three plain numbers rather than the
  /// filter's own struct: loc/ does not include lio/, and the translation lives
  /// in super_lio_maploc.cpp with the rest of the T0 handoff.
  ///
  /// IMU and lidar are both subscribed `best_effort`. Lidar loss the summary
  /// already catches (`frame delivery`); IMU loss had no observation port at
  /// all, and it is the more dangerous of the two -- a gap the filter refuses
  /// to integrate is a pose that did not advance, and a gap it does integrate
  /// is a pose that diverges. The 2026-09-21 runs that flew had 9.24 Hz of
  /// lidar against 10.00 Hz on the healthy one, and nothing whatsoever about
  /// the IMU.
  struct ImuHealth {
    std::uint64_t refused   = 0;    ///< samples the dt guard would not integrate
    double        max_gap_s = 0.0;  ///< worst refused step
    double        period_s  = 0.0;  ///< mean accepted step = the observed IMU period
  } imu;
};

/// Bounded MPSC queue. The producer (T0) never blocks and never waits: when the
/// backend falls behind, the oldest frame is dropped rather than the newest,
/// because localization only ever wants recent geometry.
class FrameQueue {
public:
  explicit FrameQueue(std::size_t capacity = 10) : capacity_(capacity) {}

  /// Called from T0. Returns false if a frame had to be dropped to make room.
  bool push(FrontendFrame frame);

  /// Called from T1. Blocks until a frame is available or stop() is called.
  /// Returns false only when stopped.
  bool waitAndPopAll(std::vector<FrontendFrame>* out);

  /// Non-blocking drain, for tests and for the shutdown path.
  std::size_t popAll(std::vector<FrontendFrame>* out);

  void stop();
  std::size_t size() const;
  std::size_t dropped() const { return dropped_; }

private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<FrontendFrame> q_;
  std::size_t capacity_;
  std::size_t dropped_ = 0;
  bool stopped_ = false;
};

/// The merged observation handed to a registrar.
struct SourceScan {
  double        t = 0.0;     ///< time of the reference (newest) frame
  BASIC::SE3    T_O_B;       ///< odometry pose of the reference frame
  CloudXYZ::Ptr cloud_B;     ///< merged, downsampled, in the reference body frame
  int           n_frames = 0;
  double        build_ms = 0.0;

  bool valid() const { return cloud_B && !cloud_B->empty(); }
};

/// Keeps the last `accum_frames` frames and merges them on demand.
/// Deliberately not a queue consumer itself: MapLocalizer owns the queue and
/// decides when a match is due, this only knows how to build a source cloud.
class ScanAccumulator {
public:
  explicit ScanAccumulator(const ScanParams& params) : params_(params) {}

  /// Add frames drained from the queue, keeping at most accum_frames.
  void add(const std::vector<FrontendFrame>& frames);
  void add(FrontendFrame frame);

  /// Merge what is buffered into the body frame of the newest frame.
  /// Returns an invalid scan when nothing is buffered.
  SourceScan build() const;

  /// Odometry pose and time of the newest buffered frame; used to decide
  /// whether a match is due without paying for a merge.
  bool latest(double* t, BASIC::SE3* T_O_B) const;

  void clear() { frames_.clear(); }
  std::size_t size() const { return frames_.size(); }

  /// How many times build() has actually run. Worth watching: build() is the
  /// per-frame cost the backend can avoid, and it should track the number of
  /// matches, not the number of frames received.
  std::size_t buildCount() const { return build_count_; }

private:
  ScanParams params_;
  std::deque<FrontendFrame> frames_;
  mutable std::size_t build_count_ = 0;
};

}  // namespace loc
}  // namespace LI2Sup

#endif  // LOC_SCAN_ACCUMULATOR_H_
