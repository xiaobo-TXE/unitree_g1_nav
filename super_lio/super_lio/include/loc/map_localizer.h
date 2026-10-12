/**
 * @file map_localizer.h
 * @brief The localization backend: owns thread T1, the state machine, the
 *        submap, the registrars and the drift estimator.
 *
 * This is the module's external seam. The frontend sees five calls —
 * pushFrame, drift, requestInit, status, and start/stop — and nothing else.
 * Everything behind it (retrieval, stitching, accumulation, alignment, gating,
 * filtering, the initialization search) is implementation and can be reworked
 * without touching the frontend.
 *
 * Threading rules, which are the whole safety argument for bolting this onto a
 * single-threaded LIO:
 *  - T1 NEVER touches frontend state: not kf_, not ivox_, not measures_, not
 *    any SuperLIO member. Its only inputs are the frames pushed to it.
 *  - T0 calls only pushFrame() (a deep copy into a bounded queue) and drift()
 *    (a shared-lock read of seven floats). Neither can block on T1's work.
 *  - Registration runs in the registrar's own tbb::task_arena, so it cannot
 *    take every core away from the frontend's parallel_for.
 */
#ifndef LOC_MAP_LOCALIZER_H_
#define LOC_MAP_LOCALIZER_H_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include "loc/drift_estimator.h"
#include "loc/frontend_watch.h"
#include "loc/keyframe_map.h"
#include "loc/loc_params.h"
#include "loc/loc_stats.h"
#include "loc/match_log.h"
#include "loc/pose_composer.h"
#include "loc/registrar.h"
#include "loc/scan_accumulator.h"
#include "loc/submap_manager.h"

namespace LI2Sup {
namespace loc {

enum class LocState : std::uint8_t {
  kWaitOdom  = 0,
  kWaitInit  = 1,
  kInitializing = 2,
  kTracking  = 3,
  kLost      = 4,
  /// The frontend itself is unusable -- not "cannot find myself on the map"
  /// (that is kLost) but "T_O_B is not a pose the robot can be in". Output
  /// stops entirely while in this state; see frontend_watch.h.
  kFailed    = 5
};

const char* toString(LocState s);

/// Snapshot of what the backend is doing. Plain data, no ROS: the node turns it
/// into super_lio/msg/LocStatus.
struct LocStatus {
  LocState state = LocState::kWaitOdom;
  bool     valid = false;        ///< drift is usable for output
  double   stamp = 0.0;          ///< time of the last processed scan

  BASIC::SE3 T_M_O;

  double inlier_ratio  = 0.0;
  double rmse          = 0.0;
  double correction_t  = 0.0;    ///< body-frame correction of the last match [m]
  double correction_r  = 0.0;    ///< [rad]
  double last_match_age = 0.0;   ///< [s] since the last ACCEPTED match
  double match_time_ms = 0.0;
  double submap_build_ms = 0.0;
  int    consecutive_rejects = 0;
  /// Increments on every vetted discontinuity (initialization confirmed, LOST
  /// recovery confirmed). The output side watches it and snaps instead of
  /// ramping, rather than depending on the correction happening to exceed
  /// snap_t -- a relocalization a few centimetres away is still a
  /// discontinuity in provenance and should not be smeared over 5 seconds.
  std::uint32_t jump_seq = 0;
  std::string last_reject = "none";
  std::vector<int> submap_ids;
  std::size_t queue_dropped = 0;
  std::size_t frames_in = 0;      ///< frames handed to the backend
  std::size_t scan_builds = 0;    ///< times the scan merge actually ran
};

/// A request to (re)initialize, handed over from the ROS layer.
struct InitRequest {
  BASIC::SE3 T_M_B;        ///< guess for the body pose in the map frame
  bool yaw_known = true;   ///< false widens the yaw search to a full turn
  bool use_map_z = true;   ///< take z from the nearest keyframe instead of the guess
};

class MapLocalizer {
public:
  MapLocalizer(KeyframeMap::Ptr map, const LocParams& params);
  ~MapLocalizer();

  MapLocalizer(const MapLocalizer&) = delete;
  MapLocalizer& operator=(const MapLocalizer&) = delete;

  void start();
  void stop();

  // ---- called from T0 -----------------------------------------------------

  /// Hand one frontend frame to the backend. Never blocks; drops the oldest
  /// queued frame if the backend has fallen behind.
  void pushFrame(FrontendFrame frame);

  /// Current drift estimate. False until the backend has initialized, in which
  /// case *T_M_O is untouched.
  bool drift(BASIC::SE3* T_M_O) const;

  /// Ask for a (re)initialization at `req`. Takes effect on the next scan.
  /// A manual request always wins, whatever the current state — a human knows
  /// where the robot is better than a filter that thinks it is fine.
  void requestInit(const InitRequest& req);

  LocStatus status() const;

  /// Hands over the stitched submap the last rebuild produced, for display.
  /// Returns false when nothing new has been built since the last call.
  bool takeSubmapForDisplay(CloudXYZ::Ptr* cloud, std::uint64_t* version);

private:
  void run();

  /// One tracking match: build the source, refresh the submap, align, gate, fuse.
  void trackOnce(const SourceScan& scan);

  /// Multi-hypothesis (x, y, yaw) search used by both initialization and the
  /// LOST recovery. Returns true and fills *best when a hypothesis passes.
  bool searchPose(const SourceScan& scan, const BASIC::SE3& guess_M_B,
                  double yaw_range, double yaw_step, double xy_range,
                  float submap_radius, int submap_max_k,
                  RegResult* best);

  /// Rebuild a guess so that only (x, y, yaw) come from `raw`: z is taken from
  /// the map and roll/pitch from `odom_ref`. Applied to every search attempt,
  /// which is what stops a repeatedly-failing search from propagating itself
  /// off the map.
  BASIC::SE3 groundGuess(const BASIC::SE3& raw, const BASIC::SE3& odom_ref,
                         const InitRequest& req) const;

  /// Screen a batch of frontend frames before anything consumes them. Returns
  /// false when the frontend has been declared unusable and the batch must be
  /// dropped -- a non-finite pose would otherwise poison the merged source
  /// cloud, and a flying one would be published.
  bool frontendUsable(const std::vector<FrontendFrame>& batch);

  void handleInitializing(const SourceScan& scan);
  void handleTracking(const SourceScan& scan);
  void handleLost(const SourceScan& scan);

  /// Is a tracking match due? Answered from the newest frame's pose and time
  /// alone, so the caller can skip the scan merge entirely when it is not.
  bool matchDue(double t, const BASIC::SE3& T_O_B) const;
  /// Same question for an INITIALIZING / LOST search attempt, which is far more
  /// expensive and so is paced separately.
  bool attemptDue(double t, const BASIC::SE3& T_O_B) const;
  void noteAttempt(double t, const BASIC::SE3& T_O_B);
  void setState(LocState s);
  void publishStatus(const SourceScan& scan, const RegResult& reg,
                     const UpdateReport& rep);

  /// Decide whether one match is worth a line, and print it with enough context
  /// to act on: which scan, which submap, where we thought we were, what the
  /// registrar said and which gate answered. Rejections always print; an
  /// accepted match prints only when it crossed a log.warn_* threshold, or
  /// under log.debug.
  void logMatch(const SourceScan& scan, const RegResult& reg, const UpdateReport& rep,
                const Submap& sm, const BASIC::SE3& T_pred);

  /// Seconds of wall time since start().
  double wallSeconds() const;
  /// Fill in what RunStats cannot reach on its own.
  RunStats::Context statsContext() const;
  /// Emit the periodic one-liner if it is due.
  void maybeHeartbeat();
  /// Record a submap rebuild if update() produced a new version.
  void noteSubmap(const Submap& sm, std::uint64_t version_before);

  KeyframeMap::Ptr map_;
  LocParams params_;

  FrameQueue queue_;
  ScanAccumulator accumulator_;
  SubmapManager submap_;
  std::unique_ptr<IRegistrar> track_reg_;
  std::unique_ptr<IRegistrar> coarse_reg_;
  DriftEstimator estimator_;
  FrontendWatch watch_;
  MatchLogger match_log_;

  std::thread thread_;
  std::atomic<bool> running_{false};

  /// T_M_O, written by T1 and read by T0.
  mutable std::shared_mutex drift_mutex_;
  BASIC::SE3 drift_;
  std::atomic<bool> drift_valid_{false};

  mutable std::mutex status_mutex_;
  LocStatus status_;

  mutable std::mutex init_mutex_;
  InitRequest init_request_;
  bool init_pending_ = false;
  int  init_attempts_ = 0;
  /// Odometry pose at the instant the request was made. A guess names where the
  /// body was THEN; pairing it with a later odometry pose would bake in however
  /// far the robot walked in between (measured: 11 m, after a kidnap).
  BASIC::SE3 init_request_odom_;
  bool init_request_has_odom_ = false;

  /// Newest odometry pose handed to pushFrame(), so requestInit() can pair a
  /// guess with the odometry of its own moment. Written on T0 by pushFrame and
  /// read on T0 by requestInit; the mutex covers the T1 consumer.
  mutable std::mutex latest_odom_mutex_;
  BASIC::SE3 latest_odom_;
  bool latest_odom_valid_ = false;

  /// Previous initialization candidate, as a DRIFT, for the two-in-a-row
  /// consistency check that keeps a symmetric corridor from being accepted on
  /// one lucky match. Drifts are comparable across scans taken at different
  /// times; body poses are not.
  bool  have_init_candidate_ = false;
  BASIC::SE3 init_candidate_;

  /// The initial guess, also held as a drift so that retries track the robot
  /// instead of searching around where it stood when the guess arrived.
  bool  have_init_drift_guess_ = false;
  BASIC::SE3 init_drift_guess_;

  /// Same two-scan confirmation for LOST recovery. Re-entering TRACKING at the
  /// wrong place is worse than staying LOST.
  bool  have_reloc_candidate_ = false;
  BASIC::SE3 reloc_candidate_;

  mutable std::mutex display_mutex_;
  CloudXYZ::Ptr display_cloud_;
  std::uint64_t display_version_ = 0;
  std::uint64_t display_taken_ = 0;

  /// Self-observation. The board is reached through a pasted log, so what is
  /// not counted here cannot be checked there.
  RunStats stats_;
  std::chrono::steady_clock::time_point start_wall_{};
  double last_heartbeat_s_ = 0.0;
  std::size_t anomalies_ = 0;
  /// Wall time of the pending init request, so a confirmed fix can report how
  /// long it actually took to lock on.
  double init_request_wall_ = 0.0;

  std::uint32_t jump_seq_ = 0;
  std::size_t frames_in_ = 0;
  /// Latest frontend gravity estimate seen on the queue; heartbeat diagnostic
  /// only. See FrontendFrame::gravity_O.
  BASIC::V3   gravity_O_ = BASIC::V3(0, 0, -9.8);
  /// Latest frontend IMU-input health seen on the queue; log only.
  /// See FrontendFrame::ImuHealth.
  FrontendFrame::ImuHealth imu_health_;
  LocState state_ = LocState::kWaitOdom;
  BASIC::V3 last_match_pos_ = BASIC::V3::Zero();
  /// Frontend attitude at the last match, for the tilt term of the process
  /// noise. Stored as the body z axis in O, which is what makes the measure
  /// yaw-invariant.
  BASIC::V3 last_match_up_ = BASIC::V3(0, 0, 1);
  bool   have_last_match_ = false;
  double last_match_time_ = -1.0;

  /// Pacing for the search attempts of INITIALIZING and LOST, kept separate
  /// from the tracking-match clock: a failed search must not reset the
  /// tracking cadence, and vice versa.
  BASIC::V3 last_attempt_pos_ = BASIC::V3::Zero();
  bool   have_last_attempt_ = false;
  double last_attempt_time_ = -1.0;
};

}  // namespace loc
}  // namespace LI2Sup

#endif  // LOC_MAP_LOCALIZER_H_
