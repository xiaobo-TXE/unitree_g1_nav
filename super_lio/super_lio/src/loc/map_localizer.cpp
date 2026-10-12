#include "loc/map_localizer.h"

#include <cmath>
#include <algorithm>
#include <iomanip>
#include <sstream>

#include <pcl/common/transforms.h>

#if defined(__linux__)
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

#include "basic/logs.h"
#include "loc/se3_math.h"

namespace LI2Sup {
namespace loc {

const char* toString(LocState s) {
  switch (s) {
    case LocState::kWaitOdom:     return "WAIT_ODOM";
    case LocState::kWaitInit:     return "WAIT_INIT";
    case LocState::kInitializing: return "INITIALIZING";
    case LocState::kTracking:     return "TRACKING";
    case LocState::kLost:         return "LOST";
    case LocState::kFailed:       return "FAILED";
  }
  return "UNKNOWN";
}

MapLocalizer::MapLocalizer(KeyframeMap::Ptr map, const LocParams& params)
    : map_(std::move(map)),
      params_(params),
      queue_(static_cast<std::size_t>(std::max(1, params.scan.queue_capacity))),
      accumulator_(params.scan),
      submap_(map_, params.submap, params.log),
      track_reg_(IRegistrar::Create(params.track)),
      coarse_reg_(IRegistrar::Create(params.coarse)),
      estimator_(params.est),
      watch_(params.watch),
      match_log_(params.output.match_log) {
  stats_.setExpectedFrameHz(params.log.expected_frame_hz);
  // The odometry origin is wherever the robot started, which is on the map, so
  // while the robot IS on the map its distance from that origin cannot exceed
  // the map's own extent. Anything past that is not a place the robot can be.
  if (map_ && map_->size() > 0 && params_.watch.enable) {
    const double diag =
        static_cast<double>((map_->boundsMax() - map_->boundsMin()).norm());
    watch_.setOdomRadius(diag + params_.watch.odom_radius_margin);
  }
}

MapLocalizer::~MapLocalizer() { stop(); }

void MapLocalizer::start() {
  if (running_.exchange(true)) return;
  start_wall_ = std::chrono::steady_clock::now();
  last_heartbeat_s_ = 0.0;
  thread_ = std::thread(&MapLocalizer::run, this);
}

void MapLocalizer::stop() {
  if (!running_.exchange(false)) return;
  queue_.stop();
  if (thread_.joinable()) thread_.join();
  // Printed after the join so nothing is still being counted, and before the
  // frontend's own timing table. On the board this block is the run report.
  if (params_.log.summary_on_exit) {
    LOG(INFO) << GREEN << stats_.summary(statsContext()) << RESET;
  }
}

double MapLocalizer::wallSeconds() const {
  if (start_wall_.time_since_epoch().count() == 0) return 0.0;
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_wall_).count();
}

RunStats::Context MapLocalizer::statsContext() const {
  RunStats::Context c;
  c.wall_s    = wallSeconds();
  c.frames_in = frames_in_;
  c.gravity_O = gravity_O_;
  c.imu_refused   = imu_health_.refused;
  c.imu_max_gap_s = imu_health_.max_gap_s;
  c.imu_period_s  = imu_health_.period_s;
  c.scan_builds = accumulator_.buildCount();
  c.queue_depth = queue_.size();
  c.registrar = track_reg_ ? track_reg_->name() : "none";
  {
    // Both under the status mutex: stop() reads this from the caller's thread
    // after the join, and state_ itself belongs to T1.
    std::lock_guard<std::mutex> lk(status_mutex_);
    c.stamp = status_.stamp;
    c.state = toString(status_.state);
  }
  {
    std::shared_lock<std::shared_mutex> lk(drift_mutex_);
    c.drift = drift_;
  }
  if (map_) {
    c.keyframes = map_->size();
    const auto cs = map_->cacheStats();
    c.cache_hits = cs.hits;
    c.cache_misses = cs.misses;
    c.prefetched = cs.prefetched;
    c.disk_read_ms = cs.disk_read_ms;
    c.disk_read_max_ms = cs.disk_read_max_ms;
  }
  return c;
}

void MapLocalizer::maybeHeartbeat() {
  if (params_.log.heartbeat_s <= 0.0) return;
  const double now = wallSeconds();
  if (now - last_heartbeat_s_ < params_.log.heartbeat_s) return;
  last_heartbeat_s_ = now;
  const std::string line = stats_.heartbeat(statsContext());
  if (!line.empty()) LOG(INFO) << line;
}

void MapLocalizer::noteSubmap(const Submap& sm, std::uint64_t version_before) {
  if (!sm.valid()) {
    stats_.noteSubmapInvalid();
    return;
  }
  if (sm.version != version_before) {
    stats_.noteSubmapRebuild(sm.build_ms, sm.ids.size(), sm.cloud_M->size());
  }
}

void MapLocalizer::pushFrame(FrontendFrame frame) {
  // Timed because "T0 never blocks" is the safety argument for bolting this
  // onto a single-threaded LIO, and on the board it is a claim, not a
  // measurement, until this number comes back in the summary.
  const auto t0 = std::chrono::steady_clock::now();
  // Scrub the frontend's accumulated non-orthogonality at the boundary. See
  // orthonormalized() for why: SE3::inverse() transposes, so a rotation block
  // carrying a scale factor corrupts every conjugation downstream.
  frame.T_O_B = orthonormalized(frame.T_O_B);
  {
    std::lock_guard<std::mutex> lk(latest_odom_mutex_);
    latest_odom_ = frame.T_O_B;
    latest_odom_valid_ = true;
  }
  if (!queue_.push(std::move(frame))) {
    stats_.noteDrop();
    LOG_EVERY_N(WARNING, 50) << YELLOW << " ---> [MapLoc] backend behind, dropped a frame ("
                             << queue_.dropped() << " total)" << RESET;
  }
  stats_.notePush(
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
}

bool MapLocalizer::drift(BASIC::SE3* T_M_O) const {
  if (!drift_valid_.load(std::memory_order_acquire)) return false;
  std::shared_lock<std::shared_mutex> lk(drift_mutex_);
  if (T_M_O) *T_M_O = drift_;
  return true;
}

void MapLocalizer::requestInit(const InitRequest& req) {
  BASIC::SE3 odom_now;
  bool have_odom = false;
  {
    std::lock_guard<std::mutex> lk(latest_odom_mutex_);
    odom_now = latest_odom_;
    have_odom = latest_odom_valid_;
  }

  std::lock_guard<std::mutex> lk(init_mutex_);
  init_request_ = req;
  init_request_odom_ = odom_now;
  init_request_has_odom_ = have_odom;
  init_pending_ = true;
  init_attempts_ = 0;
  init_request_wall_ = wallSeconds();
  have_init_candidate_ = false;
  LOG(INFO) << " ---> [MapLoc] initialization requested at ("
            << req.T_M_B.t_.transpose() << "), yaw_known=" << req.yaw_known;
}

LocStatus MapLocalizer::status() const {
  std::lock_guard<std::mutex> lk(status_mutex_);
  return status_;
}

bool MapLocalizer::takeSubmapForDisplay(CloudXYZ::Ptr* cloud, std::uint64_t* version) {
  std::lock_guard<std::mutex> lk(display_mutex_);
  if (!display_cloud_ || display_version_ == display_taken_) return false;
  if (cloud) *cloud = display_cloud_;
  if (version) *version = display_version_;
  display_taken_ = display_version_;
  return true;
}

bool MapLocalizer::frontendUsable(const std::vector<FrontendFrame>& batch) {
  if (!params_.watch.enable) return true;

  // Every frame, not just the newest: frames arrive in batches, and the one
  // that proves the frontend left may not be the last one in the batch.
  for (const auto& f : batch) {
    const auto v = watch_.observe(f.t, f.T_O_B);
    if (!v.changed) continue;
    LOG(ERROR) << RED << " ---> [MapLoc] FRONTEND FAILED at t=" << std::fixed
               << std::setprecision(3) << f.t << ": "
               << FrontendWatch::toString(v.cause) << " " << v.value
               << " over limit " << v.limit
               << " | odom (" << f.T_O_B.t_.transpose() << ")"
               << " -- localization output STOPS until the frontend is sane again"
                  " and a new fix is confirmed"
               << RESET;
    stats_.noteFrontendFailed();
  }

  if (!watch_.failed()) return true;

  if (state_ != LocState::kFailed) {
    // Everything downstream of T_O_B is now unusable, including the drift we
    // already hold: it is only meaningful multiplied by a pose we no longer
    // believe. Dropping validity is what actually stops /loc/odom, the
    // map->world transform and the IMU-rate path -- before this, `drift_valid_`
    // was set at the first fix and never cleared, so the 2026-09-21 run that
    // reached 8 km published confidently the whole way.
    drift_valid_.store(false, std::memory_order_release);
    setState(LocState::kFailed);
  }

  if (watch_.recovered()) {
    LOG(WARNING) << YELLOW
        << " ---> [MapLoc] frontend plausible again after " << watch_.healthyStreak()
        << " frames; requiring a fresh fix rather than resuming -- a pose that"
           " flew and settled is still in the wrong place" << RESET;
    watch_.reset();
    accumulator_.clear();
    setState(LocState::kWaitInit);
  }
  return false;
}

void MapLocalizer::setState(LocState s) {
  if (state_ == s) return;
  LOG(INFO) << " ---> [MapLoc] state " << toString(state_) << " -> " << toString(s);
  state_ = s;
  std::lock_guard<std::mutex> lk(status_mutex_);
  status_.state = s;
  status_.jump_seq = jump_seq_;
}

void MapLocalizer::run() {
#if defined(__linux__)
  // Yield to the frontend: a late correction is harmless, a late odometry frame
  // is not.
  if (setpriority(PRIO_PROCESS, static_cast<id_t>(syscall(SYS_gettid)), 5) != 0) {
    LOG(WARNING) << " ---> [MapLoc] could not lower backend thread priority";
  }
#endif
  // Everything a board bring-up needs to confirm about the environment, in one
  // line: which registrar was actually compiled in, how many cores the tbb
  // arena is allowed, and how many the machine has. A board that silently ran
  // the PCL fallback, or an arena sized for the dev box, shows up here.
  LOG(INFO) << " ---> [MapLoc] backend thread up | registrar " << track_reg_->name()
            << " | arena " << params_.track.num_threads << " of "
            << std::thread::hardware_concurrency() << " cores"
            << " | accum " << params_.scan.accum_frames << " frames"
            << " | match every " << params_.trigger.match_min_dist << " m or "
            << params_.trigger.match_max_interval << " s"
            << " | lru " << params_.map.lru_capacity
            << (params_.map.preload_all ? " (preload_all)" : "")
            << " | debug " << (params_.log.debug ? "ON" : "off");

  std::vector<FrontendFrame> batch;
  while (running_.load()) {
    if (!queue_.waitAndPopAll(&batch)) break;
    frames_in_ += batch.size();
    for (const auto& f : batch) stats_.noteFrameStamp(f.t);
    if (!batch.empty()) {
      gravity_O_ = batch.back().gravity_O;
      imu_health_ = batch.back().imu;
    }

    // Before anything consumes the frames. A pose the robot cannot be in must
    // not reach the accumulator, the estimator, or an output topic.
    if (!frontendUsable(batch)) {
      maybeHeartbeat();
      continue;
    }
    accumulator_.add(batch);
    if (accumulator_.size() == 0) continue;

    {
      std::lock_guard<std::mutex> lk(init_mutex_);
      if (init_pending_) {
        init_pending_ = false;
        init_attempts_ = 0;
        have_init_candidate_ = false;
        have_init_drift_guess_ = false;
        // A manual request always wins, including while TRACKING.
        setState(LocState::kInitializing);
      }
    }

    if (state_ == LocState::kWaitOdom) setState(LocState::kWaitInit);
    if (state_ == LocState::kWaitInit) continue;  // nothing to do without a guess

    // Decide whether there is work BEFORE merging the scan. build() costs
    // 0.5 ms per frame here and an estimated 3-5 ms on the board with real
    // 8k-point sweeps, and every state below acts at most once per
    // match_min_dist / attempt interval, so merging on every 10 Hz frame was
    // pure waste. latest() answers the question from the newest frame's header
    // alone.
    double t_latest = 0.0;
    BASIC::SE3 T_O_B_latest;
    if (!accumulator_.latest(&t_latest, &T_O_B_latest)) continue;

    const bool due = (state_ == LocState::kTracking)
                         ? matchDue(t_latest, T_O_B_latest)
                         : attemptDue(t_latest, T_O_B_latest);
    if (!due) {
      // Tick here too: a robot standing still in WAIT_INIT must not look
      // identical to a dead node.
      maybeHeartbeat();
      continue;
    }

    const SourceScan scan = accumulator_.build();
    if (!scan.valid()) continue;
    stats_.noteScanBuild(scan.build_ms);

    switch (state_) {
      case LocState::kInitializing: handleInitializing(scan); break;
      case LocState::kTracking:     handleTracking(scan);     break;
      case LocState::kLost:         handleLost(scan);         break;
      default: break;
    }
    maybeHeartbeat();
  }
  LOG(INFO) << " ---> [MapLoc] backend thread down";
}

bool MapLocalizer::matchDue(double t, const BASIC::SE3& T_O_B) const {
  if (!have_last_match_ || last_match_time_ < 0.0) return true;
  const double moved = (T_O_B.t_ - last_match_pos_).norm();
  if (moved >= params_.trigger.match_min_dist) return true;
  return (t - last_match_time_) >= params_.trigger.match_max_interval;
}

bool MapLocalizer::attemptDue(double t, const BASIC::SE3& T_O_B) const {
  if (!have_last_attempt_ || last_attempt_time_ < 0.0) return true;
  // Try hard first, then settle into the paced cadence.
  if (state_ == LocState::kInitializing && init_attempts_ < params_.init.eager_attempts) {
    return true;
  }
  const double min_dist = (state_ == LocState::kLost)
                              ? params_.lost.attempt_min_dist
                              : params_.init.attempt_min_dist;
  const double min_interval = (state_ == LocState::kLost)
                                  ? params_.lost.attempt_min_interval
                                  : params_.init.attempt_min_interval;
  const double moved = (T_O_B.t_ - last_attempt_pos_).norm();
  if (moved >= min_dist) return true;
  return (t - last_attempt_time_) >= min_interval;
}

void MapLocalizer::noteAttempt(double t, const BASIC::SE3& T_O_B) {
  last_attempt_pos_ = T_O_B.t_;
  last_attempt_time_ = t;
  have_last_attempt_ = true;
}

void MapLocalizer::handleTracking(const SourceScan& scan) {
  // run() has already established that a match is due.
  trackOnce(scan);

  const bool too_many_rejects =
      estimator_.consecutiveRejects() >= params_.lost.reject_count;
  const bool too_long =
      estimator_.ageOfLastAccept(scan.t) > params_.lost.timeout;
  if (too_many_rejects || too_long) {
    stats_.noteLost();
    LOG(WARNING) << RED << " ---> [MapLoc] lost: " << estimator_.consecutiveRejects()
                 << " consecutive rejects, " << estimator_.ageOfLastAccept(scan.t)
                 << " s since the last accepted match" << RESET;
    setState(LocState::kLost);
  }
}

void MapLocalizer::trackOnce(const SourceScan& scan) {
  // Predict where the body is, using the drift we already believe in. The
  // submap follows the predicted MAP position, not the odometry position.
  BASIC::SE3 T_M_O_now;
  drift(&T_M_O_now);
  const BASIC::SE3 T_pred = T_M_O_now * scan.T_O_B;
  const Eigen::Vector3f p_pred = T_pred.t_.template cast<float>();

  const std::uint64_t submap_before = submap_.current().version;
  const Submap& sm = submap_.update(p_pred);
  noteSubmap(sm, submap_before);
  if (!sm.valid()) {
    // No map around the predicted position. Record the attempt anyway, or the
    // next frame passes matchDue again and re-merges the scan for nothing.
    LOG_EVERY_N(WARNING, 20)
        << " ---> [MapLoc] no submap at the predicted position ("
        << p_pred.transpose() << "); off the map?";
    last_match_pos_  = scan.T_O_B.t_;
    last_match_up_   = scan.T_O_B.R_.col(2);
    last_match_time_ = scan.t;
    have_last_match_ = true;
    return;
  }

  track_reg_->setTarget(sm.cloud_M, sm.version);
  {
    std::lock_guard<std::mutex> lk(display_mutex_);
    display_cloud_ = sm.cloud_M;
    display_version_ = sm.version;
  }

  const RegResult reg = track_reg_->align(*scan.cloud_B, T_pred);

  const double gap_s = have_last_match_ ? scan.t - last_match_time_ : 0.0;

  // Process noise grows with the distance travelled AND the time elapsed since
  // the last match; a stationary robot covers no distance but still drifts.
  if (have_last_match_) {
    // How far the frontend's attitude tilted since the last match. Yaw-free by
    // construction: it is the angle between the body up-axis then and now, so
    // turning on flat ground contributes nothing and a staircase contributes a
    // lot. See EstimatorParams::sigma_r_per_tilt.
    const double cos_tilt =
        std::clamp<double>(scan.T_O_B.R_.col(2).dot(last_match_up_), -1.0, 1.0);
    estimator_.predict((scan.T_O_B.t_ - last_match_pos_).norm(),
                       std::max(0.0, scan.t - last_match_time_),
                       std::acos(cos_tilt));
  }
  const UpdateReport rep = estimator_.update(reg, scan.T_O_B, scan.t);

  if (rep.accepted) {
    {
      std::unique_lock<std::shared_mutex> lk(drift_mutex_);
      drift_ = estimator_.drift();
    }
    drift_valid_.store(true, std::memory_order_release);
  }

  stats_.noteMatch(rep.accepted, rep.reason, reg, rep, gap_s);
  logMatch(scan, reg, rep, sm, T_pred);

  last_match_pos_ = scan.T_O_B.t_;
  last_match_up_  = scan.T_O_B.R_.col(2);
  last_match_time_ = scan.t;
  have_last_match_ = true;

  // Every tracking match, accepted or not. reg.T here is the registrar's raw
  // answer, which is the one quantity no topic carries and the one D1 needs.
  match_log_.write(scan.t, "TRACKING", rep.accepted, toString(rep.reason), reg, rep,
                   scan.T_O_B, sm.version, sm.ids);
  publishStatus(scan, reg, rep);
}

BASIC::SE3 MapLocalizer::groundGuess(const BASIC::SE3& raw,
                                     const BASIC::SE3& odom_ref,
                                     const InitRequest& req) const {
  // Only (x, y, yaw) of `raw` is trusted. The other three degrees of freedom are
  // re-derived from sources that cannot accumulate error:
  //  - z from the nearest keyframe, because map keyframes are this same IMU's
  //    height and beat both a human's guess and a propagated estimate;
  //  - roll and pitch from the odometry, because O and M are both gravity
  //    aligned, so the body tilt is numerically the same in either frame.
  BASIC::V3 t = raw.t_;
  if (req.use_map_z && params_.init.z_from_map) {
    // Nearest in the horizontal plane only. A 3D query uses the z we are trying
    // to establish, so a wrong z pulls in keyframes from another level and then
    // confirms itself.
    const int id = map_->nearestIdXY(t.template cast<float>());
    if (id >= 0) t.z() = map_->kf(id).T_M_B.t_.z();
  }

  const Eigen::Matrix3d R_ob = odom_ref.R_.template cast<double>();
  const double yaw_guess = yawOf(raw.R_.template cast<double>());
  const Eigen::Matrix3d R =
      Eigen::AngleAxisd(yaw_guess - yawOf(R_ob), Eigen::Vector3d::UnitZ()).toRotationMatrix() * R_ob;
  return BASIC::SE3(BASIC::M3(R.cast<BASIC::scalar>()), t);
}

void MapLocalizer::handleInitializing(const SourceScan& scan) {
  noteAttempt(scan.t, scan.T_O_B);

  InitRequest req;
  BASIC::SE3 odom_at_request;
  bool have_odom_at_request = false;
  {
    std::lock_guard<std::mutex> lk(init_mutex_);
    req = init_request_;
    odom_at_request = init_request_odom_;
    have_odom_at_request = init_request_has_odom_;
  }

  BASIC::SE3 guess = req.T_M_B;

  // The guess names a body pose at the instant it was given, but the robot
  // keeps walking while we retry. Convert it once into a DRIFT and carry that
  // instead: a drift is stationary, so replaying it against the current
  // odometry pose propagates the guess for free. Without this, every retry
  // searches around where the robot used to be.
  //
  // Crucially the conversion pairs the guess with the odometry pose from the
  // SAME instant, captured in requestInit(). Using the current scan's odometry
  // would bake in everything walked since the request -- after a kidnap the
  // request is only acted on ~100 frames later, and that error was 11 m.
  if (!have_init_drift_guess_) {
    const BASIC::SE3 odom_ref =
        have_odom_at_request ? odom_at_request : scan.T_O_B;
    init_drift_guess_ = groundGuess(guess, odom_ref, req) * odom_ref.inverse();
    have_init_drift_guess_ = true;
  }
  // Re-ground every attempt, do not just replay the anchor. Propagation carries
  // the odometry's own error, and a search that keeps failing keeps propagating:
  // measured, the guess wandered to z = -69 m on a map whose z range is
  // [-6.1, 0.6], at which point every alignment saw an empty submap. z from the
  // map and roll/pitch from the current scan are re-derived from sources that
  // cannot drift, so only x, y and yaw are ever carried forward.
  guess = groundGuess(init_drift_guess_ * scan.T_O_B, scan.T_O_B, req);

  const double yaw_range = req.yaw_known ? params_.init.yaw_range : M_PI;
  const double yaw_step  = req.yaw_known ? params_.init.yaw_step
                                         : 30.0 * M_PI / 180.0;

  RegResult best;
  const bool found = searchPose(scan, guess, yaw_range, yaw_step, params_.init.xy_step,
                                params_.init.submap_radius, params_.init.submap_max_k, &best);

  ++init_attempts_;
  if (!found) {
    LOG(WARNING) << " ---> [MapLoc] initialization attempt " << init_attempts_
                 << "/" << params_.init.max_attempts << " failed";
    have_init_candidate_ = false;
    if (init_attempts_ >= params_.init.max_attempts) {
      LOG(ERROR) << " ---> [MapLoc] giving up on this guess; waiting for a new one";
      setState(LocState::kWaitInit);
    }
    return;
  }

  // Two consecutive scans must agree before we believe it. A single good score
  // in a symmetric corridor is not evidence.
  //
  // The comparison is between DRIFTS, not body poses. The two candidates come
  // from scans taken at different times, so their poses legitimately differ by
  // however far the robot walked in between; only the drift they imply should
  // be the same. Comparing poses here would reject every honest initialization
  // taken while moving.
  const BASIC::SE3 drift_candidate = best.T * scan.T_O_B.inverse();

  if (!have_init_candidate_) {
    have_init_candidate_ = true;
    init_candidate_ = drift_candidate;
    LOG(INFO) << " ---> [MapLoc] initialization candidate at ("
              << best.T.t_.transpose() << "), inlier " << best.inlier_ratio
              << ", rmse " << best.rmse << "; waiting for confirmation";
    return;
  }

  const Vec6d diff = logSE3(init_candidate_.inverse() * drift_candidate);
  const double dt = diff.tail<3>().norm();
  const double dr = diff.head<3>().norm();
  if (dt > params_.init.consistency_t || dr > params_.init.consistency_r) {
    LOG(WARNING) << " ---> [MapLoc] candidates disagree (" << dt << " m, "
                 << dr * 180.0 / M_PI << " deg), restarting the search";
    init_candidate_ = drift_candidate;
    return;
  }

  // Confirmed. The first accepted drift is a snap by construction: P is seeded
  // from the measurement covariance, inflated so the first tracking updates can
  // still move it.
  const BASIC::SE3 T_M_O = drift_candidate;
  const Mat6d Ad = adjointSE3(scan.T_O_B);
  estimator_.reset(T_M_O, 4.0 * Ad * best.cov * Ad.transpose(), scan.t);
  ++jump_seq_;
  {
    std::unique_lock<std::shared_mutex> lk(drift_mutex_);
    drift_ = T_M_O;
  }
  drift_valid_.store(true, std::memory_order_release);

  submap_.invalidate();
  last_match_pos_ = scan.T_O_B.t_;
  last_match_up_  = scan.T_O_B.R_.col(2);
  last_match_time_ = scan.t;
  have_last_match_ = true;

  stats_.noteFix(false, wallSeconds() - init_request_wall_, init_attempts_);
  LOG(INFO) << GREEN << " ---> [MapLoc] INITIALIZED at (" << best.T.t_.transpose()
            << "), inlier " << best.inlier_ratio << ", rmse " << best.rmse << " m, after "
            << init_attempts_ << " attempt(s) / "
            << (wallSeconds() - init_request_wall_) << " s" << RESET;
  setState(LocState::kTracking);

  UpdateReport rep;
  rep.accepted = true;
  publishStatus(scan, best, rep);
}

void MapLocalizer::handleLost(const SourceScan& scan) {
  noteAttempt(scan.t, scan.T_O_B);

  // Search around where we think we are, with a wider net than tracking uses.
  BASIC::SE3 T_M_O_now;
  if (!drift(&T_M_O_now)) {
    setState(LocState::kWaitInit);
    return;
  }
  const BASIC::SE3 guess = T_M_O_now * scan.T_O_B;

  RegResult best;
  const bool found = searchPose(scan, guess, params_.lost.yaw_range,
                                params_.init.yaw_step, params_.lost.xy_range,
                                params_.init.submap_radius, params_.init.submap_max_k,
                                &best);
  last_match_pos_ = scan.T_O_B.t_;
  last_match_up_  = scan.T_O_B.R_.col(2);
  last_match_time_ = scan.t;
  have_last_match_ = true;

  if (!found) {
    LOG_EVERY_N(WARNING, 5) << " ---> [MapLoc] relocalization failed, still LOST";
    have_reloc_candidate_ = false;
    return;
  }

  // Confirm across two scans, exactly as initialization does, and for the same
  // reason: a single good score somewhere in a repetitive environment is not
  // evidence, and re-entering TRACKING at the wrong place is far worse than
  // staying LOST a little longer. The comparison is between DRIFTS, which are
  // invariant to the robot moving between the two scans.
  const BASIC::SE3 drift_candidate = best.T * scan.T_O_B.inverse();
  if (!have_reloc_candidate_) {
    have_reloc_candidate_ = true;
    reloc_candidate_ = drift_candidate;
    LOG(INFO) << " ---> [MapLoc] relocalization candidate at ("
              << best.T.t_.transpose() << "), inlier " << best.inlier_ratio
              << "; waiting for confirmation";
    return;
  }
  const Vec6d rdiff = logSE3(reloc_candidate_.inverse() * drift_candidate);
  if (rdiff.tail<3>().norm() > params_.init.consistency_t ||
      rdiff.head<3>().norm() > params_.init.consistency_r) {
    LOG(WARNING) << " ---> [MapLoc] relocalization candidates disagree ("
                 << rdiff.tail<3>().norm() << " m, "
                 << rdiff.head<3>().norm() * 180.0 / M_PI << " deg), still LOST";
    reloc_candidate_ = drift_candidate;
    return;
  }

  const BASIC::SE3 T_M_O = drift_candidate;
  const Mat6d Ad = adjointSE3(scan.T_O_B);
  estimator_.reset(T_M_O, 4.0 * Ad * best.cov * Ad.transpose(), scan.t);
  ++jump_seq_;
  have_reloc_candidate_ = false;
  {
    std::unique_lock<std::shared_mutex> lk(drift_mutex_);
    drift_ = T_M_O;
  }
  drift_valid_.store(true, std::memory_order_release);
  submap_.invalidate();

  stats_.noteFix(true, 0.0, 0);
  LOG(INFO) << GREEN << " ---> [MapLoc] recovered at (" << best.T.t_.transpose()
            << "), inlier " << best.inlier_ratio << ", rmse " << best.rmse << RESET;
  setState(LocState::kTracking);

  UpdateReport rep;
  rep.accepted = true;
  publishStatus(scan, best, rep);
}

bool MapLocalizer::searchPose(const SourceScan& scan, const BASIC::SE3& guess_M_B,
                              double yaw_range, double yaw_step, double xy_range,
                              float submap_radius, int submap_max_k,
                              RegResult* best) {
  const auto t_search0 = std::chrono::steady_clock::now();
  auto search_ms = [&t_search0]() {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                     t_search0).count();
  };

  const Eigen::Vector3f center = guess_M_B.t_.template cast<float>();
  const std::uint64_t submap_before = submap_.current().version;
  const Submap& sm = submap_.update(center, submap_radius, submap_max_k);
  noteSubmap(sm, submap_before);
  if (!sm.valid()) {
    LOG(WARNING) << YELLOW << " ---> [MapLoc] no submap around the initial guess ("
                 << center.transpose() << "); is the guess on the map at all?" << RESET;
    stats_.noteSearch(false, search_ms());
    return false;
  }
  coarse_reg_->setTarget(sm.cloud_M, sm.version);
  track_reg_->setTarget(sm.cloud_M, sm.version);
  {
    std::lock_guard<std::mutex> lk(display_mutex_);
    display_cloud_ = sm.cloud_M;
    display_version_ = sm.version;
  }

  // Only (x, y, yaw) is searched: z came from the map and roll/pitch from the
  // frontend, both better than anything a search over them would find.
  std::vector<BASIC::V3> offsets{BASIC::V3::Zero()};
  if (xy_range > 1e-6) {
    offsets.push_back(BASIC::V3(xy_range, 0, 0));
    offsets.push_back(BASIC::V3(-xy_range, 0, 0));
    offsets.push_back(BASIC::V3(0, xy_range, 0));
    offsets.push_back(BASIC::V3(0, -xy_range, 0));
  }

  const Eigen::Matrix3d R_guess = guess_M_B.R_.template cast<double>();
  const double yaw0 = yawOf(R_guess);

  std::vector<BASIC::SE3> hypotheses;
  const int n_yaw = yaw_step > 1e-6
                        ? static_cast<int>(std::floor(yaw_range / yaw_step))
                        : 0;
  for (int i = -n_yaw; i <= n_yaw; ++i) {
    const double yaw = yaw0 + i * yaw_step;
    const Eigen::Matrix3d R =
        Eigen::AngleAxisd(yaw - yaw0, Eigen::Vector3d::UnitZ()).toRotationMatrix() * R_guess;
    for (const auto& d : offsets) {
      hypotheses.emplace_back(BASIC::M3(R.cast<BASIC::scalar>()), BASIC::V3(guess_M_B.t_ + d));
    }
  }

  std::vector<RegResult> coarse_results;
  int coarse_unconverged = 0;
  for (const auto& hyp : hypotheses) {
    RegResult r = coarse_reg_->align(*scan.cloud_B, hyp);
    // Admission is hasPose(), NOT converged. The coarse pass only has to land
    // inside the fine pass's basin, and at max_iter it frequently reports
    // converged=false while holding a perfectly good pose. Requiring
    // convergence here is what made initialization fail at random and made
    // post-kidnap relocalization fail outright.
    if (!r.hasPose()) continue;
    if (!r.converged) ++coarse_unconverged;
    coarse_results.push_back(r);
  }

  // Rank by how much of the scan a hypothesis explains, with convergence only
  // as a tie-break. Then pay for a fine alignment on the best few.
  std::sort(coarse_results.begin(), coarse_results.end(),
            [](const RegResult& a, const RegResult& b) {
              if (a.inlier_ratio != b.inlier_ratio) return a.inlier_ratio > b.inlier_ratio;
              return a.converged && !b.converged;
            });

  // Seeds for the fine pass: the best coarse results first. If none of those
  // refine into something acceptable, fall back to refining the raw hypotheses
  // directly -- the coarse pass is useless in some places (measured: 0/25 good
  // at frame 2300, where the tracking registrar nonetheless recovers 1.0 m/10°
  // on its own), and that fallback is the difference between relocalizing and
  // giving up.
  double best_seen_inlier = 0.0, best_seen_rmse = 0.0;
  auto refineFrom = [&](const std::vector<BASIC::SE3>& seeds, RegResult* out) {
    bool have = false;
    for (const auto& seed : seeds) {
      RegResult fine = track_reg_->align(*scan.cloud_B, seed);
      if (!fine.hasPose()) continue;
      if (fine.inlier_ratio > best_seen_inlier) {
        best_seen_inlier = fine.inlier_ratio;
        best_seen_rmse = fine.rmse;
      }
      if (fine.inlier_ratio < params_.init.inlier_min || fine.rmse > params_.init.rmse_max) {
        continue;
      }
      if (!have || fine.rmse < out->rmse) {
        *out = fine;
        have = true;
      }
    }
    return have;
  };

  std::vector<BASIC::SE3> seeds;
  const int top_k = std::min<int>(params_.init.refine_top_k,
                                  static_cast<int>(coarse_results.size()));
  seeds.reserve(top_k);
  for (int i = 0; i < top_k; ++i) seeds.push_back(coarse_results[i].T);

  bool have_best = refineFrom(seeds, best);
  if (!have_best) {
    LOG(INFO) << " ---> [MapLoc] coarse seeds gave nothing acceptable ("
              << coarse_results.size() << " candidates, " << coarse_unconverged
              << " unconverged); refining the " << hypotheses.size()
              << " raw hypotheses directly";
    have_best = refineFrom(hypotheses, best);
  }

  {
    // A search row too, so the CSV covers every attempt and not just tracking.
    UpdateReport blank;
    RegResult logged = have_best ? *best : RegResult{};
    match_log_.write(scan.t, toString(state_), have_best,
                     have_best ? "accepted" : "search_failed", logged, blank,
                     scan.T_O_B, sm.version, sm.ids);
  }

  stats_.noteSearch(have_best, search_ms());

  if (!have_best) {
    LOG(WARNING) << YELLOW << " ---> [MapLoc] search failed [" << toString(state_) << "] odom("
              << scan.T_O_B.t_.transpose() << ") near (" << guess_M_B.t_.transpose()
              << "): best was inlier " << best_seen_inlier << " rmse " << best_seen_rmse
              << ", need inlier >= " << params_.init.inlier_min << " and rmse <= "
              << params_.init.rmse_max << "; submap v" << sm.version << " has "
              << sm.ids.size() << " keyframes at (" << sm.center.transpose() << "), "
              << coarse_results.size() << " coarse candidates (" << coarse_unconverged
              << " unconverged), " << hypotheses.size() << " hypotheses in "
              << search_ms() << " ms" << RESET;
    return false;
  }
  LOG(INFO) << " ---> [MapLoc] search accepted: inlier " << best->inlier_ratio
            << ", rmse " << best->rmse << " m, " << hypotheses.size() << " hypotheses in "
            << search_ms() << " ms (" << coarse_results.size()
            << " coarse candidates, " << coarse_unconverged << " unconverged)";
  return true;
}

void MapLocalizer::logMatch(const SourceScan& scan, const RegResult& reg,
                            const UpdateReport& rep, const Submap& sm,
                            const BASIC::SE3& T_pred) {
  const LogParams& L = params_.log;
  const double r2d = 180.0 / M_PI;

  // Name every threshold this match crossed rather than just saying "bad".
  // "rmse" sends you to the map or to dynamic obstacles; "slow" sends you to
  // the CPU budget; "corr_t" says the estimate was being yanked. They are
  // different investigations and the log should not merge them.
  std::string flags;
  auto flag = [&flags](const char* f) { flags += flags.empty() ? f : (std::string(" ") + f); };
  if (reg.rmse > L.warn_rmse)                      flag("rmse");
  if (reg.inlier_ratio < L.warn_inlier)            flag("low_inlier");
  if (rep.body_correction_t > L.warn_corr_t)       flag("corr_t");
  if (rep.body_correction_r > L.warn_corr_r)       flag("corr_r");
  if (reg.time_ms > L.warn_match_ms)               flag("slow");
  if (reg.degenerate_t_dims || reg.degenerate_r_dims) flag("degenerate");

  const bool anomalous = !flags.empty();
  // A rejection always prints: they are rare in a healthy run (measured 0.57%)
  // and six in a row is what declares LOST, so the six lines that explain it
  // must all be there. Accepted-but-suspicious matches are rate limited,
  // because a stretch of bad geometry can flag every single one.
  if (!rep.accepted) {
    ++anomalies_;
  } else if (anomalous) {
    ++anomalies_;
    const int n = std::max(1, L.warn_every_n);
    if (static_cast<int>(anomalies_ % static_cast<std::size_t>(n)) != 1 && n > 1) return;
  } else if (!L.debug) {
    return;
  }

  std::ostringstream ids;
  for (std::size_t i = 0; i < sm.ids.size() && i < 6; ++i) ids << sm.ids[i] << " ";
  if (sm.ids.size() > 6) ids << "... +" << (sm.ids.size() - 6);

  std::ostringstream o;
  o << " ---> [MapLoc] match t=" << std::fixed << std::setprecision(3) << scan.t
    << " [" << toString(state_) << "] "
    << (rep.accepted ? (anomalous ? std::string("ACCEPTED but ") + flags : std::string("ok"))
                     : std::string("REJECTED ") + toString(rep.reason) +
                           (anomalous ? " (" + flags + ")" : ""))
    << " | inlier " << std::setprecision(3) << reg.inlier_ratio
    << " rmse " << reg.rmse << " m"
    << " corr " << std::setprecision(1) << rep.body_correction_t * 1000.0 << " mm / "
    << std::setprecision(3) << rep.body_correction_r * r2d << " deg"
    << " chi2 " << std::setprecision(2) << rep.chi2
    << " | reg " << reg.time_ms << " ms it " << reg.iterations
    << " conv " << reg.converged << " deg_t " << reg.degenerate_t_dims
    << " deg_r " << reg.degenerate_r_dims
    << " src " << reg.num_source << " tgt " << reg.num_target
    << " gap " << (scan.t - last_match_time_) << " s"
    // Where we thought we were and where the odometry says we are: together
    // these say whether a bad match is a bad prediction or bad geometry.
    << " | pred (" << std::setprecision(2) << T_pred.t_.transpose() << ")"
    << " odom (" << scan.T_O_B.t_.transpose() << ")"
    << " | submap v" << sm.version << " c(" << sm.center.transpose() << ") "
    << sm.ids.size() << " kf [" << ids.str() << "]";

  if (rep.accepted && !anomalous) {
    LOG(INFO) << o.str();
  } else if (rep.accepted) {
    LOG(WARNING) << YELLOW << o.str() << RESET;
  } else {
    LOG(WARNING) << RED << o.str() << RESET;
  }
}

void MapLocalizer::publishStatus(const SourceScan& scan, const RegResult& reg,
                                 const UpdateReport& rep) {
  std::lock_guard<std::mutex> lk(status_mutex_);
  status_.state = state_;
  status_.valid = drift_valid_.load(std::memory_order_acquire);
  status_.stamp = scan.t;
  status_.T_M_O = estimator_.drift();
  status_.inlier_ratio = reg.inlier_ratio;
  status_.rmse = reg.rmse;
  status_.correction_t = rep.body_correction_t;
  status_.correction_r = rep.body_correction_r;
  status_.last_match_age = estimator_.ageOfLastAccept(scan.t);
  status_.match_time_ms = reg.time_ms;
  status_.submap_build_ms = submap_.current().build_ms;
  status_.consecutive_rejects = estimator_.consecutiveRejects();
  status_.jump_seq = jump_seq_;
  status_.last_reject = toString(rep.reason);
  status_.submap_ids = submap_.current().ids;
  status_.queue_dropped = queue_.dropped();
  status_.frames_in = frames_in_;
  status_.scan_builds = accumulator_.buildCount();
}

}  // namespace loc
}  // namespace LI2Sup
