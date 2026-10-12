/**
 * @file loc_params.h
 * @brief Parameters for the map-localization backend (MapLoc).
 *
 * Plain structs, no ROS and no globals: every loc module takes what it needs by
 * value or const-ref, so all of them can be unit tested without a node. The ROS
 * layer (see ROSWrapper::LoadLocParamFromRos) is the only place that knows about
 * the `lio.maploc.*` parameter names.
 *
 * See docs/map_localization_design.md section 11.2 for the parameter table.
 */
#ifndef LOC_PARAMS_H_
#define LOC_PARAMS_H_

#include <string>
#include <cmath>

namespace LI2Sup {
namespace loc {

/// Which registration backend to instantiate behind IRegistrar.
enum class RegistrationType {
  VGICP = 0,  ///< small_gicp GICP factor against a GaussianVoxelMap target
  GICP  = 1,  ///< small_gicp GICP factor against a KdTree target
  PCL_GICP = 2  ///< pcl::GeneralizedIterativeClosestPoint, reference/fallback only
};

/// Map storage and keyframe retrieval.
struct MapParams {
  std::string map_dir;             ///< loc_map/ produced by loc_map_builder (required)
  bool  preload_all   = false;     ///< load every keyframe cloud at startup
  int   lru_capacity  = 300;       ///< resident keyframe clouds when !preload_all
};

/// Submap assembly around the predicted body position.
struct SubmapParams {
  float radius        = 15.0f;     ///< keyframe search radius [m]
  int   max_k         = 20;        ///< at most this many keyframes per submap
  float refresh_dist  = 3.0f;      ///< rebuild once the centre moved this far [m]
  float prefetch_scale = 1.5f;     ///< prefetch radius = scale * radius
  /// Upper bound on cache-warming reads per rebuild. The prefetch happens
  /// synchronously inside the rebuild, so leaving it unbounded put up to
  /// 3 * max_k flash reads on the critical path -- fine on a dev box, an
  /// estimated 100 ms+ on the board. Rebuilds recur every refresh_dist metres,
  /// so a bounded prefetch still fills the cache, just over a few rebuilds.
  int   prefetch_max_files = 20;
};

/// Turning frontend frames into one registration source cloud.
struct ScanParams {
  int   accum_frames  = 5;         ///< frames merged into one source cloud
  float source_voxel  = 0.25f;     ///< source downsample leaf [m]
  int   queue_capacity = 10;       ///< FrameQueue depth; oldest dropped when full
};

/// One registration configuration. Two instances exist: tracking and coarse.
struct RegParams {
  RegistrationType type = RegistrationType::VGICP;
  float  target_voxel   = 0.5f;    ///< GaussianVoxelMap leaf [m]
  float  source_voxel   = 0.25f;   ///< source leaf [m], mirrors ScanParams for the coarse pass
  double max_corr_dist  = 1.0;     ///< correspondence rejection distance [m]
  int    max_iter       = 20;
  int    num_threads    = 2;       ///< size of the tbb::task_arena the align runs in
  int    cov_neighbors  = 10;      ///< kNN used for per-point covariance estimation

  /// Covariance floor, applied after sigma^2 * H^-1 so a good fit cannot become
  /// over-confident. Rotational floor is stored in radians.
  double cov_floor_t    = 0.03;    ///< [m]
  double cov_floor_r    = 0.3 * M_PI / 180.0;  ///< [rad]
  /// Degeneracy: an eigenvalue below ratio * lambda_max counts as unobservable.
  double degenerate_eig_ratio = 1e-3;
};

/// Gating and the SE(3) EKF over T_M_O.
struct EstimatorParams {
  /// Process noise on T_M_O, per metre of odometry travelled.
  ///
  /// These describe how far the drift wanders BETWEEN TWO MATCHES (~0.5 m), not
  /// its long-run average rate, and the two differ by an order of magnitude on
  /// a walking robot. Measured on the mapping bag by comparing /lio/odom
  /// against the PGO trajectory (see docs, section 19):
  ///
  ///   window   translation           rotation
  ///    0.5 m   median 0.18 m/m       median 0.0033 rad/m   <- the match interval
  ///    2.0 m   median 0.060 m/m      median 0.0011 rad/m
  ///   10.0 m   median 0.017 m/m      median 0.0003 rad/m   <- the long-run rate
  ///
  /// The original 0.02 m/m came from the long-run rate and was ~10x too small
  /// for one match interval, so P stayed tiny, the chi-square gate turned into
  /// a hair trigger, and perfectly good matches (inlier 0.86-0.98, rmse
  /// 0.12-0.24 m) were rejected six in a row until the filter declared LOST.
  /// The short-window figure is not measurement noise: timestamp association
  /// between the two trajectories is good to 3 ms median, worth ~4 mm at
  /// 1.2 m/s. It is real gait-induced wobble, exactly what design doc 16.9
  /// warns passes straight through T_O_B.
  /// ROUND 3 CORRECTION: the 0.5 m figure above is a statistical artefact.
  /// Round 2's D2 measured the same quantity properly -- comparing the odometry
  /// pose snapshotted WITH each match against the PGO trajectory over the
  /// actual match interval -- and got 0.0185 m/m median (p95 0.051), i.e. the
  /// same as the long-run rate, not 10x it. The 0.18 figure came from dividing
  /// timestamp-association noise by a short window. 0.25 left the chi-square
  /// gate wide open (round 2 measured zero rejections in a 39 minute run, which
  /// is a symptom, not health).
  double sigma_t_per_m = 0.05;     ///< [m per travelled m], from D2 p95
  double sigma_r_per_m = 0.002;    ///< [rad per travelled m]

  /// Process noise per second, applied on top of the distance term. A
  /// stationary robot still accumulates drift through IMU bias, and 9% of this
  /// dataset is stationary; without this, P stops growing whenever the robot
  /// stands still and the same over-confidence returns.
  double sigma_t_per_s = 0.02;     ///< [m per second]
  double sigma_r_per_s = 0.0005;   ///< [rad per second]

  /// Rotation process noise proportional to how much the FRONTEND's attitude
  /// actually moved since the last match. Dimensionless: rad of sigma per rad
  /// of body tilt change.
  ///
  /// The distance and time terms above describe drift on flat ground, which is
  /// where they were calibrated (D2). They do not describe what a legged robot
  /// does on a staircase: the attitude estimate is transiently stressed by the
  /// gait and the terrain, and the registration sees a correction the drift
  /// model has no room for. Measured on the board 2026-09-17, climbing the
  /// switchback stairs at (2.6, 7.2): seven matches refused by chi2 and the
  /// rate gate while reporting inlier 0.955 and rmse 0.099 m -- good geometry
  /// (the rotation Hessian is better conditioned there than in the open, 0.61
  /// vs 0.41) asking for a 2.1 deg turn that sigma_r_per_m = 0.002 rad/m priced
  /// at 0.115 deg per metre.
  ///
  /// Keyed on TILT change, not total rotation: yaw moves whenever the robot
  /// turns on flat ground, and that is no reason to distrust its attitude. Tilt
  /// is measured as the angle between the body z axis at the two matches, so it
  /// is yaw-invariant. On this map the separation is clean -- per 1 m of
  /// travel, median tilt change is 6.4 deg on the stairs and 1.0 deg on the
  /// flat.
  ///
  /// At 0.10 a flat-ground match adds 0.10 deg (total 0.12 -> 0.17 deg) while a
  /// stair match adds 0.64 deg, a ~7x loosening exactly where it is needed. The
  /// body gate (5 deg) and the rate gate are both P-independent and still cap
  /// how far a single accepted observation can pull the robot.
  double sigma_r_per_tilt = 0.10;  ///< [rad sigma per rad of tilt change]
  double track_inlier_min = 0.5;
  double track_rmse_max   = 0.3;   ///< [m]
  double gate_body_t      = 1.0;   ///< body-frame correction limit [m]
  double gate_body_r      = 5.0 * M_PI / 180.0;   ///< [rad]
  double chi2_gate        = 16.8;  ///< 6 dof, 99%
  bool   estimate_roll_pitch = true;

  /// Roll/pitch prior on T_M_O. It constrains how fast the tilt may CHANGE, not
  /// how large it may be: the prior pulls tilt toward an anchor, and the anchor
  /// follows the estimate at rp_anchor_rate.
  ///
  /// It used to pull toward zero, on the reasoning that both frames are gravity
  /// aligned so any tilt is error. Two board runs say otherwise. Fitting a
  /// single rigid T_M_O to a whole run leaves 0.35 m rms with tilt free and
  /// 1.97 m rms with tilt forced to zero (20260920; 0.92 m vs 4.28 m on
  /// 20260918), and the tilt it wants is 4.09 deg / 6.00 deg. The map is not
  /// the tilted one -- the heading-correlated component of map-up in the
  /// keyframe body frames is 0.17 deg -- so the frontend's gravity alignment
  /// drifts degrees over ~10 min of driving, and T_M_O has to hold that.
  ///
  /// Pulling a real 5 deg tilt toward zero after every accepted update (3300
  /// times in the 20260920 run) is a ratchet, not a weak prior: the tilt error
  /// leaks into translation (T_M_O.z reached -7.5 m) and every registration
  /// then reports the 1-2 deg of rotation the filter refuses to hold. That is
  /// what the rejections were -- 87% of them had inlier >= 0.85 and
  /// rmse <= 0.20, median corr_r 1.88 deg against 0.15 deg when accepted -- and
  /// six in a row is LOST, 12 times in that run.
  ///
  /// Anchored, the prior keeps the job it is actually needed for: tilt and
  /// translation trade off against each other in degenerate geometry, and
  /// holding tilt still over the short term is what stops a single match from
  /// making that trade.
  double rp_pseudo_sigma  = 2.0 * M_PI / 180.0;   ///< tilt vs anchor, 1 sigma [rad]

  /// How fast the anchor may follow the estimate [rad/s]. The drift this has to
  /// track is slow -- the board runs built 4 deg over ~9 min, 0.007 deg/s -- so
  /// 0.1 deg/s leaves an order of magnitude of margin while still holding tilt
  /// effectively fixed across the few seconds a bad match lives for.
  ///
  /// There is deliberately no ceiling on the anchor. One was tried and measured
  /// to be decorative: with healthy covariance a 1 deg cap moved a 4 deg
  /// observed tilt only to 3.82 deg. The registration quality gates are the
  /// real bound -- the anchor only goes where the alignments against a fixed
  /// map lead it, and a tilt large enough to matter wrecks inlier and rmse long
  /// before it accumulates.
  double rp_anchor_rate   = 0.1 * M_PI / 180.0;
  int    max_degenerate_t_dims = 1;  ///< drop the observation beyond this many
  /// Same for rotation. degenerate_r_dims was computed but never acted on; a
  /// slope with only ground plus one wall leaves rotation unobservable, and the
  /// resulting tilt error is exactly what the bad clusters look like.
  int    max_degenerate_r_dims = 0;

  /// Drift-rate gate: a second, P-INDEPENDENT consistency check.
  ///
  /// The chi-square gate scales with P, and P scales with the process noise, so
  /// raising the process noise (which round 1 had to do, 0.02 -> 0.25 m/m)
  /// widened the chi-square gate at the same time -- leaving almost nothing
  /// standing between a single badly-wrong match and the state. This gate
  /// compares consecutive accepted observations directly: the drift cannot
  /// physically move faster than some bound per metre travelled, whatever the
  /// filter currently believes about its own uncertainty.
  ///
  /// An observation that breaks the bound is HELD, not dropped: if the next one
  /// agrees with it, the jump was real and we take it; if not, the outlier is
  /// discarded. Cost is one match of latency on a genuine jump.
  /// The budget for how far two consecutive observations may disagree is
  /// AFFINE in the distance travelled, not a pure rate:
  ///
  ///   allowed = rate_gate_* * travelled + rate_gate_floor_*
  ///
  /// The rate term is the drift itself, which accumulates per metre. The floor
  /// is per-observation noise, which does not: two matches taken a centimetre
  /// apart still disagree by whatever the registration noise is. A pure rate
  /// has to divide by the travelled distance, and at a standstill that is zero,
  /// so the old form floored the DENOMINATOR at 0.2 m instead -- which says
  /// that a parked robot is allowed exactly as much observation noise as one
  /// that walked 0.2 m, and no more. Measured on the board (20260921-19, robot
  /// stationary, frozen odometry): three good matches -- inlier 0.92-0.95,
  /// rmse 0.155, the correction the robot would have felt 38-59 mm -- refused
  /// because 0.5-0.7 deg of rotation over the 0.2 m floor scored 2.4-3.7 deg/m
  /// against the 2.9 deg/m limit. Six in a row is LOST, standing still.
  ///
  /// At distance the two forms converge, so this does not reopen the gate where
  /// it was doing its job: the floor is a constant, the rate term still governs.
  /// The 0.5 m jump of RateGateStillCatchesAJumpFarFromTheOrigin is still held.
  ///
  /// The floors are sized from the registration noise the project already
  /// declares: TrackParams::cov_floor_t / cov_floor_r (0.03 m, 0.3 deg) are the
  /// stated "a single match is never more certain than this". A difference of
  /// two independent observations has sqrt(2) times that sigma, and a gate
  /// belongs at about 3 sigma of it -- 3 * 1.41 * 0.03 = 0.13 m, and the same
  /// for 0.3 deg gives 1.3 deg. They are plain defaults rather than derived at
  /// runtime because cov_floor_* lives in TrackParams; if you retune those,
  /// retune these with them.
  double rate_gate_t = 0.6;          ///< [m per travelled m]
  double rate_gate_r = 0.05;         ///< [rad per travelled m]
  double rate_gate_floor_t = 0.13;   ///< [m], per-observation noise allowance
  double rate_gate_floor_r = 1.3 * M_PI / 180.0;  ///< [rad], ditto
  bool   rate_gate_enable = true;

  /// Let the chi-square gate hold a refused observation and take it if the next
  /// match says the same thing, exactly as the rate gate does.
  ///
  /// Without it, a real discontinuity too small to trip the rate gate but too
  /// large for chi-square has no way in: it is refused until the process noise
  /// has grown P enough to admit it, and the output carries the whole error
  /// meanwhile. Measured on the mapping bag, a 365 mm frontend slip at 2.3 m/s
  /// scored 0.34 m/m against the 0.6 rate limit and took five refusals, 1.5 s.
  ///
  /// Switchable because it is the only path that can accept an observation the
  /// filter itself calls inconsistent; a run can be A/B'd against it.
  bool   chi2_confirm_enable = true;
  /// Covariance multiplier for an alignment that hit its iteration limit
  /// instead of converging. Such a pose is usually still good (measured), so it
  /// is down-weighted rather than discarded.
  double unconverged_cov_scale = 4.0;
};

/// Output rate limiting (PoseComposer).
struct ComposerParams {
  double v_max  = 0.2;             ///< body-frame correction speed [m/s]
  double w_max  = 5.0 * M_PI / 180.0;  ///< [rad/s]
  double tau    = 1.0;             ///< first-order time constant [s]
  double snap_t = 1.0;             ///< beyond this, snap instead of ramp [m]
  double snap_r = 10.0 * M_PI / 180.0;  ///< [rad]
};

/// Initialization / relocalization search (design doc section 6.5).
struct InitParams {
  double init_pose[6] = {0, 0, 0, 0, 0, 0};  ///< x y z roll pitch yaw, degrees for angles
  std::string source  = "yaml";    ///< "yaml" | "topic" | "gps"
  bool   yaw_known    = true;
  double yaw_range    = 45.0 * M_PI / 180.0;
  double yaw_step     = 15.0 * M_PI / 180.0;
  double xy_step      = 1.0;       ///< 0 disables the (dx,dy) offsets
  bool   z_from_map   = true;      ///< take z from the nearest keyframe
  float  submap_radius = 25.0f;
  int    submap_max_k  = 40;
  double inlier_min   = 0.6;
  double rmse_max     = 0.2;       ///< [m]
  int    max_attempts = 25;
  /// Minimum spacing between two search attempts. Without it, INITIALIZING runs
  /// a full multi-hypothesis search on every 10 Hz scan (~150 ms each on x86,
  /// ~1 s on the board) and burns through max_attempts in a second while a
  /// stationary robot has shown the backend nothing new.
  double attempt_min_dist = 0.5;   ///< [m] travelled since the last attempt
  double attempt_min_interval = 1.0;  ///< [s] since the last attempt
  /// The first few attempts after a request run back to back, unpaced. Pacing
  /// exists so a stationary robot with a hopeless guess does not burn a core
  /// forever, but the seconds right after an operator drops a pose are exactly
  /// when trying hard pays: a sparse, open location needs several attempts, and
  /// paying a full second between each put it over a 5 s budget.
  int    eager_attempts = 3;
  int    refine_top_k = 3;         ///< coarse hypotheses promoted to a fine align
  /// Two consecutive scans must agree before TRACKING is entered.
  double consistency_t = 0.3;      ///< [m]
  double consistency_r = 3.0 * M_PI / 180.0;
};

/// Loss detection and in-place relocalization.
struct LostParams {
  int    reject_count = 6;         ///< consecutive rejections before LOST
  double timeout      = 10.0;      ///< [s] without an accepted observation
  double yaw_range    = 30.0 * M_PI / 180.0;
  double xy_range     = 3.0;       ///< [m]
  /// Recovery is paced tighter than first-time initialization: the robot is
  /// already moving and every attempt costs real localization time. Two
  /// attempts are needed (the recovery is confirmed across two scans like
  /// initialization is), so this interval directly sets the recovery latency.
  double attempt_min_dist = 0.2;      ///< [m]
  double attempt_min_interval = 0.3;  ///< [s]
};

/// When to stop believing the frontend at all. See frontend_watch.h.
///
/// These are physical bounds, not tuning: crossing one means the frontend
/// reported something the robot cannot have done, and the consequence is that
/// localization output stops. Set them well clear of anything a correct run
/// approaches -- a false positive silences a working localizer, which is why
/// none of them is a "seems high" threshold.
struct FrontendWatchParams {
  bool   enable = true;

  /// [m/s] implied body speed. A quadruped tops out near 2 m/s; the board run
  /// that diverged left at ~14.5 m/s and kept accelerating, so anything in
  /// between separates the two cleanly.
  double max_speed = 5.0;
  /// Consecutive frames over max_speed. One frame over is a timestamp glitch
  /// or a dropped frame, not a divergence.
  int    speed_frames = 3;
  /// [s] Longest frame interval the speed check will measure across. A longer
  /// stall says something about the queue or the sensor, not about the pose,
  /// and dividing by it would manufacture a speed.
  double max_gap_s = 1.0;

  /// [m] Added to the loaded map's bounding-box diagonal to bound how far from
  /// the odometry origin the robot may be. The origin is on the map, so while
  /// the robot is on the map this distance cannot exceed the map's extent.
  /// The run that flew reached 8 km against a 260 m map.
  double odom_radius_margin = 50.0;

  /// Frames of plausible motion before the frontend is trusted again. At 10 Hz
  /// this is 2 s. Recovery goes to WAIT_INIT, never straight back to TRACKING:
  /// a pose that flew and settled is still in the wrong place, so the old drift
  /// is meaningless and a fresh fix is required.
  int    recover_frames = 20;
};

/// When to run a registration at all.
struct TriggerParams {
  double match_min_dist    = 0.5;  ///< [m] travelled since the last match
  double match_max_interval = 2.0; ///< [s] since the last match
};

/// What gets printed, and how loud. The board is reached through a pasted log,
/// so the defaults print everything needed to *verify* a run (startup banner,
/// state changes, anomalies, a periodic heartbeat, an end-of-run summary) and
/// nothing that merely narrates a healthy one.
///
/// `debug` is the master switch for the per-event firehose: one line per match,
/// one per submap rebuild, one per search hypothesis. Useful for the first few
/// board runs and for chasing a specific fault; off for everything else.
struct LogParams {
  bool   debug = false;             ///< per-match / per-rebuild detail
  double heartbeat_s = 5.0;         ///< periodic one-liner; 0 disables
  bool   summary_on_exit = true;

  /// A match crossing any of these is printed with full context -- scan time,
  /// submap version and keyframe ids, predicted position, gate outcome --
  /// whether or not it was accepted. Rejections always print; these catch the
  /// accepted-but-suspicious ones, which are the ones nothing else would show.
  /// [m]. Calibrated to sit between the observed live p95 and track_rmse_max
  /// (the line where a match is actually REFUSED), not against the bag figure.
  /// Replaying the mapping session aligns a scan against the very data it was
  /// built from and lands at rmse 0.09; a real run days later, with furniture
  /// moved and people walking, measured 0.14 p50 / 0.17 p95 on the same map.
  /// A threshold of 0.15 therefore cried wolf twice in the first 105 s of
  /// board running while nothing was wrong.
  double warn_rmse    = 0.25;
  double warn_inlier  = 0.75;
  double warn_corr_t  = 0.10;       ///< [m]
  double warn_corr_r  = 1.0 * M_PI / 180.0;  ///< [rad]
  double warn_match_ms  = 50.0;     ///< alignment slower than this
  double warn_submap_ms = 150.0;    ///< submap rebuild slower than this
  int    warn_every_n = 10;         ///< rate limit for a repeating anomaly

  /// Frames per second the lidar is expected to deliver, for the "frame
  /// delivery" line of the summary. 0 infers it from the arrival intervals.
  ///
  /// Inference is enough for IRREGULAR loss -- the case actually measured, two
  /// bag replays at once each receiving a fluctuating 30-100% of the stream --
  /// and it adapts to whatever lidar is fitted. It is blind to PERFECTLY
  /// UNIFORM loss: if exactly every other frame vanishes, every surviving
  /// interval is one whole period longer and a 10 Hz sensor is
  /// indistinguishable from a 5 Hz one. Setting this pins the yardstick and
  /// closes that hole, at the cost of having to be right.
  double expected_frame_hz = 0.0;
};

/// Publishing choices owned by the ROS layer.
struct OutputParams {
  bool publish_imu_rate = true;
  bool publish_submap   = true;
  bool publish_when_invalid = false;  ///< publish /loc/odom with valid=false before init
  std::string save_trajectory;        ///< TUM file written on shutdown; empty disables
  /// Per-match CSV trace (see loc/match_log.h). Empty disables. Diagnostic
  /// only, but it carries the raw registration pose, which no topic does.
  std::string match_log;
  /// Publish /loc/odom_raw, the pose before the output rate limiter. The
  /// difference against /loc/odom is exactly what the limiter costs.
  bool publish_odom_raw = false;

  /// Run the smoothing layer at all. Default OFF.
  ///
  /// Round 2 measured what it cost as the main output: ATE RMSE 0.601 vs 0.190,
  /// rotation 1.637 deg vs 0.159 deg, and it produced the single largest
  /// per-frame output jump in the whole run (4.86 m). It was not trading
  /// accuracy for smoothness -- it was worse at both, because its own state was
  /// diverging (see design doc 20.5 / 21). /loc/odom now carries the estimate
  /// directly; when this is enabled the smoothed pose goes to /loc/odom_smooth
  /// as a separate topic, so it can be evaluated without being in the way.
  bool smooth_output = false;
  std::string odom_frame = "world";
  std::string map_frame  = "map";
};

/// Everything the backend needs. Assembled once at startup and then read-only.
struct LocParams {
  MapParams       map;
  SubmapParams    submap;
  ScanParams      scan;
  RegParams       track;   ///< tracking registration
  RegParams       coarse;  ///< initialization / relocalization registration
  EstimatorParams est;
  ComposerParams  composer;
  InitParams      init;
  LostParams      lost;
  FrontendWatchParams watch;
  TriggerParams   trigger;
  OutputParams    output;
  LogParams       log;

  /// Defaults for the coarse pass differ from tracking; call after construction
  /// if the caller has not set them explicitly.
  static LocParams Defaults() {
    LocParams p;
    p.coarse.target_voxel  = 2.0f;
    p.coarse.source_voxel  = 0.5f;
    p.coarse.max_corr_dist = 3.0;
    // 30, not 10: the probe on this map shows max_iter=10 leaves the coarse
    // pass unconverged in 10/25 cases at frame 12062 and 25/25 at frame 2300,
    // while 30 converges 25/25 for ~1-2 ms more.
    p.coarse.max_iter      = 30;
    return p;
  }
};

}  // namespace loc
}  // namespace LI2Sup

#endif  // LOC_PARAMS_H_
