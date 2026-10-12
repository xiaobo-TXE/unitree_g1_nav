/**
 * @file gnss_types.h
 * @brief GNSS data types for MapLoc. No ROS, no PCL — testable without a node.
 *
 * Scope note: this is the `record_only` rung of the ladder in
 * docs/gnss_integration_design.md section 8
 * (disabled -> record_only -> assist_init -> fuse_xy). Everything needed to
 * INGEST, VET and DIAGNOSE a fix lives here. The types the FUSION stages need
 * (PositionObservation, GnssPreparation) are deliberately absent: they cannot
 * be written honestly without a georef.yaml, and georef.yaml cannot be
 * calibrated without a receiver that has seen a satellite.
 *
 * Which is the point of this rung. The 2026-09-22 recording ran 19 minutes with
 * `status.status == STATUS_FIX` on 2095 messages, and every one of them carried
 * the same position — satellites_visible was 0 the whole time and gpsd fell
 * back to a default coordinate while still claiming a 3D fix. Nothing in the
 * NavSatFix message says "this is fiction". So the two checks that would have
 * caught it in the first minute are first-class here, not afterthoughts:
 * `satellites_visible` (see GnssParams::min_satellites) and the frozen-position
 * run (see GnssHealth::frozen_run).
 *
 * Units: degrees at the message boundary, metres and seconds everywhere else,
 * variances in m^2 inside the structs. Geodetic quantities stay `double` for
 * their whole life; they must never be narrowed to BASIC::scalar (float), whose
 * resolution at the 6.4e6 m scale of ECEF is about 0.5 m.
 */
#ifndef LOC_GNSS_TYPES_H_
#define LOC_GNSS_TYPES_H_

#include <cstdint>

#include <Eigen/Core>

namespace LI2Sup {
namespace loc {

/// Capability ladder. Each rung is a superset of the one below and each is
/// separately regression-testable against "GNSS off changes nothing".
/// Only kDisabled and kRecordOnly are implemented; the parser accepts the two
/// higher names and refuses to pretend, falling back to kRecordOnly with a log.
enum class GnssMode : std::uint8_t {
  kDisabled   = 0,  ///< no subscription at all
  kRecordOnly = 1,  ///< subscribe, vet, count, publish diagnostics; never fuse
  kAssistInit = 2,  ///< (not implemented) GNSS may seed the init search
  kFuseXy     = 3   ///< (not implemented) horizontal EKF updates
};

/// GNSS admission, orthogonal to LocState (docs/gnss_data_collection §5.4).
/// Slow to promote, fast to demote. record_only never leaves the first three.
enum class GnssAdmission : std::uint8_t {
  kDisabled  = 0,
  kNoGeoref  = 1,  ///< no map<->ENU calibration, so no map-frame position exists
  kNoSignal  = 2,  ///< nothing usable arrived within fix_timeout_s
  kProbation = 3,  ///< (not implemented) residuals computed, no update applied
  kAdmitted  = 4   ///< (not implemented) horizontal updates applied
};

/// Why a fix did not become an observation. Ordered roughly by how early the
/// check runs, so the worst reason seen in a window is also the earliest one.
enum class GnssReject : std::uint8_t {
  kNone = 0,
  kDisabled,        ///< the mode does not fuse; the normal record_only answer
  kNoFix,           ///< status.status < 0
  kNoSatellites,    ///< claims a fix, reports fewer than min_satellites in view
  kNotFinite,       ///< NaN/Inf in latitude, longitude or altitude
  kOutOfRange,      ///< |lat| > 90 or |lon| > 180
  kBadCovariance,   ///< covariance unusable even after the configured floors
  kStaleClock,      ///< header.stamp is not in this machine's time domain
  kDuplicateEpoch,  ///< bit-identical repeat of the previous epoch
  kRateLimited,     ///< arrived inside 1/max_rate_hz of the last accepted one
  kFrozen,          ///< position has not moved for frozen_warn_n epochs
  kQueueFull,       ///< dropped at the queue to make room for a newer fix
  kNoGeoref,        ///< no map<->ENU calibration loaded
  kTooOld,          ///< older than max_latency_s; no state rollback is done
  kOdomGap,         ///< odometry history does not bracket the measurement time
  kTimeJump         ///< falls before a detected clock reset (bag replay)
};

const char* toString(GnssMode m);
const char* toString(GnssAdmission a);
const char* toString(GnssReject r);

/// One GNSS epoch, already off the ROS message and vetted by the adapter.
struct GnssFix {
  double t_meas = 0.0;   ///< measurement time (header.stamp + stamp_offset), LIO time domain
  double t_recv = 0.0;   ///< arrival time on the node clock; measures latency, never fused
  double lat_deg = 0.0;
  double lon_deg = 0.0;
  double alt_m   = 0.0;  ///< datum unconfirmed (MSL vs ellipsoid) -> not used

  /// 3x3 ENU covariance [m^2], already checked element by element. Entries the
  /// receiver did not supply carry the configured floor, not zero.
  Eigen::Matrix3d cov_enu = Eigen::Matrix3d::Zero();
  bool cov_h_known = false;  ///< horizontal variance came from the receiver
  bool cov_v_known = false;

  bool has_fix = false;              ///< status.status >= STATUS_FIX
  bool stamp_is_measurement = true;  ///< false => receive time was substituted

  /// From /extended_fix only; -1 when that topic is absent or the field is empty.
  int    satellites_visible = -1;
  int    satellites_used    = -1;
  double hdop = -1.0;

  /// How many consecutive epochs (including this one) reported a bit-identical
  /// latitude/longitude. 1 for a healthy moving receiver. See GnssHealth.
  int frozen_run = 1;
};

/// What the ROS adapter did with the stream, on T0. Separate from the backend
/// counters below because the T1 loop is driven by frontend frames: when the
/// frontend stalls, these keep moving and the backend ones do not, and that
/// difference is exactly what identifies where a stall is.
struct GnssIngestStats {
  std::uint32_t n_in = 0;          ///< NavSatFix callbacks entered
  std::uint32_t n_no_fix = 0;      ///< status.status < 0
  std::uint32_t n_not_finite = 0;  ///< NaN/Inf or out of range
  std::uint32_t n_bad_cov = 0;     ///< covariance replaced by the floor
  std::uint32_t n_dedup = 0;       ///< bit-identical epoch repeats
  std::uint32_t n_rate_limited = 0;
  std::uint32_t n_stale_clock = 0;
  std::uint32_t n_enqueued = 0;
  std::uint32_t n_dropped = 0;     ///< shed at a full queue
  std::uint32_t n_frame_id = 0;    ///< frame_id did not match expect_frame_id
  std::uint32_t n_extended = 0;    ///< /extended_fix callbacks entered
};

/// One snapshot of GNSS health, published on /loc/gnss_status. Assembled on T0
/// from the adapter counters plus whatever the backend last recorded.
///
/// Every field is about GNSS only. Nothing here may be written into, or read
/// from, the lidar health in LocStatus: a GNSS outage must not be able to
/// manufacture a LOST, and a GNSS success must not be able to hide one
/// (docs/gnss_ros_interface.md section 7).
struct GnssHealth {
  GnssMode      mode      = GnssMode::kDisabled;
  GnssAdmission admission = GnssAdmission::kDisabled;
  bool georef_valid = false;  ///< always false until the calibration stage lands
  /// A message arrived within fix_timeout_s -- NOT that it was usable. A
  /// receiver with no antenna keeps talking; that is precisely what makes it
  /// hard to spot. Pair this with reject_reason to tell the two apart.
  bool receiving    = false;

  double last_fix_stamp = 0.0;  ///< t_meas of the newest enqueued fix
  double last_fix_recv  = 0.0;  ///< t_recv of the same, for the age on T0
  double last_msg_recv  = 0.0;  ///< arrival of the last message, accepted or not
  double last_fuse_stamp = 0.0; ///< stays 0 until fusion exists
  double latency = 0.0;         ///< t_recv - t_meas of the newest fix [s]

  GnssReject reject_reason = GnssReject::kDisabled;
  std::int32_t reject_streak = 0;

  GnssIngestStats ingest;       ///< T0 side
  std::uint32_t n_examined = 0; ///< fixes the backend drained (T1 side)
  std::uint32_t n_waiting = 0;  ///< currently queued

  /// Last quality report seen. -1 means the field was never supplied.
  int    satellites_visible = -1;
  int    satellites_used    = -1;
  double hdop = -1.0;

  /// Longest run of consecutive bit-identical positions in this session, and
  /// the run in progress. A receiver that is moving must break these. The
  /// 2026-09-22 bag reached 2095 while the robot drove several laps.
  int frozen_run = 0;
  int frozen_run_max = 0;

  /// Horizontal 1 sigma [m] that WOULD be used, and whether it came from the
  /// receiver. On the hardware measured so far this is always the configured
  /// floor, and that fact has to be visible on the topic rather than buried in
  /// a config file.
  double sigma_h = 0.0;
  bool   sigma_h_from_receiver = false;
};

}  // namespace loc
}  // namespace LI2Sup

#endif  // LOC_GNSS_TYPES_H_
