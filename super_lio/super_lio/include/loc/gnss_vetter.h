/**
 * @file gnss_vetter.h
 * @brief Everything that decides whether a GNSS epoch is worth keeping.
 *
 * Split out of GnssSubscriber so it can be tested the way the rest of this
 * module is tested: plain data in, verdict out, no node, no executor, no
 * network. The checks here are the ones that failed in the field, so they are
 * the ones that most need to stay pinned by tests — and a check that can only
 * be exercised by standing a receiver on a roof is a check that will rot.
 *
 * The verdicts, in the order they are applied and with the evidence each one
 * answers to (2026-09-22, 19 min, 2283 messages):
 *
 *   kNotFinite / kOutOfRange   NaN latitude/longitude, or a NaN anywhere in
 *                              position_covariance. Both horizontal covariance
 *                              entries were NaN on 2283/2283 messages while
 *                              covariance_type claimed DIAGONAL_KNOWN, so the
 *                              declared type is never consulted. This has to
 *                              happen at the boundary: `chi2 > gate` evaluates
 *                              FALSE for NaN, so a NaN does not fail a gate
 *                              downstream, it PASSES one.
 *
 *   kNoFix                     status.status < STATUS_FIX. 188 messages.
 *
 *   kDuplicateEpoch            The same solution published twice. 45.9 % of the
 *                              stream. Fusing both copies would halve the
 *                              apparent variance of one observation.
 *
 *   kFrozen                    A NEW epoch carrying the OLD coordinate. Dedup
 *                              cannot see this because the stamp advances. The
 *                              longest run in that bag was 2095 messages, while
 *                              the robot drove several laps.
 *
 *   kNoSatellites              Claims a fix with too few satellites in view.
 *                              Needs /extended_fix; NavSatFix has no such
 *                              field, which is why a disconnected antenna is
 *                              otherwise indistinguishable from a steady one.
 *
 *   kRateLimited               Faster than max_rate_hz.
 *
 * Not thread safe by itself; GnssSubscriber owns one and serializes access.
 */
#ifndef LOC_GNSS_VETTER_H_
#define LOC_GNSS_VETTER_H_

#include <string>

#include "loc/gnss_types.h"
#include "loc/loc_params.h"

namespace LI2Sup {
namespace loc {

/// A NavSatFix, with the ROS types removed and nothing else changed.
/// Field names and semantics match sensor_msgs/msg/NavSatFix exactly, so the
/// adapter is a transcription and this file holds all the judgement.
struct RawFix {
  double stamp = 0.0;   ///< header.stamp in seconds; <= 0 means "not set"
  double recv  = 0.0;   ///< arrival time in the same units
  std::string frame_id;
  int    status = 0;    ///< NavSatStatus::status; < 0 is STATUS_NO_FIX
  double lat_deg = 0.0;
  double lon_deg = 0.0;
  double alt_m   = 0.0;
  /// position_covariance, row major ENU. Any element may be NaN; that is the
  /// normal case on this hardware, not an exception.
  double cov[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
};

class GnssVetter {
public:
  explicit GnssVetter(const GnssParams& params) : params_(params) {}

  /// Returns kNone and fills *out when the epoch should be enqueued; otherwise
  /// returns the first check that refused it and leaves *out untouched.
  ///
  /// kFrozen is the one verdict that still fills *out and still returns kNone:
  /// capturing a frozen stream is the entire point of record_only, so the epoch
  /// is passed along with GnssFix::frozen_run set and the caller decides. A
  /// fusing rung must refuse it; a recording rung must not.
  GnssReject vet(const RawFix& raw, GnssFix* out);

  /// Latest quality report from /extended_fix. Carried onto the next epoch
  /// rather than paired strictly by stamp: it gates and annotates, and is never
  /// a position source, so a one-epoch skew cannot move the robot.
  void setQuality(int satellites_visible, int satellites_used, double hdop);

  const GnssIngestStats& stats() const { return stats_; }
  int frozenRun() const { return frozen_run_; }
  int frozenRunMax() const { return frozen_run_max_; }
  int satellitesVisible() const { return sats_visible_; }
  int satellitesUsed() const { return sats_used_; }
  double hdop() const { return hdop_; }
  bool haveQuality() const { return have_quality_; }

  /// Forget the previous epoch. Used when the clock jumps, so an epoch from
  /// before a bag replay restart cannot be compared against one from after it.
  void reset();

private:
  /// position_covariance -> a matrix a filter may divide by. Element by
  /// element; covariance_type is never consulted. Unknown entries become the
  /// configured floor, never zero — zero variance reads as infinite precision.
  /// Returns false when the result is still not finite.
  bool sanitizeCovariance(const RawFix& raw, GnssFix* out);

  GnssParams params_;
  GnssIngestStats stats_;

  bool   have_prev_ = false;
  double prev_t_ = 0.0, prev_lat_ = 0.0, prev_lon_ = 0.0, prev_alt_ = 0.0;
  bool   have_enqueued_ = false;
  double last_enqueued_t_ = 0.0;
  int    frozen_run_ = 0;
  int    frozen_run_max_ = 0;

  bool   have_quality_ = false;
  int    sats_visible_ = -1;
  int    sats_used_ = -1;
  double hdop_ = -1.0;
};

}  // namespace loc
}  // namespace LI2Sup

#endif  // LOC_GNSS_VETTER_H_
