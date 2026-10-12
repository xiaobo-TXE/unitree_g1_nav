/**
 * @file gnss_subscriber.h
 * @brief The only place in MapLoc that knows GNSS arrives as a ROS message.
 *
 * Kept out of LocPublisher on purpose: that class is the OUTPUT side, and its
 * one subscription (/initialpose) is a human command, not a sensor stream.
 *
 * This class is a transcription layer and nothing more: it turns a NavSatFix
 * into a loc::RawFix, asks loc::GnssVetter for a verdict, and forwards what
 * survives. Every judgement -- NaN handling, dedup, the frozen-position run,
 * the satellite gate -- lives in the vetter, so all of it is testable without a
 * node, an executor or a receiver. See loc/gnss_vetter.h for what each check is
 * answering to.
 *
 * It does not project, interpolate, filter or take any lock the backend holds.
 * Everything that needs the map, the odometry or the filter state happens on T1.
 *
 * ---------------------------------------------------------------------------
 * What the vetting is actually defending against, measured on this rig:
 *
 *  1. NaN covariance. Both horizontal entries of position_covariance are NaN on
 *     EVERY epoch (2283/2283 on the 2026-09-22 bag) while covariance_type says
 *     2 (DIAGONAL_KNOWN). A NaN that reaches the estimator passes the gate
 *     rather than failing it — `chi2 > threshold` is FALSE for NaN — so it must
 *     be stopped here, at the boundary, not filtered later.
 *
 *  2. Duplicate epochs. gpsd_client publishes ~2 Hz from a 1 Hz solution:
 *     45.9 % of messages repeat the previous epoch bit for bit. Fusing both
 *     copies would halve the apparent variance of an observation that carries
 *     no new information.
 *
 *  3. A fix that is not a fix. On 2026-09-22 the receiver reported
 *     status.status == STATUS_FIX for 19 minutes with satellites_visible == 0,
 *     while gpsd substituted a default coordinate; the position was
 *     bit-identical across 2095 messages and several laps of driving. NavSatFix
 *     alone cannot express this, so two checks cover it: the satellite count
 *     from /extended_fix, and a frozen-position run counted here. Without both,
 *     a disconnected antenna looks exactly like a perfectly steady one.
 * ---------------------------------------------------------------------------
 *
 * Thread affinity: the node executor thread (T0), like every other callback.
 */
#ifndef LOC_GNSS_SUBSCRIBER_H_
#define LOC_GNSS_SUBSCRIBER_H_

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#ifdef HAVE_GPS_MSGS
#include <gps_msgs/msg/gps_fix.hpp>
#endif

#include "loc/gnss_types.h"
#include "loc/gnss_vetter.h"
#include "loc/loc_params.h"

namespace LI2Sup {

class GnssSubscriber {
public:
  using FixCallback = std::function<void(const loc::GnssFix&)>;

  /// Subscribes immediately unless params.mode is disabled. `node` must outlive
  /// this object (it is the LIO node, which owns everything here).
  GnssSubscriber(rclcpp::Node* node, const loc::GnssParams& params);

  /// Where a vetted epoch goes. Set once, before the executor spins.
  void setFixCallback(FixCallback cb) { fix_cb_ = std::move(cb); }

  /// Everything the callback has counted. Cheap; safe from any thread.
  loc::GnssIngestStats stats() const;

  /// The newest vetted epoch plus the quality context it arrived with. Read by
  /// the status publisher on T0.
  struct Snapshot {
    double last_fix_meas = 0.0;
    double last_fix_recv = 0.0;
    /// Arrival time of the last message of ANY kind, accepted or refused.
    /// Deliberately separate from last_fix_recv: "the receiver is talking to
    /// us" and "we have a usable fix" are different failures with different
    /// fixes, and collapsing them sends you to check cabling and QoS when the
    /// wire is fine and the antenna is not.
    double last_msg_recv = 0.0;
    double latency = 0.0;       ///< t_recv - t_meas; live only, meaningless under replay
    int    satellites_visible = -1;
    int    satellites_used = -1;
    double hdop = -1.0;
    int    frozen_run = 0;
    int    frozen_run_max = 0;
    loc::GnssReject last_reject = loc::GnssReject::kNone;
    std::int32_t reject_streak = 0;
    bool   have_extended = false;  ///< at least one /extended_fix message arrived
  };
  Snapshot snapshot() const;

  /// True when the subscription was actually created.
  bool active() const { return sub_fix_ != nullptr; }

  /// One line for the startup banner: topics, QoS choice and which checks are
  /// armed. Printed by the caller so it lands with the rest of the H1 dump.
  std::string describe() const;

private:
  void onFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
#ifdef HAVE_GPS_MSGS
  void onExtended(const gps_msgs::msg::GPSFix::SharedPtr msg);
#endif

  rclcpp::Node* node_;
  loc::GnssParams params_;
  FixCallback fix_cb_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_fix_;
#ifdef HAVE_GPS_MSGS
  rclcpp::Subscription<gps_msgs::msg::GPSFix>::SharedPtr sub_extended_;
#endif

  /// Serializes the vetter, which keeps the previous-epoch state and is not
  /// thread safe on its own. /fix and /extended_fix may land on different
  /// executor threads.
  mutable std::mutex mutex_;
  loc::GnssVetter vetter_;
  Snapshot snap_;
};

}  // namespace LI2Sup

#endif  // LOC_GNSS_SUBSCRIBER_H_
