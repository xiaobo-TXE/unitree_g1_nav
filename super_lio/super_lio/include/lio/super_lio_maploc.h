/**
 * @file super_lio_maploc.h
 * @brief SuperLIO plus map localization. The frontend algorithm is untouched.
 *
 * The entire integration is one override: Output() runs the base class first,
 * then does two cheap things — hand the backend a copy of this frame, and
 * compose and publish the corrected pose. Both are sub-millisecond, so the
 * frontend's 10 Hz cadence is unaffected.
 *
 * What this class must never do (see the threading rules in map_localizer.h):
 * pass any frontend member to the backend by reference. scan_undistort_full_ in
 * particular is resized in place every frame, so the copy in Output() is not
 * optional — sharing the handle would hand T1 the *next* frame's points.
 */
#ifndef SUPER_LIO_MAPLOC_H_
#define SUPER_LIO_MAPLOC_H_

#include <memory>

#include "lio/super_lio.h"
#include "loc/keyframe_map.h"
#include "loc/loc_params.h"
#include "loc/map_localizer.h"
#include "loc/pose_composer.h"
#include "ros/loc_publisher.h"

namespace LI2Sup {

class SuperLIOMapLoc : public SuperLIO {
public:
  explicit SuperLIOMapLoc(const loc::LocParams& params) : params_(params), composer_(params.composer) {}
  ~SuperLIOMapLoc() override;

  /// Runs SuperLIO::init(), then loads the map and starts the backend thread.
  void init() override;

  /// Writes the trajectory file, if one was requested. Called on shutdown.
  void finish();

protected:
  void Output() override;

private:
  /// Publishes /loc/status at least once a second even when no match ran, so a
  /// stalled backend is visible rather than silent.
  void publishStatusIfDue(double stamp, bool jumped);

  /// Compare the configured mounting against what gravity says, once.
  void reportMounting(const BASIC::SE3& T_O_B) const;

  loc::LocParams params_;
  loc::KeyframeMap::Ptr map_;
  std::unique_ptr<loc::MapLocalizer> localizer_;
  std::unique_ptr<LocPublisher> publisher_;
  loc::PoseComposer composer_;

  BASIC::SE3 applied_drift_;      ///< last drift the composer applied, for the IMU path
  bool   have_applied_ = false;
  double last_status_pub_ = -1.0;
  bool   last_jump_ = false;
  std::uint32_t last_jump_seq_ = 0;
  bool mounting_calibrated_ = false;
};

}  // namespace LI2Sup

#endif  // SUPER_LIO_MAPLOC_H_
