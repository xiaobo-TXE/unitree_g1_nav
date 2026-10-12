/**
 * @file submap_manager.h
 * @brief Keeps a stitched local map around the robot, rebuilt only when needed.
 *
 * Interface contract: one call, update(). It decides whether a rebuild is due
 * (centre moved past refresh_dist, first call, or the candidate set changed by
 * more than a third), does the disk reads and the stitching, and returns the
 * current submap. Callers never ask "should I refresh?" separately.
 *
 * The returned Submap carries a plain point cloud in the map frame; the target
 * structure a registration backend wants (voxel map, kd-tree, covariances) is
 * built behind IRegistrar, keyed on `version`. That keeps small_gicp types out
 * of this interface and lets the backend be swapped without touching this file.
 *
 * Thread affinity: backend thread (T1) only; update() blocks on disk.
 */
#ifndef LOC_SUBMAP_MANAGER_H_
#define LOC_SUBMAP_MANAGER_H_

#include <memory>
#include <vector>

#include "loc/keyframe_map.h"
#include "loc/loc_params.h"

namespace LI2Sup {
namespace loc {

/// A stitched neighbourhood of the map. `version` changes iff `cloud_M` changed,
/// which is what downstream caches key on.
struct Submap {
  std::uint64_t   version = 0;
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  std::vector<int> ids;
  CloudXYZ::Ptr   cloud_M;   ///< map frame; null until the first successful build
  double          build_ms = 0.0;

  bool valid() const { return cloud_M && !cloud_M->empty(); }
};

class SubmapManager {
public:
  SubmapManager(KeyframeMap::Ptr map, const SubmapParams& params,
                const LogParams& log = LogParams())
      : map_(std::move(map)), params_(params), log_(log) {}

  /// Returns the submap covering `p_pred` (predicted body position in the map
  /// frame), rebuilding first if the current one no longer fits. Cheap when it
  /// already fits: a radius query and a set comparison.
  /// `radius`/`max_k` override the configured values when > 0, which is how the
  /// initialization search asks for its wider submap.
  const Submap& update(const Eigen::Vector3f& p_pred, float radius = -1.0f, int max_k = -1);

  /// Force the next update() to rebuild (used after a relocalization jump).
  void invalidate() { submap_.version = 0; submap_.cloud_M.reset(); submap_.ids.clear(); }

  const Submap& current() const { return submap_; }

private:
  bool needRefresh(const Eigen::Vector3f& p_pred, const std::vector<int>& candidates) const;
  void rebuild(const Eigen::Vector3f& p_pred, std::vector<int> ids);

  KeyframeMap::Ptr map_;
  SubmapParams     params_;
  LogParams        log_;
  Submap           submap_;
  std::uint64_t    next_version_ = 1;
};

}  // namespace loc
}  // namespace LI2Sup

#endif  // LOC_SUBMAP_MANAGER_H_
