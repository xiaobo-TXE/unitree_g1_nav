#include "loc/submap_manager.h"

#include <chrono>
#include <algorithm>

#include <pcl/common/transforms.h>

#include "basic/logs.h"

namespace LI2Sup {
namespace loc {

const Submap& SubmapManager::update(const Eigen::Vector3f& p_pred, float radius, int max_k) {
  const float  r = radius > 0.0f ? radius : params_.radius;
  const int    k = max_k  > 0    ? max_k  : params_.max_k;

  auto candidates = map_->queryNear(p_pred, r, k);
  if (candidates.empty()) {
    // Drop what we have rather than hand back a submap for somewhere else.
    // Returning the previous one made every caller silently align against
    // geometry that could be a hundred metres away -- which reads as "0 inliers,
    // registration is broken" instead of "the robot is not on the map". A
    // caller that gets an invalid submap can at least say so.
    LOG_EVERY_N(WARNING, 20) << YELLOW << " ---> [MapLoc] no keyframe within " << r
                             << " m of (" << p_pred.transpose()
                             << "); submap invalidated -- off the map, or the drift"
                                " estimate is wrong" << RESET;
    invalidate();
    return submap_;
  }
  if (!needRefresh(p_pred, candidates)) return submap_;

  rebuild(p_pred, std::move(candidates));

  // Warm the cache for where we are heading, so the next rebuild is RAM-only.
  // Bounded: this is synchronous, on the same thread that just rebuilt.
  auto ahead = map_->queryNear(p_pred, r * params_.prefetch_scale,
                               std::max(0, params_.prefetch_max_files));
  map_->prefetch(ahead);
  return submap_;
}

bool SubmapManager::needRefresh(const Eigen::Vector3f& p_pred,
                                const std::vector<int>& candidates) const {
  if (!submap_.valid()) return true;
  if ((p_pred - submap_.center).norm() > params_.refresh_dist) return true;

  // Revisited corridors gain candidates without the robot moving far; refresh
  // when the retrieved set has drifted by more than a third.
  std::vector<int> a = submap_.ids, b = candidates;
  std::sort(a.begin(), a.end());
  std::sort(b.begin(), b.end());
  std::vector<int> common;
  std::set_intersection(a.begin(), a.end(), b.begin(), b.end(), std::back_inserter(common));
  const std::size_t larger = std::max(a.size(), b.size());
  if (larger == 0) return true;
  const double changed = 1.0 - static_cast<double>(common.size()) / static_cast<double>(larger);
  return changed > (1.0 / 3.0);
}

void SubmapManager::rebuild(const Eigen::Vector3f& p_pred, std::vector<int> ids) {
  const auto t0 = std::chrono::steady_clock::now();

  auto stitched = CloudXYZ::Ptr(new CloudXYZ());
  std::size_t reserve = 0;
  for (int id : ids) reserve += static_cast<std::size_t>(std::max(0, map_->kf(id).n_points));
  stitched->reserve(reserve ? reserve : 200000);

  CloudXYZ transformed;
  int missing = 0;
  for (int id : ids) {
    auto cloud = map_->cloud(id);
    if (!cloud || cloud->empty()) { ++missing; continue; }
    // Keyframe clouds are stored in their own body frame precisely so that a
    // re-optimized keyframes.txt does not invalidate the pcd files.
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3, 3>(0, 0) = map_->kf(id).T_M_B.R_.template cast<float>();
    T.block<3, 1>(0, 3) = map_->kf(id).T_M_B.t_.template cast<float>();
    pcl::transformPointCloud(*cloud, transformed, T);
    *stitched += transformed;
  }

  if (stitched->empty()) {
    LOG(WARNING) << " ---> [MapLoc] submap rebuild produced no points (" << missing
                 << " of " << ids.size() << " keyframe clouds unreadable)";
    return;
  }

  submap_.cloud_M = stitched;
  submap_.ids     = std::move(ids);
  submap_.center  = p_pred;
  submap_.version = next_version_++;
  submap_.build_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

  // A healthy rebuild is not news: it happens every few metres for the whole
  // run, and one INFO per rebuild was the bulk of the daily log. The count and
  // the timing distribution go to RunStats and come back in the heartbeat and
  // the summary. Three cases still print:
  //   - the first one, because "the map is reachable and stitching works" is
  //     exactly what a board bring-up needs confirmed;
  //   - a slow one, because on flash storage this is where the board will hurt;
  //   - all of them under log.debug.
  const bool first = (submap_.version == 1);
  const bool slow  = log_.warn_submap_ms > 0.0 && submap_.build_ms > log_.warn_submap_ms;
  if (slow) {
    LOG(WARNING) << YELLOW << " ---> [MapLoc] slow submap rebuild: v" << submap_.version
                 << ", " << submap_.ids.size() << " keyframes, " << stitched->size()
                 << " points, " << submap_.build_ms << " ms (over "
                 << log_.warn_submap_ms << " ms; " << missing << " unreadable)" << RESET;
  } else if (first || log_.debug) {
    LOG(INFO) << " ---> [MapLoc] submap v" << submap_.version << ": " << submap_.ids.size()
              << " keyframes, " << stitched->size() << " points, "
              << submap_.build_ms << " ms";
  }
}

}  // namespace loc
}  // namespace LI2Sup
