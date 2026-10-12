#include "loc/scan_accumulator.h"

#include <chrono>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

namespace LI2Sup {
namespace loc {

bool FrameQueue::push(FrontendFrame frame) {
  bool dropped = false;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (stopped_) return true;
    while (q_.size() >= capacity_) {
      q_.pop_front();
      ++dropped_;
      dropped = true;
    }
    q_.push_back(std::move(frame));
  }
  cv_.notify_one();
  return !dropped;
}

bool FrameQueue::waitAndPopAll(std::vector<FrontendFrame>* out) {
  std::unique_lock<std::mutex> lk(mutex_);
  cv_.wait(lk, [this] { return stopped_ || !q_.empty(); });
  if (stopped_ && q_.empty()) return false;
  out->clear();
  out->reserve(q_.size());
  while (!q_.empty()) {
    out->push_back(std::move(q_.front()));
    q_.pop_front();
  }
  return true;
}

std::size_t FrameQueue::popAll(std::vector<FrontendFrame>* out) {
  std::lock_guard<std::mutex> lk(mutex_);
  out->clear();
  out->reserve(q_.size());
  while (!q_.empty()) {
    out->push_back(std::move(q_.front()));
    q_.pop_front();
  }
  return out->size();
}

void FrameQueue::stop() {
  {
    std::lock_guard<std::mutex> lk(mutex_);
    stopped_ = true;
  }
  cv_.notify_all();
}

std::size_t FrameQueue::size() const {
  std::lock_guard<std::mutex> lk(mutex_);
  return q_.size();
}

void ScanAccumulator::add(FrontendFrame frame) {
  if (!frame.cloud_B || frame.cloud_B->empty()) return;
  frames_.push_back(std::move(frame));
  while (frames_.size() > static_cast<std::size_t>(std::max(1, params_.accum_frames))) {
    frames_.pop_front();
  }
}

void ScanAccumulator::add(const std::vector<FrontendFrame>& frames) {
  for (const auto& f : frames) add(f);
}

bool ScanAccumulator::latest(double* t, BASIC::SE3* T_O_B) const {
  if (frames_.empty()) return false;
  const auto& f = frames_.back();
  if (t) *t = f.t;
  if (T_O_B) *T_O_B = f.T_O_B;
  return true;
}

SourceScan ScanAccumulator::build() const {
  SourceScan scan;
  if (frames_.empty()) return scan;

  ++build_count_;
  const auto t0 = std::chrono::steady_clock::now();
  const FrontendFrame& ref = frames_.back();
  const BASIC::SE3 T_Bk_O = ref.T_O_B.inverse();

  auto merged = CloudXYZ::Ptr(new CloudXYZ());
  std::size_t reserve = 0;
  for (const auto& f : frames_) reserve += f.cloud_B->size();
  merged->reserve(reserve);

  CloudXYZ transformed;
  for (const auto& f : frames_) {
    // p_Bk = T_O_Bk^-1 * T_O_Bj * p_Bj
    const BASIC::SE3 T_Bk_Bj = T_Bk_O * f.T_O_B;
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3, 3>(0, 0) = T_Bk_Bj.R_.template cast<float>();
    T.block<3, 1>(0, 3) = T_Bk_Bj.t_.template cast<float>();
    pcl::transformPointCloud(*f.cloud_B, transformed, T);
    *merged += transformed;
  }

  if (params_.source_voxel > 0.0f) {
    auto filtered = CloudXYZ::Ptr(new CloudXYZ());
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(params_.source_voxel, params_.source_voxel, params_.source_voxel);
    vg.setInputCloud(merged);
    vg.filter(*filtered);
    merged = filtered;
  }

  scan.t        = ref.t;
  scan.T_O_B    = ref.T_O_B;
  scan.cloud_B  = merged;
  scan.n_frames = static_cast<int>(frames_.size());
  scan.build_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  return scan;
}

}  // namespace loc
}  // namespace LI2Sup
