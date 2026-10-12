#include "loc/keyframe_map.h"

#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <filesystem>

#include <pcl/io/pcd_io.h>

#include "basic/logs.h"

namespace LI2Sup {
namespace loc {

namespace {

/// meta.yaml is written by loc_map_builder and is deliberately flat, so a
/// two-line "key: value" reader is enough and we avoid a yaml-cpp dependency.
bool readFlatYaml(const std::string& path, std::unordered_map<std::string, std::string>* out) {
  std::ifstream f(path);
  if (!f.is_open()) return false;
  std::string line;
  while (std::getline(f, line)) {
    const auto hash = line.find('#');
    if (hash != std::string::npos) line = line.substr(0, hash);
    const auto colon = line.find(':');
    if (colon == std::string::npos) continue;
    std::string key = line.substr(0, colon);
    std::string val = line.substr(colon + 1);
    auto trim = [](std::string& s) {
      const auto b = s.find_first_not_of(" \t\r\n");
      const auto e = s.find_last_not_of(" \t\r\n");
      s = (b == std::string::npos) ? std::string() : s.substr(b, e - b + 1);
    };
    trim(key);
    trim(val);
    if (!key.empty()) (*out)[key] = val;
  }
  return true;
}

std::vector<double> parseList(const std::string& s) {
  std::vector<double> out;
  std::string t = s;
  for (char& c : t) {
    if (c == '[' || c == ']' || c == ',') c = ' ';
  }
  std::istringstream is(t);
  double v;
  while (is >> v) out.push_back(v);
  return out;
}

}  // namespace

bool KeyframeMap::load(const std::string& dir, bool preload_all, int lru_capacity) {
  namespace fs = std::filesystem;
  dir_ = dir;
  if (!dir_.empty() && dir_.back() == '/') dir_.pop_back();
  preload_all_ = preload_all;

  if (!fs::exists(dir_)) {
    LOG(ERROR) << " ---> [MapLoc] map_dir does not exist: " << dir_;
    return false;
  }
  if (!parseMeta(dir_ + "/meta.yaml")) {
    LOG(WARNING) << " ---> [MapLoc] meta.yaml missing or unreadable in " << dir_
                 << "; continuing without build-time metadata";
  }
  if (!parseKeyframes(dir_ + "/keyframes.txt", dir_)) {
    LOG(ERROR) << " ---> [MapLoc] failed to read " << dir_ << "/keyframes.txt";
    return false;
  }
  if (kfs_.empty()) {
    LOG(ERROR) << " ---> [MapLoc] map contains no keyframes";
    return false;
  }

  tree_cloud_.reset(new CloudXYZ());
  tree_cloud_->reserve(kfs_.size());
  bound_min_ = BASIC::V3::Constant(std::numeric_limits<BASIC::scalar>::max());
  bound_max_ = BASIC::V3::Constant(std::numeric_limits<BASIC::scalar>::lowest());
  for (const auto& k : kfs_) {
    tree_cloud_->emplace_back(k.pos.x(), k.pos.y(), k.pos.z());
    bound_min_ = bound_min_.cwiseMin(k.T_M_B.t_);
    bound_max_ = bound_max_.cwiseMax(k.T_M_B.t_);
  }
  tree_.setInputCloud(tree_cloud_);

  {
    std::lock_guard<std::mutex> lk(cache_mutex_);
    cache_.setCapacity(preload_all_ ? kfs_.size() : static_cast<std::size_t>(std::max(1, lru_capacity)));
  }

  if (preload_all_) {
    std::size_t loaded = 0, points = 0;
    for (const auto& k : kfs_) {
      auto c = readFromDisk(k.id);
      if (!c) continue;
      points += c->size();
      std::lock_guard<std::mutex> lk(cache_mutex_);
      cache_.put(k.id, c);
      ++loaded;
    }
    LOG(INFO) << " ---> [MapLoc] preloaded " << loaded << "/" << kfs_.size()
              << " keyframe clouds, " << points << " points";
  }

  loaded_ = true;
  LOG(INFO) << " ---> [MapLoc] loaded map " << dir_ << ": " << kfs_.size()
            << " keyframes, bbox x[" << bound_min_.x() << "," << bound_max_.x()
            << "] y[" << bound_min_.y() << "," << bound_max_.y()
            << "] z[" << bound_min_.z() << "," << bound_max_.z() << "]";
  return true;
}

bool KeyframeMap::parseMeta(const std::string& path) {
  std::unordered_map<std::string, std::string> kv;
  if (!readFlatYaml(path, &kv)) return false;

  auto num = [&kv](const char* key, auto def) {
    auto it = kv.find(key);
    if (it == kv.end()) return static_cast<decltype(def)>(def);
    try {
      return static_cast<decltype(def)>(std::stod(it->second));
    } catch (...) {
      return static_cast<decltype(def)>(def);
    }
  };

  meta_.format_version = num("format_version", 0);
  meta_.source_frames  = num("source_frames", 0);
  meta_.kf_dist        = num("kf_dist", 0.0);
  meta_.kf_angle       = num("kf_angle", 0.0);
  meta_.kf_range       = num("kf_range", 0.0);
  meta_.kf_voxel       = num("kf_voxel", 0.0);
  meta_.global_voxel   = num("global_voxel", 0.0);
  meta_.num_keyframes  = num("num_keyframes", 0);
  meta_.total_points   = static_cast<long long>(num("total_points", 0.0));
  if (kv.count("source_session")) meta_.source_session = kv["source_session"];

  if (kv.count("gravity")) {
    const auto g = parseList(kv["gravity"]);
    if (g.size() == 3) meta_.gravity = BASIC::V3(g[0], g[1], g[2]);
  }
  if (kv.count("extrinsic_lidar_imu")) {
    // 12 numbers: t(3) then R row-major(9), matching lio.extrinsic.lidar_imu.
    const auto e = parseList(kv["extrinsic_lidar_imu"]);
    if (e.size() == 12) {
      BASIC::V3 t(e[0], e[1], e[2]);
      BASIC::M3 R;
      R << e[3], e[4], e[5], e[6], e[7], e[8], e[9], e[10], e[11];
      meta_.T_B_L = BASIC::SE3(R, t);
      meta_.has_extrinsic = true;
    }
  }
  return true;
}

bool KeyframeMap::parseKeyframes(const std::string& path, const std::string& dir) {
  std::ifstream f(path);
  if (!f.is_open()) return false;

  kfs_.clear();
  std::string line;
  int line_no = 0;
  while (std::getline(f, line)) {
    ++line_no;
    if (line.empty() || line[0] == '#') continue;
    std::istringstream is(line);
    // id  t  x y z  qx qy qz qw  src_begin src_end  n_points
    Keyframe k;
    double x, y, z, qx, qy, qz, qw;
    if (!(is >> k.id >> k.stamp >> x >> y >> z >> qx >> qy >> qz >> qw
             >> k.src_begin >> k.src_end >> k.n_points)) {
      LOG(WARNING) << " ---> [MapLoc] keyframes.txt line " << line_no << " malformed, skipped";
      continue;
    }
    BASIC::Quat q(static_cast<BASIC::scalar>(qw), static_cast<BASIC::scalar>(qx),
                  static_cast<BASIC::scalar>(qy), static_cast<BASIC::scalar>(qz));
    q.normalize();
    k.T_M_B = BASIC::SE3(q, BASIC::V3(x, y, z));
    k.pos   = Eigen::Vector3f(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));

    char buf[32];
    std::snprintf(buf, sizeof(buf), "kf/%06d.pcd", k.id);
    k.path = dir + "/" + buf;
    kfs_.push_back(std::move(k));
  }

  // Retrieval indexes by vector position, so ids must be dense and ordered.
  std::sort(kfs_.begin(), kfs_.end(), [](const Keyframe& a, const Keyframe& b) { return a.id < b.id; });
  for (std::size_t i = 0; i < kfs_.size(); ++i) {
    if (kfs_[i].id != static_cast<int>(i)) {
      LOG(ERROR) << " ---> [MapLoc] keyframe ids are not dense (expected " << i
                 << ", got " << kfs_[i].id << ")";
      return false;
    }
  }
  return true;
}

std::vector<int> KeyframeMap::queryNear(const Eigen::Vector3f& p, float radius, int max_k) const {
  std::vector<int> out;
  if (kfs_.empty() || max_k <= 0) return out;

  pcl::PointXYZ q(p.x(), p.y(), p.z());
  std::vector<int> idx;
  std::vector<float> sq_dist;
  // radiusSearch returns ascending distance and already respects max_k.
  if (tree_.radiusSearch(q, radius, idx, sq_dist, max_k) <= 0) return out;
  out.reserve(idx.size());
  for (int i : idx) out.push_back(kfs_[i].id);
  return out;
}

int KeyframeMap::nearestId(const Eigen::Vector3f& p) const {
  if (kfs_.empty()) return -1;
  pcl::PointXYZ q(p.x(), p.y(), p.z());
  std::vector<int> idx(1);
  std::vector<float> sq_dist(1);
  if (tree_.nearestKSearch(q, 1, idx, sq_dist) <= 0) return -1;
  return kfs_[idx[0]].id;
}

int KeyframeMap::nearestIdXY(const Eigen::Vector3f& p) const {
  // Linear scan: 2811 keyframes is well under a millisecond, and this runs once
  // per initialization attempt. A second, 2D tree would have to be kept in sync
  // with the 3D one for no measurable gain.
  int best = -1;
  float best_sq = std::numeric_limits<float>::max();
  for (const auto& k : kfs_) {
    const float dx = k.pos.x() - p.x();
    const float dy = k.pos.y() - p.y();
    const float sq = dx * dx + dy * dy;
    if (sq < best_sq) { best_sq = sq; best = k.id; }
  }
  return best;
}

KeyframeMap::CloudConstPtr KeyframeMap::readFromDisk(int id) const {
  if (id < 0 || id >= static_cast<int>(kfs_.size())) return nullptr;
  const auto t0 = std::chrono::steady_clock::now();
  auto cloud = CloudXYZ::Ptr(new CloudXYZ());
  const int rc = pcl::io::loadPCDFile<pcl::PointXYZ>(kfs_[id].path, *cloud);
  const auto us = static_cast<std::uint64_t>(
      std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count());
  disk_us_.fetch_add(us, std::memory_order_relaxed);
  auto prev = disk_us_max_.load(std::memory_order_relaxed);
  while (us > prev &&
         !disk_us_max_.compare_exchange_weak(prev, us, std::memory_order_relaxed)) {
  }
  if (rc < 0) {
    LOG_EVERY_N(WARNING, 50) << " ---> [MapLoc] cannot read keyframe cloud " << kfs_[id].path;
    return nullptr;
  }
  return cloud;
}

KeyframeMap::CloudConstPtr KeyframeMap::cloud(int id) {
  {
    std::lock_guard<std::mutex> lk(cache_mutex_);
    auto hit = cache_.get(id);
    if (hit) {
      cache_hits_.fetch_add(1, std::memory_order_relaxed);
      return hit;
    }
  }
  cache_misses_.fetch_add(1, std::memory_order_relaxed);
  // Read outside the lock: a miss costs milliseconds and must not block prefetch.
  auto c = readFromDisk(id);
  if (!c) return nullptr;
  std::lock_guard<std::mutex> lk(cache_mutex_);
  cache_.put(id, c);
  return c;
}

void KeyframeMap::prefetch(const std::vector<int>& ids) {
  for (int id : ids) {
    {
      std::lock_guard<std::mutex> lk(cache_mutex_);
      if (cache_.contains(id)) continue;
    }
    auto c = readFromDisk(id);
    if (!c) continue;
    prefetched_.fetch_add(1, std::memory_order_relaxed);
    std::lock_guard<std::mutex> lk(cache_mutex_);
    cache_.put(id, c);
  }
}

std::size_t KeyframeMap::cacheSize() const {
  std::lock_guard<std::mutex> lk(cache_mutex_);
  return cache_.size();
}

KeyframeMap::CacheStats KeyframeMap::cacheStats() const {
  CacheStats s;
  s.hits   = cache_hits_.load(std::memory_order_relaxed);
  s.misses = cache_misses_.load(std::memory_order_relaxed);
  s.prefetched = prefetched_.load(std::memory_order_relaxed);
  s.disk_read_ms = static_cast<double>(disk_us_.load(std::memory_order_relaxed)) / 1000.0;
  s.disk_read_max_ms =
      static_cast<double>(disk_us_max_.load(std::memory_order_relaxed)) / 1000.0;
  return s;
}

KeyframeMap::CloudConstPtr KeyframeMap::globalDisplayCloud() {
  if (global_display_) return global_display_;
  auto c = CloudXYZ::Ptr(new CloudXYZ());
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(dir_ + "/global_ds.pcd", *c) < 0) {
    LOG(WARNING) << " ---> [MapLoc] no global_ds.pcd in " << dir_ << ", /loc/map will be empty";
    return nullptr;
  }
  global_display_ = c;
  return global_display_;
}

}  // namespace loc
}  // namespace LI2Sup
