/**
 * @file keyframe_map.h
 * @brief Read-only access to a loc_map/ directory: keyframe poses, 3D retrieval
 *        and on-demand cloud loading behind an LRU cache.
 *
 * Interface contract:
 *  - load() must succeed before any other call; it reads meta.yaml and
 *    keyframes.txt only, never the clouds.
 *  - queryNear()/kf()/nearestId()/size() are const and thread safe once loaded.
 *  - cloud()/prefetch() mutate the LRU cache and are guarded by an internal
 *    mutex, but are meant to be called from the backend thread (T1) only:
 *    they can block on disk for a few milliseconds.
 *  - Returned clouds are shared_ptr<const>; the cache may evict its own handle
 *    at any time but the caller's copy stays valid.
 */
#ifndef LOC_KEYFRAME_MAP_H_
#define LOC_KEYFRAME_MAP_H_

#include <list>
#include <mutex>
#include <atomic>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "basic/alias.h"
#include "basic/Manifold.h"

namespace LI2Sup {
namespace loc {

using CloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

/// One map keyframe. `cloud` lives on disk in the keyframe's own body frame.
struct Keyframe {
  int         id = -1;
  double      stamp = 0.0;
  BASIC::SE3  T_M_B;            ///< keyframe body pose in the map frame
  Eigen::Vector3f pos = Eigen::Vector3f::Zero();  ///< == T_M_B.t_, retrieval key
  std::string path;             ///< absolute path to kf/xxxxxx.pcd
  int         n_points = 0;
  int         src_begin = 0, src_end = 0;  ///< raw frame range absorbed, for tracing
};

/// What meta.yaml records about how the map was built.
struct MapMeta {
  int         format_version = 0;
  std::string source_session;
  int         source_frames = 0;
  float       kf_dist = 0, kf_angle = 0, kf_range = 0, kf_voxel = 0, global_voxel = 0;
  int         num_keyframes = 0;
  long long   total_points = 0;
  BASIC::V3   gravity = BASIC::V3::Zero();
  BASIC::SE3  T_B_L;            ///< lidar->imu extrinsic used when building
  bool        has_extrinsic = false;
};

/// Fixed-capacity least-recently-used cache. Not thread safe by itself;
/// KeyframeMap owns the lock.
template <typename K, typename V>
class LruCache {
public:
  explicit LruCache(std::size_t capacity) : capacity_(capacity) {}

  /// Returns a default-constructed V if absent, and promotes on hit.
  V get(const K& key) {
    auto it = index_.find(key);
    if (it == index_.end()) return V{};
    order_.splice(order_.begin(), order_, it->second);
    return it->second->second;
  }

  void put(const K& key, V value) {
    auto it = index_.find(key);
    if (it != index_.end()) {
      it->second->second = std::move(value);
      order_.splice(order_.begin(), order_, it->second);
      return;
    }
    order_.emplace_front(key, std::move(value));
    index_[key] = order_.begin();
    while (capacity_ > 0 && index_.size() > capacity_) {
      index_.erase(order_.back().first);
      order_.pop_back();
    }
  }

  bool contains(const K& key) const { return index_.count(key) != 0; }
  std::size_t size() const { return index_.size(); }
  void clear() { order_.clear(); index_.clear(); }
  void setCapacity(std::size_t c) { capacity_ = c; }

private:
  using Entry = std::pair<K, V>;
  std::size_t capacity_;
  std::list<Entry> order_;                                        // front == most recent
  std::unordered_map<K, typename std::list<Entry>::iterator> index_;
};

class KeyframeMap {
public:
  using Ptr = std::shared_ptr<KeyframeMap>;
  // PCL 1.10 (Ubuntu 20.04 / Foxy) hands out boost::shared_ptr cloud handles,
  // 1.11+ uses std::shared_ptr. Follow PCL's own typedef so both build.
  using CloudConstPtr = CloudXYZ::ConstPtr;

  KeyframeMap() : cache_(300) {}

  /// Reads meta.yaml + keyframes.txt and builds the retrieval tree.
  /// `preload_all` also pulls every cloud into memory (hundreds of MB).
  bool load(const std::string& dir, bool preload_all = false, int lru_capacity = 300);

  bool loaded() const { return loaded_; }
  std::size_t size() const { return kfs_.size(); }
  const MapMeta& meta() const { return meta_; }
  const std::vector<Keyframe>& keyframes() const { return kfs_; }
  const Keyframe& kf(int id) const { return kfs_.at(id); }

  /// Keyframe ids within `radius` of `p`, nearest first, at most `max_k`.
  /// 3D on purpose: this map has 6 m of relief, a 2D query crosses levels.
  std::vector<int> queryNear(const Eigen::Vector3f& p, float radius, int max_k) const;

  /// Nearest keyframe id, or -1 when the map is empty.
  int nearestId(const Eigen::Vector3f& p) const;

  /// Nearest keyframe ignoring z. This is the one to use when establishing an
  /// initial z: a 3D query needs the very z it is being asked to supply, so a
  /// bad input z selects a keyframe on another level and then confirms it.
  int nearestIdXY(const Eigen::Vector3f& p) const;

  /// Cloud in the keyframe's own body frame. Cache hit, or a disk read.
  /// Returns nullptr when the file is missing or the id is out of range.
  CloudConstPtr cloud(int id);

  /// Warm the cache. Ignores ids already resident.
  void prefetch(const std::vector<int>& ids);

  /// Diagnostics.
  std::size_t cacheSize() const;

  /// How the LRU is doing. On the board this is a real risk that no topic
  /// shows: a miss is a synchronous pcd read on the backend thread, and flash
  /// there is far slower than the development machine's SSD. A hit rate that
  /// falls with a growing disk time is the signature of an lru_capacity too
  /// small for the route.
  struct CacheStats {
    std::size_t hits = 0;
    std::size_t misses = 0;
    std::size_t prefetched = 0;   ///< reads done ahead of need, not on the critical path
    double      disk_read_ms = 0.0;   ///< misses AND prefetches
    double      disk_read_max_ms = 0.0;
  };
  CacheStats cacheStats() const;
  BASIC::V3 boundsMin() const { return bound_min_; }
  BASIC::V3 boundsMax() const { return bound_max_; }

  /// Global display cloud (global_ds.pcd), loaded on demand. May be null.
  CloudConstPtr globalDisplayCloud();

private:
  bool parseMeta(const std::string& path);
  bool parseKeyframes(const std::string& path, const std::string& dir);
  CloudConstPtr readFromDisk(int id) const;

  std::vector<Keyframe> kfs_;
  MapMeta     meta_;
  std::string dir_;
  bool        loaded_ = false;
  bool        preload_all_ = false;

  pcl::KdTreeFLANN<pcl::PointXYZ> tree_;
  CloudXYZ::Ptr tree_cloud_;

  BASIC::V3 bound_min_ = BASIC::V3::Zero(), bound_max_ = BASIC::V3::Zero();

  mutable std::mutex cache_mutex_;
  mutable LruCache<int, CloudConstPtr> cache_;

  /// Atomic so cloud() stays lock-free outside the cache mutex; microseconds
  /// so the accumulator can be an integer.
  mutable std::atomic<std::size_t>   cache_hits_{0};
  mutable std::atomic<std::size_t>   cache_misses_{0};
  mutable std::atomic<std::size_t>   prefetched_{0};
  mutable std::atomic<std::uint64_t> disk_us_{0};
  mutable std::atomic<std::uint64_t> disk_us_max_{0};
  CloudConstPtr global_display_;
};

}  // namespace loc
}  // namespace LI2Sup

#endif  // LOC_KEYFRAME_MAP_H_
