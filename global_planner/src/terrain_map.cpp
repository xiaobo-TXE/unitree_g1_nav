// Copyright 2026 jiewang

#include "global_planner/terrain_map.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace global_planner
{
namespace
{

struct VoxelKey
{
  int32_t x;
  int32_t y;
  int32_t z;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    uint64_t h = static_cast<uint32_t>(key.x);
    h = (h * 0x9e3779b185ebca87ULL) ^ static_cast<uint32_t>(key.y);
    h = (h * 0x9e3779b185ebca87ULL) ^ static_cast<uint32_t>(key.z);
    return static_cast<std::size_t>(h ^ (h >> 32U));
  }
};

struct Accumulator
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double traversability{0.0};
  uint32_t count{0};
  uint32_t traversable_count{0};
};

struct Candidate
{
  TerrainNode node;
  int32_t iz{0};
};

}  // namespace

TerrainMap::TerrainMap(TerrainMapConfig config)
: config_(std::move(config))
{
}

bool TerrainMap::loadPcd(const std::string & path, std::string * error)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(path, cloud) < 0) {
    if (error != nullptr) {
      *error = "PCL failed to read PCD file: " + path;
    }
    return false;
  }

  std::vector<TerrainPoint> points;
  points.reserve(cloud.size());
  for (const auto & point : cloud.points) {
    points.push_back({point.x, point.y, point.z, point.intensity});
  }
  // Do not keep PCL's aligned point buffer while the voxel hash is built.
  pcl::PointCloud<pcl::PointXYZI>().swap(cloud);
  return buildFromPoints(points, error);
}

bool TerrainMap::buildFromPoints(
  const std::vector<TerrainPoint> & points, std::string * error)
{
  raw_points_ = points;
  nodes_.clear();
  columns_.clear();
  raw_point_count_ = points.size();
  traversable_point_count_ = 0;
  inflated_node_count_ = 0;

  if (config_.resolution_xy <= 0.0 || config_.resolution_z <= 0.0) {
    if (error != nullptr) {
      *error = "voxel resolutions must be positive";
    }
    return false;
  }
  if (config_.min_traversable_fraction < 0.0 || config_.min_traversable_fraction > 1.0) {
    if (error != nullptr) {
      *error = "min_traversable_fraction must be in [0, 1]";
    }
    return false;
  }
  if (config_.normalized_traversability_threshold < 0.0 ||
    config_.normalized_traversability_threshold > 1.0)
  {
    if (error != nullptr) {
      *error = "normalized_traversability_threshold must be in [0, 1]";
    }
    return false;
  }
  if (config_.intensity_normalization_mode == IntensityNormalizationMode::kLegacy255 &&
    (config_.min_traversability < 0.0 || config_.min_traversability > 255.0))
  {
    if (error != nullptr) {
      *error = "min_traversability must be in [0, 255] in legacy_255 mode";
    }
    return false;
  }
  if (config_.layer_merge_height < 0.0 || config_.robot_radius < 0.0 ||
    config_.inflation_vertical_tolerance < 0.0 ||
    config_.min_inflation_obstacle_fraction < 0.0 ||
    config_.min_inflation_obstacle_fraction > 1.0)
  {
    if (error != nullptr) {
      *error = "invalid layer merge or obstacle inflation parameters";
    }
    return false;
  }

  bool found_finite_intensity = false;
  raw_intensity_min_ = std::numeric_limits<double>::infinity();
  raw_intensity_max_ = -std::numeric_limits<double>::infinity();
  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
      !std::isfinite(point.z) || !std::isfinite(point.intensity))
    {
      continue;
    }
    found_finite_intensity = true;
    raw_intensity_min_ = std::min(raw_intensity_min_, static_cast<double>(point.intensity));
    raw_intensity_max_ = std::max(raw_intensity_max_, static_cast<double>(point.intensity));
  }
  if (!found_finite_intensity) {
    if (error != nullptr) {
      *error = "point cloud contains no finite XYZ/intensity samples";
    }
    return false;
  }

  switch (config_.intensity_normalization_mode) {
    case IntensityNormalizationMode::kAutoMinMax:
      effective_intensity_min_ = raw_intensity_min_;
      effective_intensity_max_ = raw_intensity_max_;
      effective_threshold_ = config_.normalized_traversability_threshold;
      break;
    case IntensityNormalizationMode::kManualMinMax:
      effective_intensity_min_ = config_.intensity_input_min;
      effective_intensity_max_ = config_.intensity_input_max;
      effective_threshold_ = config_.normalized_traversability_threshold;
      break;
    case IntensityNormalizationMode::kLegacy255:
      effective_intensity_min_ = 0.0;
      effective_intensity_max_ = 255.0;
      effective_threshold_ = config_.min_traversability / 255.0;
      break;
  }
  if (!std::isfinite(effective_intensity_min_) ||
    !std::isfinite(effective_intensity_max_) ||
    effective_intensity_max_ <= effective_intensity_min_)
  {
    if (error != nullptr) {
      *error = config_.intensity_normalization_mode == IntensityNormalizationMode::kAutoMinMax ?
        "auto_minmax requires at least two distinct finite intensity values; use manual_minmax" :
        "intensity_input_max must be greater than intensity_input_min";
    }
    return false;
  }

  std::unordered_map<VoxelKey, Accumulator, VoxelKeyHash> voxels;
  voxels.reserve(points.size() / 2U + 1U);

  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
      !std::isfinite(point.z) || !std::isfinite(point.intensity))
    {
      continue;
    }

    double traversability = std::clamp(
      (static_cast<double>(point.intensity) - effective_intensity_min_) /
      (effective_intensity_max_ - effective_intensity_min_), 0.0, 1.0);
    if (config_.intensity_semantics == IntensitySemantics::kCost) {
      traversability = 1.0 - traversability;
    }
    const bool is_traversable = traversability >= effective_threshold_;
    if (is_traversable) {
      ++traversable_point_count_;
    }

    const VoxelKey key{
      static_cast<int32_t>(std::floor(point.x / config_.resolution_xy)),
      static_cast<int32_t>(std::floor(point.y / config_.resolution_xy)),
      static_cast<int32_t>(std::floor(point.z / config_.resolution_z))};
    auto & accumulator = voxels[key];
    accumulator.x += point.x;
    accumulator.y += point.y;
    accumulator.z += point.z;
    accumulator.traversability += traversability;
    ++accumulator.count;
    accumulator.traversable_count += is_traversable ? 1U : 0U;
  }

  std::vector<Candidate> candidates;
  candidates.reserve(voxels.size());
  for (const auto & item : voxels) {
    const auto & key = item.first;
    const auto & value = item.second;
    const double fraction = static_cast<double>(value.traversable_count) / value.count;
    if (fraction + 1e-9 < config_.min_traversable_fraction) {
      continue;
    }
    const double inverse_count = 1.0 / value.count;
    candidates.push_back(
      {
        TerrainNode{
          static_cast<float>(value.x * inverse_count),
          static_cast<float>(value.y * inverse_count),
          static_cast<float>(value.z * inverse_count),
          static_cast<float>(value.traversability * inverse_count), key.x, key.y},
        key.z});
  }

  if (config_.robot_radius > 0.0 && config_.min_inflation_obstacle_fraction > 0.0) {
    const int xy_cells = static_cast<int>(
      std::ceil(config_.robot_radius / config_.resolution_xy));
    const int z_cells = static_cast<int>(
      std::ceil(config_.inflation_vertical_tolerance / config_.resolution_z));
    const double radius_squared = config_.robot_radius * config_.robot_radius;

    candidates.erase(
      std::remove_if(
        candidates.begin(), candidates.end(),
        [&](const Candidate & candidate) {
          for (int dx = -xy_cells; dx <= xy_cells; ++dx) {
            for (int dy = -xy_cells; dy <= xy_cells; ++dy) {
              for (int dz = -z_cells; dz <= z_cells; ++dz) {
                const auto obstacle = voxels.find(
                  VoxelKey{candidate.node.ix + dx, candidate.node.iy + dy, candidate.iz + dz});
                if (obstacle == voxels.end()) {
                  continue;
                }
                const auto & value = obstacle->second;
                const double obstacle_fraction = 1.0 -
                static_cast<double>(value.traversable_count) / value.count;
                if (obstacle_fraction + 1e-9 < config_.min_inflation_obstacle_fraction) {
                  continue;
                }
                const double obstacle_x = value.x / value.count;
                const double obstacle_y = value.y / value.count;
                const double obstacle_z = value.z / value.count;
                const double delta_x = obstacle_x - candidate.node.x;
                const double delta_y = obstacle_y - candidate.node.y;
                if (delta_x * delta_x + delta_y * delta_y <= radius_squared &&
                std::abs(obstacle_z - candidate.node.z) <=
                config_.inflation_vertical_tolerance)
                {
                  ++inflated_node_count_;
                  return true;
                }
              }
            }
          }
          return false;
        }),
      candidates.end());
  }
  voxels.clear();
  voxels.rehash(0U);

  std::sort(
    candidates.begin(), candidates.end(),
    [](const Candidate & lhs, const Candidate & rhs) {
      if (lhs.node.ix != rhs.node.ix) {return lhs.node.ix < rhs.node.ix;}
      if (lhs.node.iy != rhs.node.iy) {return lhs.node.iy < rhs.node.iy;}
      return lhs.node.z < rhs.node.z;
    });

  nodes_.reserve(candidates.size());
  for (const auto & candidate : candidates) {
    if (!nodes_.empty()) {
      auto & previous = nodes_.back();
      if (previous.ix == candidate.node.ix && previous.iy == candidate.node.iy &&
        std::abs(previous.z - candidate.node.z) <= config_.layer_merge_height)
      {
        // Merge nearby vertical voxels without blending separate terrain layers.
        // Equal weighting prevents a dense voxel from dominating its neighbor.
        previous.x = 0.5F * (previous.x + candidate.node.x);
        previous.y = 0.5F * (previous.y + candidate.node.y);
        previous.z = 0.5F * (previous.z + candidate.node.z);
        previous.traversability =
          std::min(previous.traversability, candidate.node.traversability);
        continue;
      }
    }
    nodes_.push_back(candidate.node);
  }

  if (nodes_.empty()) {
    if (error != nullptr) {
      *error = "no traversable terrain nodes remained after voxelization";
    }
    return false;
  }

  columns_.reserve(nodes_.size());
  uint32_t begin = 0U;
  while (begin < nodes_.size()) {
    uint32_t end = begin + 1U;
    while (end < nodes_.size() && nodes_[end].ix == nodes_[begin].ix &&
      nodes_[end].iy == nodes_[begin].iy)
    {
      ++end;
    }
    columns_.emplace(columnKey(nodes_[begin].ix, nodes_[begin].iy), NodeRange{begin, end});
    begin = end;
  }
  return true;
}

const NodeRange * TerrainMap::findColumn(int32_t ix, int32_t iy) const
{
  const auto found = columns_.find(columnKey(ix, iy));
  return found == columns_.end() ? nullptr : &found->second;
}

int32_t TerrainMap::gridX(double x) const
{
  return static_cast<int32_t>(std::floor(x / config_.resolution_xy));
}

int32_t TerrainMap::gridY(double y) const
{
  return static_cast<int32_t>(std::floor(y / config_.resolution_xy));
}

int32_t TerrainMap::nearestNode(
  double x, double y, double z, double radius, double vertical_tolerance) const
{
  if (radius < 0.0 || vertical_tolerance < 0.0) {
    return -1;
  }
  const int cell_radius = static_cast<int>(std::ceil(radius / config_.resolution_xy));
  const int32_t center_x = gridX(x);
  const int32_t center_y = gridY(y);
  const double radius_squared = radius * radius;
  double best_score = std::numeric_limits<double>::infinity();
  int32_t best_id = -1;

  for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
    for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
      const NodeRange * range = findColumn(center_x + dx, center_y + dy);
      if (range == nullptr) {
        continue;
      }
      for (uint32_t id = range->begin; id < range->end; ++id) {
        const auto & node = nodes_[id];
        const double delta_x = node.x - x;
        const double delta_y = node.y - y;
        const double horizontal_squared = delta_x * delta_x + delta_y * delta_y;
        const double delta_z = std::abs(node.z - z);
        if (horizontal_squared > radius_squared || delta_z > vertical_tolerance) {
          continue;
        }
        const double score = horizontal_squared + delta_z * delta_z;
        if (score < best_score) {
          best_score = score;
          best_id = static_cast<int32_t>(id);
        }
      }
    }
  }
  return best_id;
}

uint64_t TerrainMap::columnKey(int32_t ix, int32_t iy)
{
  return (static_cast<uint64_t>(static_cast<uint32_t>(ix)) << 32U) |
         static_cast<uint32_t>(iy);
}

}  // namespace global_planner
