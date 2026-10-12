// Copyright 2026 jiewang

#ifndef GLOBAL_PLANNER__TERRAIN_MAP_HPP_
#define GLOBAL_PLANNER__TERRAIN_MAP_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace global_planner
{

enum class IntensityNormalizationMode
{
  kAutoMinMax,
  kManualMinMax,
  kLegacy255
};

enum class IntensitySemantics
{
  kTraversability,
  kCost
};

struct TerrainPoint
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
};

struct TerrainNode
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float traversability{0.0F};
  int32_t ix{0};
  int32_t iy{0};
};

struct TerrainMapConfig
{
  double resolution_xy{0.20};
  double resolution_z{0.10};
  double layer_merge_height{0.12};
  IntensityNormalizationMode intensity_normalization_mode{
    IntensityNormalizationMode::kAutoMinMax};
  IntensitySemantics intensity_semantics{IntensitySemantics::kTraversability};
  double intensity_input_min{0.0};
  double intensity_input_max{255.0};
  double normalized_traversability_threshold{0.50};
  double min_traversability{128.0};
  double min_traversable_fraction{0.60};
  double robot_radius{0.25};
  double inflation_vertical_tolerance{0.30};
  double min_inflation_obstacle_fraction{0.50};
};

struct NodeRange
{
  uint32_t begin{0};
  uint32_t end{0};
};

class TerrainMap
{
public:
  explicit TerrainMap(TerrainMapConfig config = {});

  bool loadPcd(const std::string & path, std::string * error = nullptr);
  bool buildFromPoints(const std::vector<TerrainPoint> & points, std::string * error = nullptr);

  const TerrainMapConfig & config() const {return config_;}
  const std::vector<TerrainPoint> & rawPoints() const {return raw_points_;}
  const std::vector<TerrainNode> & nodes() const {return nodes_;}
  std::size_t rawPointCount() const {return raw_point_count_;}
  std::size_t traversablePointCount() const {return traversable_point_count_;}
  std::size_t inflatedNodeCount() const {return inflated_node_count_;}
  double rawIntensityMin() const {return raw_intensity_min_;}
  double rawIntensityMax() const {return raw_intensity_max_;}
  double effectiveIntensityMin() const {return effective_intensity_min_;}
  double effectiveIntensityMax() const {return effective_intensity_max_;}
  double effectiveTraversabilityThreshold() const {return effective_threshold_;}
  const NodeRange * findColumn(int32_t ix, int32_t iy) const;

  int32_t gridX(double x) const;
  int32_t gridY(double y) const;

  int32_t nearestNode(
    double x, double y, double z, double radius,
    double vertical_tolerance) const;

private:
  static uint64_t columnKey(int32_t ix, int32_t iy);

  TerrainMapConfig config_;
  std::vector<TerrainPoint> raw_points_;
  std::vector<TerrainNode> nodes_;
  std::unordered_map<uint64_t, NodeRange> columns_;
  std::size_t raw_point_count_{0};
  std::size_t traversable_point_count_{0};
  std::size_t inflated_node_count_{0};
  double raw_intensity_min_{0.0};
  double raw_intensity_max_{0.0};
  double effective_intensity_min_{0.0};
  double effective_intensity_max_{1.0};
  double effective_threshold_{0.5};
};

}  // namespace global_planner

#endif  // GLOBAL_PLANNER__TERRAIN_MAP_HPP_
