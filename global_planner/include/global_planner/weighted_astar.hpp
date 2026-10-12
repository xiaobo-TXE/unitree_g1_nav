// Copyright 2026 jiewang

#ifndef GLOBAL_PLANNER__WEIGHTED_ASTAR_HPP_
#define GLOBAL_PLANNER__WEIGHTED_ASTAR_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "global_planner/terrain_map.hpp"

namespace global_planner
{

struct PlannerConfig
{
  int neighbor_cell_radius{1};
  double max_neighbor_distance{0.45};
  double max_step_height{0.28};
  double max_slope_degrees{55.0};
  double traversability_weight{2.0};
  double slope_weight{0.20};
  double step_weight{1.0};
  double heuristic_weight{1.15};
  double planning_timeout{2.0};
  bool enable_connectivity_precheck{true};
  bool enable_path_smoothing{true};
  double smoothing_sample_step{0.10};
  double smoothing_xy_tolerance{0.16};
  double smoothing_height_tolerance{0.10};
  double smoothing_max_segment_length{5.0};
};

struct PlanResult
{
  bool success{false};
  bool canceled{false};
  std::string message;
  std::vector<uint32_t> node_ids;
  std::size_t raw_node_count{0};
  std::size_t expanded_nodes{0};
  double planning_time_ms{0.0};
  double path_length{0.0};
};

class WeightedAStar
{
public:
  explicit WeightedAStar(const TerrainMap & map, PlannerConfig config = {});

  PlanResult plan(
    uint32_t start_id, uint32_t goal_id,
    const std::atomic<uint64_t> * generation = nullptr,
    uint64_t expected_generation = 0);

  std::size_t componentCount() const {return component_count_;}
  std::size_t largestComponentSize() const {return largest_component_size_;}
  double connectivityBuildTimeMs() const {return connectivity_build_time_ms_;}

private:
  struct QueueEntry
  {
    float f;
    float g;
    uint32_t id;
  };

  struct QueueGreater
  {
    bool operator()(const QueueEntry & lhs, const QueueEntry & rhs) const
    {
      return lhs.f > rhs.f;
    }
  };

  double heuristic(uint32_t from, uint32_t to) const;
  bool traversableEdge(uint32_t from, uint32_t to, double * distance, double * slope) const;
  double edgeCost(uint32_t from, uint32_t to, double distance, double slope) const;
  bool hasLineOfSight(uint32_t from, uint32_t to) const;
  std::vector<uint32_t> smoothPath(const std::vector<uint32_t> & path) const;
  void buildConnectedComponents();
  void touch(uint32_t id);

  const TerrainMap & map_;
  PlannerConfig config_;
  std::vector<float> g_score_;
  std::vector<int32_t> parent_;
  std::vector<uint32_t> stamp_;
  std::vector<uint32_t> closed_stamp_;
  std::vector<uint32_t> component_id_;
  uint32_t search_stamp_{0};
  std::size_t component_count_{0};
  std::size_t largest_component_size_{0};
  double connectivity_build_time_ms_{0.0};
};

}  // namespace global_planner

#endif  // GLOBAL_PLANNER__WEIGHTED_ASTAR_HPP_
