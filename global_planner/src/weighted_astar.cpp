// Copyright 2026 jiewang

#include "global_planner/weighted_astar.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace global_planner
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}

WeightedAStar::WeightedAStar(const TerrainMap & map, PlannerConfig config)
: map_(map), config_(std::move(config)),
  g_score_(map.nodes().size()), parent_(map.nodes().size()), stamp_(map.nodes().size(), 0U),
  closed_stamp_(map.nodes().size(), 0U),
  component_id_(map.nodes().size(), std::numeric_limits<uint32_t>::max())
{
  if (config_.enable_connectivity_precheck) {
    buildConnectedComponents();
  } else if (!map.nodes().empty()) {
    std::fill(component_id_.begin(), component_id_.end(), 0U);
    component_count_ = 1U;
    largest_component_size_ = map.nodes().size();
  }
}

PlanResult WeightedAStar::plan(
  uint32_t start_id, uint32_t goal_id, const std::atomic<uint64_t> * generation,
  uint64_t expected_generation)
{
  PlanResult result;
  const auto begin_time = std::chrono::steady_clock::now();
  const auto & nodes = map_.nodes();
  if (start_id >= nodes.size() || goal_id >= nodes.size()) {
    result.message = "start or goal node id is out of range";
    return result;
  }
  if (config_.enable_connectivity_precheck && component_id_[start_id] != component_id_[goal_id]) {
    result.message = "start and goal are in different traversable components";
    result.planning_time_ms = 1000.0 * std::chrono::duration<double>(
      std::chrono::steady_clock::now() - begin_time).count();
    return result;
  }

  if (++search_stamp_ == 0U) {
    std::fill(stamp_.begin(), stamp_.end(), 0U);
    std::fill(closed_stamp_.begin(), closed_stamp_.end(), 0U);
    search_stamp_ = 1U;
  }

  std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueGreater> open;
  touch(start_id);
  g_score_[start_id] = 0.0F;
  open.push(
    {static_cast<float>(config_.heuristic_weight * heuristic(start_id, goal_id)), 0.0F,
      start_id});

  bool found = false;
  while (!open.empty()) {
    const QueueEntry current = open.top();
    open.pop();
    if (stamp_[current.id] != search_stamp_ || current.g > g_score_[current.id] + 1e-5F) {
      continue;
    }
    if (closed_stamp_[current.id] == search_stamp_) {
      continue;
    }
    closed_stamp_[current.id] = search_stamp_;
    if (current.id == goal_id) {
      found = true;
      break;
    }
    ++result.expanded_nodes;

    if ((result.expanded_nodes & 0xFFU) == 0U) {
      if (generation != nullptr &&
        generation->load(std::memory_order_relaxed) != expected_generation)
      {
        result.canceled = true;
        result.message = "superseded by a newer planning request";
        break;
      }
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - begin_time).count();
      if (config_.planning_timeout > 0.0 && elapsed > config_.planning_timeout) {
        result.message = "planning timeout";
        break;
      }
    }

    const auto & current_node = nodes[current.id];
    for (int dx = -config_.neighbor_cell_radius; dx <= config_.neighbor_cell_radius; ++dx) {
      for (int dy = -config_.neighbor_cell_radius; dy <= config_.neighbor_cell_radius; ++dy) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const NodeRange * range = map_.findColumn(current_node.ix + dx, current_node.iy + dy);
        if (range == nullptr) {
          continue;
        }
        for (uint32_t neighbor_id = range->begin; neighbor_id < range->end; ++neighbor_id) {
          if (closed_stamp_[neighbor_id] == search_stamp_) {
            continue;
          }
          double distance = 0.0;
          double slope = 0.0;
          if (!traversableEdge(current.id, neighbor_id, &distance, &slope)) {
            continue;
          }
          const float tentative = static_cast<float>(
            current.g + edgeCost(current.id, neighbor_id, distance, slope));
          touch(neighbor_id);
          if (tentative + 1e-5F >= g_score_[neighbor_id]) {
            continue;
          }
          g_score_[neighbor_id] = tentative;
          parent_[neighbor_id] = static_cast<int32_t>(current.id);
          const float f = static_cast<float>(
            tentative + config_.heuristic_weight * heuristic(neighbor_id, goal_id));
          open.push({f, tentative, neighbor_id});
        }
      }
    }
  }

  if (found) {
    for (int32_t id = static_cast<int32_t>(goal_id); id >= 0; id = parent_[id]) {
      result.node_ids.push_back(static_cast<uint32_t>(id));
      if (static_cast<uint32_t>(id) == start_id) {
        break;
      }
    }
    if (result.node_ids.empty() || result.node_ids.back() != start_id) {
      result.node_ids.clear();
      result.message = "invalid parent chain";
    } else {
      std::reverse(result.node_ids.begin(), result.node_ids.end());
      result.raw_node_count = result.node_ids.size();
      if (config_.enable_path_smoothing && result.node_ids.size() > 2U) {
        result.node_ids = smoothPath(result.node_ids);
      }
      for (std::size_t i = 1; i < result.node_ids.size(); ++i) {
        const auto & a = nodes[result.node_ids[i - 1U]];
        const auto & b = nodes[result.node_ids[i]];
        result.path_length += std::hypot(std::hypot(b.x - a.x, b.y - a.y), b.z - a.z);
      }
      result.success = true;
      result.message = "path found";
    }
  } else if (result.message.empty()) {
    result.message = "no path found";
  }

  result.planning_time_ms = 1000.0 * std::chrono::duration<double>(
    std::chrono::steady_clock::now() - begin_time).count();
  return result;
}

bool WeightedAStar::hasLineOfSight(uint32_t from, uint32_t to) const
{
  if (from == to) {
    return true;
  }
  const auto & start = map_.nodes()[from];
  const auto & goal = map_.nodes()[to];
  const double dx = goal.x - start.x;
  const double dy = goal.y - start.y;
  const double dz = goal.z - start.z;
  const double horizontal = std::hypot(dx, dy);
  if (horizontal > config_.smoothing_max_segment_length ||
    config_.smoothing_sample_step <= 0.0)
  {
    return false;
  }

  const int sample_count = std::max(
    1, static_cast<int>(std::ceil(horizontal / config_.smoothing_sample_step)));
  uint32_t previous = from;
  for (int sample = 1; sample < sample_count; ++sample) {
    const double ratio = static_cast<double>(sample) / sample_count;
    const int32_t support = map_.nearestNode(
      start.x + ratio * dx, start.y + ratio * dy, start.z + ratio * dz,
      config_.smoothing_xy_tolerance, config_.smoothing_height_tolerance);
    if (support < 0) {
      return false;
    }
    const uint32_t support_id = static_cast<uint32_t>(support);
    if (support_id == previous) {
      continue;
    }
    double distance = 0.0;
    double slope = 0.0;
    if (!traversableEdge(previous, support_id, &distance, &slope)) {
      return false;
    }
    previous = support_id;
  }

  if (previous == to) {
    return true;
  }
  double distance = 0.0;
  double slope = 0.0;
  return traversableEdge(previous, to, &distance, &slope);
}

std::vector<uint32_t> WeightedAStar::smoothPath(const std::vector<uint32_t> & path) const
{
  std::vector<uint32_t> smoothed;
  smoothed.reserve(path.size());
  smoothed.push_back(path.front());

  std::size_t anchor = 0U;
  while (anchor + 1U < path.size()) {
    std::size_t upper = anchor + 1U;
    const auto & first_a = map_.nodes()[path[anchor]];
    const auto & first_b = map_.nodes()[path[upper]];
    double traversed = std::hypot(
      std::hypot(first_b.x - first_a.x, first_b.y - first_a.y), first_b.z - first_a.z);
    while (upper + 1U < path.size()) {
      const auto & a = map_.nodes()[path[upper]];
      const auto & b = map_.nodes()[path[upper + 1U]];
      const double step = std::hypot(std::hypot(b.x - a.x, b.y - a.y), b.z - a.z);
      if (traversed + step > config_.smoothing_max_segment_length) {
        break;
      }
      traversed += step;
      ++upper;
    }

    std::size_t selected = anchor + 1U;
    for (std::size_t candidate = upper; candidate > anchor + 1U; --candidate) {
      if (hasLineOfSight(path[anchor], path[candidate])) {
        selected = candidate;
        break;
      }
    }
    smoothed.push_back(path[selected]);
    anchor = selected;
  }
  return smoothed;
}

void WeightedAStar::buildConnectedComponents()
{
  const auto begin_time = std::chrono::steady_clock::now();
  const auto & nodes = map_.nodes();
  constexpr uint32_t kUnassigned = std::numeric_limits<uint32_t>::max();
  std::vector<uint32_t> queue;

  for (uint32_t seed = 0U; seed < nodes.size(); ++seed) {
    if (component_id_[seed] != kUnassigned) {
      continue;
    }
    const uint32_t component = static_cast<uint32_t>(component_count_++);
    component_id_[seed] = component;
    queue.clear();
    queue.push_back(seed);
    std::size_t head = 0U;

    while (head < queue.size()) {
      const uint32_t current_id = queue[head++];
      const auto & current = nodes[current_id];
      for (int dx = -config_.neighbor_cell_radius; dx <= config_.neighbor_cell_radius; ++dx) {
        for (int dy = -config_.neighbor_cell_radius; dy <= config_.neighbor_cell_radius; ++dy) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          const NodeRange * range = map_.findColumn(current.ix + dx, current.iy + dy);
          if (range == nullptr) {
            continue;
          }
          for (uint32_t neighbor = range->begin; neighbor < range->end; ++neighbor) {
            if (component_id_[neighbor] != kUnassigned) {
              continue;
            }
            double distance = 0.0;
            double slope = 0.0;
            if (!traversableEdge(current_id, neighbor, &distance, &slope)) {
              continue;
            }
            component_id_[neighbor] = component;
            queue.push_back(neighbor);
          }
        }
      }
    }
    largest_component_size_ = std::max(largest_component_size_, queue.size());
  }

  connectivity_build_time_ms_ = 1000.0 * std::chrono::duration<double>(
    std::chrono::steady_clock::now() - begin_time).count();
}

double WeightedAStar::heuristic(uint32_t from, uint32_t to) const
{
  const auto & a = map_.nodes()[from];
  const auto & b = map_.nodes()[to];
  return std::hypot(std::hypot(b.x - a.x, b.y - a.y), b.z - a.z);
}

bool WeightedAStar::traversableEdge(
  uint32_t from, uint32_t to, double * distance, double * slope) const
{
  const auto & a = map_.nodes()[from];
  const auto & b = map_.nodes()[to];
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double dz = b.z - a.z;
  const double horizontal = std::hypot(dx, dy);
  if (horizontal < 1e-4 || horizontal > config_.max_neighbor_distance ||
    std::abs(dz) > config_.max_step_height)
  {
    return false;
  }
  *slope = std::atan2(std::abs(dz), horizontal);
  if (*slope * 180.0 / kPi > config_.max_slope_degrees) {
    return false;
  }
  *distance = std::hypot(horizontal, dz);
  return true;
}

double WeightedAStar::edgeCost(
  uint32_t from, uint32_t to, double distance, double slope) const
{
  const auto & node = map_.nodes()[to];
  const double normalized_risk = std::clamp(
    1.0 - static_cast<double>(node.traversability), 0.0, 1.0);
  return distance * (1.0 + config_.traversability_weight * normalized_risk) +
         config_.slope_weight * slope +
         config_.step_weight * std::abs(node.z - map_.nodes()[from].z);
}

void WeightedAStar::touch(uint32_t id)
{
  if (stamp_[id] == search_stamp_) {
    return;
  }
  stamp_[id] = search_stamp_;
  g_score_[id] = std::numeric_limits<float>::infinity();
  parent_[id] = -1;
}

}  // namespace global_planner
