#ifndef PATH_SEARCHING_REFERENCE_HEIGHT_H_
#define PATH_SEARCHING_REFERENCE_HEIGHT_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <limits>
#include <vector>

// Height of the nearest segment of a world-frame reference route at an XY query.
// The caller handles endpoints separately so an A* search can still reach them.
inline double referenceHeightAtXY(const std::vector<Eigen::Vector3d> &path,
                                  const Eigen::Vector2d &xy)
{
  double best_dist2 = std::numeric_limits<double>::infinity();
  double height = path.front()(2);
  for (size_t i = 1; i < path.size(); ++i)
  {
    const Eigen::Vector2d from = path[i - 1].head<2>();
    const Eigen::Vector2d delta = path[i].head<2>() - from;
    const double len2 = delta.squaredNorm();
    const double ratio = len2 > 1e-10
        ? std::max(0.0, std::min(1.0, (xy - from).dot(delta) / len2)) : 0.0;
    const double dist2 = (xy - from - ratio * delta).squaredNorm();
    if (dist2 < best_dist2)
    {
      best_dist2 = dist2;
      height = path[i - 1](2) + ratio * (path[i](2) - path[i - 1](2));
    }
  }
  return height;
}

#endif
