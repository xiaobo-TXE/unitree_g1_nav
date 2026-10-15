#ifndef PLAN_MANAGE_YAW_TRAJECTORY_H
#define PLAN_MANAGE_YAW_TRAJECTORY_H

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Eigen>

namespace scan_planner
{

// Independent heading (yaw) trajectory: a scalar uniform B-spline planned as a
// 7th degree of freedom. It follows the travel direction softly and blends to
// the requested goal yaw near the goal, with rate/acceleration limiting and an
// optional hard terminal constraint.
struct YawTrajectoryParams
{
  int order{3};                 // B-spline degree
  double knot_interval{0.2};    // target control-point spacing [s]
  double yaw_align_distance{1.0}; // metres to the goal over which the blend reaches 1
  double max_yaw_rate{0.8};     // rad/s
  double max_yaw_acc{1.5};      // rad/s^2
  int limit_iterations{10};
};

struct YawTrajectoryInput
{
  double duration{0.0};
  double initial_yaw{0.0};
  bool have_goal_yaw{false};
  Eigen::Vector2d goal_xy{Eigen::Vector2d::Zero()};
  double goal_yaw{0.0};
  bool pin_terminal{false};
};

inline double yawWrapAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

// position_at / velocity_at map a time in [0, duration] to an XY vector.
// On success fills yaw_pts (control points) and yaw_dt (knot span); the spline
// spans (yaw_pts.size() - order) * yaw_dt which equals the requested duration.
template <typename PositionAt, typename VelocityAt>
bool buildYawControlPoints(const YawTrajectoryInput &in,
                           const YawTrajectoryParams &params,
                           const PositionAt &position_at,
                           const VelocityAt &velocity_at,
                           std::vector<double> &yaw_pts,
                           double &yaw_dt)
{
  yaw_pts.clear();
  yaw_dt = 0.0;

  const int order = params.order;
  if (order < 1 || !std::isfinite(in.duration) || in.duration <= 1e-3)
    return false;

  int num_ctrl = static_cast<int>(std::ceil(in.duration / params.knot_interval)) + order;
  if (num_ctrl < order + 2)
    num_ctrl = order + 2;
  const double dt = in.duration / static_cast<double>(num_ctrl - order);
  if (!std::isfinite(dt) || dt <= 1e-3)
    return false;

  // 1) Soft target: follow the velocity direction, blended toward the goal yaw
  //    over the final yaw_align_distance metres.
  std::vector<double> yaw(static_cast<size_t>(num_ctrl), in.initial_yaw);
  double previous = in.initial_yaw;
  for (int i = 0; i < num_ctrl; ++i)
  {
    const double t = std::min(in.duration, std::max(0.0, (i - 0.5 * (order - 1)) * dt));
    const Eigen::Vector2d velocity = velocity_at(t);
    const double tangent =
        velocity.squaredNorm() > 1e-4 ? std::atan2(velocity.y(), velocity.x()) : previous;

    double weight = 0.0;
    if (in.have_goal_yaw && params.yaw_align_distance > 1e-6)
    {
      const double distance = (position_at(t) - in.goal_xy).norm();
      weight = std::clamp((params.yaw_align_distance - distance) / params.yaw_align_distance,
                          0.0, 1.0);
    }
    yaw[static_cast<size_t>(i)] = tangent + weight * yawWrapAngle(in.goal_yaw - tangent);
    previous = yaw[static_cast<size_t>(i)];
  }

  // 2) Rate/acceleration limiting in control-point space. For a uniform
  //    B-spline, |dP| <= rate*dt and |second difference| <= acc*dt^2. The
  //    backward rate pass runs last each iteration and is anchored at the
  //    terminal, so it holds the rate limit exactly and preserves a pinned
  //    terminal block.
  const double max_step = params.max_yaw_rate * dt;
  const double max_bend = params.max_yaw_acc * dt * dt;
  const int pin_begin = std::max(0, num_ctrl - 1 - order);
  const int bend_hi = in.pin_terminal ? pin_begin : num_ctrl - 1;
  for (int iteration = 0; iteration < params.limit_iterations; ++iteration)
  {
    if (in.pin_terminal)
      for (int i = pin_begin; i < num_ctrl; ++i)
        yaw[static_cast<size_t>(i)] = in.goal_yaw;

    for (int i = 1; i < bend_hi; ++i)
    {
      const double bend = yaw[static_cast<size_t>(i - 1)] - 2.0 * yaw[static_cast<size_t>(i)] +
                          yaw[static_cast<size_t>(i + 1)];
      if (std::abs(bend) > max_bend)
        yaw[static_cast<size_t>(i)] +=
            (std::abs(bend) - max_bend) * (bend > 0.0 ? 0.5 : -0.5);
    }

    for (int i = num_ctrl - 2; i >= 0; --i)
      yaw[static_cast<size_t>(i)] = std::clamp(
          yaw[static_cast<size_t>(i)], yaw[static_cast<size_t>(i + 1)] - max_step,
          yaw[static_cast<size_t>(i + 1)] + max_step);
  }

  yaw_dt = dt;
  yaw_pts = yaw;
  return true;
}

} // namespace scan_planner

#endif // PLAN_MANAGE_YAW_TRAJECTORY_H
