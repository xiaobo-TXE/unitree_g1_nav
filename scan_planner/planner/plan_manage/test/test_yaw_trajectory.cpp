#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>
#include <plan_manage/yaw_trajectory.h>

namespace
{
using scan_planner::buildYawControlPoints;
using scan_planner::YawTrajectoryInput;
using scan_planner::YawTrajectoryParams;
using scan_planner::yawWrapAngle;

constexpr double kPi = 3.14159265358979323846;

// Straight path along +y at constant speed (tangent = +pi/2).
Eigen::Vector2d straightPosition(double t) { return Eigen::Vector2d(0.0, 2.0 * t); }
Eigen::Vector2d straightVelocity(double) { return Eigen::Vector2d(0.0, 2.0); }

// Limiter effectively disabled, to isolate the blending logic.
YawTrajectoryParams unboundedParams()
{
  YawTrajectoryParams p;
  p.yaw_align_distance = 1.0;
  p.max_yaw_rate = 1e9;
  p.max_yaw_acc = 1e9;
  return p;
}

double maxBend(const std::vector<double> &v)
{
  double m = 0.0;
  for (size_t i = 1; i + 1 < v.size(); ++i)
    m = std::max(m, std::abs(v[i - 1] - 2.0 * v[i] + v[i + 1]));
  return m;
}
} // namespace

TEST(YawTrajectory, FollowsVelocityDirectionWhenNoGoalYaw)
{
  YawTrajectoryInput in;
  in.duration = 5.0;
  in.initial_yaw = 0.0;
  in.have_goal_yaw = false;

  std::vector<double> yaw_pts;
  double dt = 0.0;
  ASSERT_TRUE(buildYawControlPoints(in, unboundedParams(), straightPosition, straightVelocity,
                                    yaw_pts, dt));
  ASSERT_GE(yaw_pts.size(), 4u);
  for (double yaw : yaw_pts)
    EXPECT_NEAR(yaw, kPi / 2.0, 1e-9);
}

TEST(YawTrajectory, KnotSpanMatchesDuration)
{
  YawTrajectoryInput in;
  in.duration = 3.7;
  in.have_goal_yaw = false;

  YawTrajectoryParams p = unboundedParams();
  std::vector<double> yaw_pts;
  double dt = 0.0;
  ASSERT_TRUE(buildYawControlPoints(in, p, straightPosition, straightVelocity, yaw_pts, dt));

  const double span = static_cast<double>(yaw_pts.size() - p.order) * dt;
  EXPECT_NEAR(span, in.duration, 1e-9);
}

TEST(YawTrajectory, PinsTerminalControlPointsToGoalYaw)
{
  YawTrajectoryInput in;
  in.duration = 5.0;
  in.have_goal_yaw = true;
  in.goal_xy = Eigen::Vector2d(0.0, 10.0);
  in.goal_yaw = 0.0;
  in.pin_terminal = true;

  YawTrajectoryParams p = unboundedParams();
  std::vector<double> yaw_pts;
  double dt = 0.0;
  ASSERT_TRUE(buildYawControlPoints(in, p, straightPosition, straightVelocity, yaw_pts, dt));

  // The last (order + 1) control points define the spline endpoint.
  const size_t pin_begin = yaw_pts.size() - static_cast<size_t>(p.order) - 1;
  for (size_t i = pin_begin; i < yaw_pts.size(); ++i)
    EXPECT_NEAR(yaw_pts[i], in.goal_yaw, 1e-12);
  // Far from the goal the heading still follows the travel direction.
  EXPECT_NEAR(yaw_pts.front(), kPi / 2.0, 1e-9);
}

TEST(YawTrajectory, BlendsTowardGoalYawNearGoal)
{
  YawTrajectoryInput in;
  in.duration = 5.0;
  in.have_goal_yaw = true;
  in.goal_xy = Eigen::Vector2d(0.0, 10.0); // equal to the path end
  in.goal_yaw = 0.0;
  in.pin_terminal = false;

  YawTrajectoryParams p = unboundedParams();
  std::vector<double> yaw_pts;
  double dt = 0.0;
  ASSERT_TRUE(buildYawControlPoints(in, p, straightPosition, straightVelocity, yaw_pts, dt));

  EXPECT_NEAR(yaw_pts.front(), kPi / 2.0, 1e-9); // far from goal: tangent
  EXPECT_NEAR(yaw_pts.back(), 0.0, 1e-9);        // at the goal: fully blended
  EXPECT_LT(yaw_pts.back(), yaw_pts.front());
}

TEST(YawTrajectory, RespectsYawRateLimit)
{
  YawTrajectoryInput in;
  in.duration = 5.0;
  in.have_goal_yaw = false;

  YawTrajectoryParams p;
  p.order = 3;
  p.knot_interval = 0.2;
  p.max_yaw_rate = 0.8;
  p.max_yaw_acc = 1e9; // isolate the rate limiter

  // Velocity direction flips, so the raw tangent jumps from 0 to pi.
  auto position = [](double t) {
    return Eigen::Vector2d(t <= 2.5 ? t : 5.0 - t, 0.0);
  };
  auto velocity = [](double t) {
    return Eigen::Vector2d(t <= 2.5 ? 1.0 : -1.0, 0.0);
  };

  std::vector<double> yaw_pts;
  double dt = 0.0;
  ASSERT_TRUE(buildYawControlPoints(in, p, position, velocity, yaw_pts, dt));

  const double max_step = p.max_yaw_rate * dt;
  for (size_t i = 0; i + 1 < yaw_pts.size(); ++i)
    EXPECT_LE(std::abs(yawWrapAngle(yaw_pts[i + 1] - yaw_pts[i])), max_step + 1e-9)
        << "control point " << i;
}

TEST(YawTrajectory, AccelerationLimitingReducesCurvature)
{
  YawTrajectoryInput in;
  in.duration = 5.0;
  in.have_goal_yaw = false;

  auto position = [](double t) {
    return Eigen::Vector2d(t <= 2.5 ? t : 5.0 - t, 0.0);
  };
  auto velocity = [](double t) {
    return Eigen::Vector2d(t <= 2.5 ? 1.0 : -1.0, 0.0);
  };

  YawTrajectoryParams loose;
  loose.max_yaw_rate = 1e9;
  loose.max_yaw_acc = 1e9;
  YawTrajectoryParams tight;
  tight.max_yaw_rate = 1e9;
  tight.max_yaw_acc = 1.5;

  std::vector<double> loose_pts, tight_pts;
  double loose_dt = 0.0, tight_dt = 0.0;
  ASSERT_TRUE(buildYawControlPoints(in, loose, position, velocity, loose_pts, loose_dt));
  ASSERT_TRUE(buildYawControlPoints(in, tight, position, velocity, tight_pts, tight_dt));

  EXPECT_LT(maxBend(tight_pts), maxBend(loose_pts));
}

TEST(YawTrajectory, RejectsDegenerateInput)
{
  std::vector<double> yaw_pts;
  double dt = 0.0;

  YawTrajectoryInput zero_duration;
  zero_duration.duration = 0.0;
  EXPECT_FALSE(buildYawControlPoints(zero_duration, unboundedParams(), straightPosition,
                                     straightVelocity, yaw_pts, dt));
  EXPECT_TRUE(yaw_pts.empty());

  YawTrajectoryInput ok;
  ok.duration = 5.0;
  YawTrajectoryParams bad_order;
  bad_order.order = 0;
  EXPECT_FALSE(buildYawControlPoints(ok, bad_order, straightPosition, straightVelocity, yaw_pts, dt));

  YawTrajectoryInput not_finite;
  not_finite.duration = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(buildYawControlPoints(not_finite, unboundedParams(), straightPosition,
                                     straightVelocity, yaw_pts, dt));
}
