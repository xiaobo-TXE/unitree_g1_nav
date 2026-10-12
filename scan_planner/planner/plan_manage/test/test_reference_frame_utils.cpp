#include <cmath>
#include <gtest/gtest.h>
#include <plan_manage/reference_frame_utils.h>

namespace
{
geometry_msgs::msg::TransformStamped transform(double x, double y, double yaw)
{
  geometry_msgs::msg::TransformStamped result;
  result.transform.translation.x = x;
  result.transform.translation.y = y;
  result.transform.rotation.z = std::sin(yaw / 2.0);
  result.transform.rotation.w = std::cos(yaw / 2.0);
  return result;
}

TEST(ReferenceFrameUtils, ReprojectsPositionAndRotatesVelocityWithoutTranslation)
{
  const Eigen::Vector3d target_map(2.0, 0.0, 0.5);
  const Eigen::Vector3d velocity_map(1.0, 0.0, 0.0);
  const auto first = scan_planner::referenceToWorld(transform(1.0, 0.0, 0.0));
  const auto second = scan_planner::referenceToWorld(transform(0.0, 1.0, M_PI / 2.0));

  EXPECT_NEAR((first * target_map).x(), 3.0, 1e-9);
  EXPECT_NEAR((second * target_map).x(), 0.0, 1e-9);
  EXPECT_NEAR((second * target_map).y(), 3.0, 1e-9);
  const Eigen::Vector3d velocity_world = second.linear() * velocity_map;
  EXPECT_NEAR(velocity_world.x(), 0.0, 1e-9);
  EXPECT_NEAR(velocity_world.y(), 1.0, 1e-9);
  EXPECT_NEAR(velocity_world.z(), 0.0, 1e-9);

  const Eigen::Vector3d robot_world(0.0, 2.0, 0.5);
  const Eigen::Vector3d robot_map = second.inverse() * robot_world;
  EXPECT_NEAR(robot_map.x(), 1.0, 1e-9);
  EXPECT_NEAR(robot_map.y(), 0.0, 1e-9);
}

TEST(ReferenceFrameUtils, DetectsAccumulatedTfChanges)
{
  const auto base = scan_planner::referenceToWorld(transform(0.0, 0.0, 0.0));
  const Eigen::Vector3d robot_world(1.0, 0.0, 0.0);
  EXPECT_FALSE(scan_planner::referenceTransformJumped(
      base, scan_planner::referenceToWorld(transform(0.19, 0.0, 0.0)),
      robot_world, 0.2, 5.0 * M_PI / 180.0));
  EXPECT_TRUE(scan_planner::referenceTransformJumped(
      base, scan_planner::referenceToWorld(transform(0.21, 0.0, 0.0)),
      robot_world, 0.2, 5.0 * M_PI / 180.0));
  EXPECT_TRUE(scan_planner::referenceTransformJumped(
      base, scan_planner::referenceToWorld(transform(0.0, 0.0, 6.0 * M_PI / 180.0)),
      Eigen::Vector3d::Zero(), 0.2, 5.0 * M_PI / 180.0));
}

TEST(ReferenceFrameUtils, FreshnessAndGracePolicy)
{
  EXPECT_TRUE(scan_planner::referenceTransformFresh(0.1, 0.3));
  EXPECT_FALSE(scan_planner::referenceTransformFresh(0.31, 0.3));
  EXPECT_FALSE(scan_planner::referenceTransformFresh(-0.1, 0.3));
  EXPECT_FALSE(scan_planner::referenceTfGraceExpired(0.49, 0.5));
  EXPECT_TRUE(scan_planner::referenceTfGraceExpired(0.5, 0.5));
  // A fresh transform resets the missing interval in the FSM; the next loss
  // starts a new grace period rather than inheriting the previous one.
  EXPECT_FALSE(scan_planner::referenceTfGraceExpired(0.0, 0.5));
}
}  // namespace
