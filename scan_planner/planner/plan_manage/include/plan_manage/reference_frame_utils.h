#ifndef PLAN_MANAGE_REFERENCE_FRAME_UTILS_H
#define PLAN_MANAGE_REFERENCE_FRAME_UTILS_H

#include <cmath>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace scan_planner
{
inline Eigen::Isometry3d referenceToWorld(
    const geometry_msgs::msg::TransformStamped &transform)
{
  const auto &q = transform.transform.rotation;
  Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = rotation.normalized().toRotationMatrix();
  result.translation() << transform.transform.translation.x,
      transform.transform.translation.y, transform.transform.translation.z;
  return result;
}

inline bool referenceTransformJumped(const Eigen::Isometry3d &previous,
                                      const Eigen::Isometry3d &current,
                                      const Eigen::Vector3d &robot_world,
                                      double max_translation, double max_yaw)
{
  const double translation =
      (previous.inverse() * robot_world - current.inverse() * robot_world).norm();
  const Eigen::Matrix3d relative = previous.linear().transpose() * current.linear();
  const double yaw = std::abs(std::atan2(relative(1, 0), relative(0, 0)));
  return translation > max_translation || yaw > max_yaw;
}

inline bool referenceTransformFresh(double age_seconds, double max_age_seconds)
{
  return std::isfinite(age_seconds) && age_seconds >= -0.05 &&
         age_seconds <= max_age_seconds;
}

inline bool referenceTfGraceExpired(double missing_seconds, double grace_seconds)
{
  return missing_seconds >= grace_seconds;
}
}  // namespace scan_planner

#endif
