#ifndef PLAN_MANAGE_REFERENCE_PATH_UTILS_H
#define PLAN_MANAGE_REFERENCE_PATH_UTILS_H

#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace scan_planner
{

inline bool transformReferencePath(
    const nav_msgs::msg::Path &path,
    const geometry_msgs::msg::TransformStamped &transform,
    nav_msgs::msg::Path &transformed_path,
    std::string *error = nullptr)
{
  transformed_path = nav_msgs::msg::Path();
  if (path.header.frame_id.empty())
  {
    if (error) *error = "reference path frame_id is empty";
    return false;
  }
  if (transform.header.frame_id.empty() || transform.child_frame_id.empty())
  {
    if (error) *error = "reference path transform has an empty frame_id";
    return false;
  }
  if (path.header.frame_id != transform.child_frame_id)
  {
    if (error) *error = "reference path frame does not match transform source frame";
    return false;
  }

  transformed_path.header = path.header;
  transformed_path.header.frame_id = transform.header.frame_id;
  transformed_path.poses.reserve(path.poses.size());
  for (const auto &pose : path.poses)
  {
    geometry_msgs::msg::PoseStamped source_pose = pose;
    if (source_pose.header.frame_id.empty())
      source_pose.header.frame_id = path.header.frame_id;
    if (source_pose.header.frame_id != path.header.frame_id)
    {
      transformed_path = nav_msgs::msg::Path();
      if (error) *error = "reference path contains poses in mixed frames";
      return false;
    }

    geometry_msgs::msg::PoseStamped transformed_pose;
    tf2::doTransform(source_pose, transformed_pose, transform);
    transformed_pose.header = transformed_path.header;
    transformed_path.poses.push_back(std::move(transformed_pose));
  }
  return true;
}

inline bool prepareReferenceWaypoints(
    const nav_msgs::msg::Path &path,
    double body_height,
    double min_distance,
    std::vector<Eigen::Vector3d> &waypoints,
    std::string *error = nullptr)
{
  waypoints.clear();
  if (path.poses.empty())
  {
    if (error) *error = "reference path is empty";
    return false;
  }
  if (!std::isfinite(body_height) || !std::isfinite(min_distance) || min_distance < 0.0)
  {
    if (error) *error = "body height and minimum distance must be finite and non-negative";
    return false;
  }

  waypoints.reserve(path.poses.size());
  Eigen::Vector3d final_waypoint;
  Eigen::Vector3d last_waypoint;
  bool first = true;

  for (const auto &pose_stamped : path.poses)
  {
    const auto &position = pose_stamped.pose.position;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z))
    {
      waypoints.clear();
      if (error) *error = "reference path contains a non-finite coordinate";
      return false;
    }

    Eigen::Vector3d waypoint(position.x, position.y, position.z + body_height);
    final_waypoint = waypoint;
    if (first || (waypoint - last_waypoint).norm() >= min_distance)
    {
      waypoints.push_back(waypoint);
      last_waypoint = waypoint;
      first = false;
    }
  }

  if ((waypoints.back() - final_waypoint).norm() > 1e-6)
    waypoints.push_back(final_waypoint);

  if (waypoints.size() < 2)
  {
    waypoints.clear();
    if (error) *error = "reference path requires at least two distinct points";
    return false;
  }
  return true;
}

}  // namespace scan_planner

#endif  // PLAN_MANAGE_REFERENCE_PATH_UTILS_H
