#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include "bspline_opt/uniform_bspline.h"

namespace scan_planner
{
class ClosedLoopController : public rclcpp::Node
{
public:
  ClosedLoopController() : Node("closed_loop_controller")
  {
    time_forward_ = declare_parameter<double>("time_forward", 0.8);
    heading_error_threshold_ = declare_parameter<double>("heading_error_threshold", 0.8);
    kp_pos_ = declare_parameter<double>("kp_pos", 0.8);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.5);
    max_vx_ = declare_parameter<double>("max_vx", 0.75);
    max_vy_ = declare_parameter<double>("max_vy", 0.35);
    max_vyaw_ = std::min(declare_parameter<double>("max_vyaw", 1.0), kMaxVYawLimit);
    finish_dist_ = declare_parameter<double>("finish_dist", 0.15);
    finish_yaw_ = declare_parameter<double>("finish_yaw", 0.10);
    if (finish_dist_ <= 0.0 || finish_yaw_ <= 0.0)
      throw std::runtime_error("finish_dist and finish_yaw must be positive");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&ClosedLoopController::bsplineCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&ClosedLoopController::odomCallback, this, std::placeholders::_1));
    rclcpp::QoS path_qos(rclcpp::KeepLast(1));
    path_qos.reliable().transient_local();
    global_path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "global_path", path_qos,
        std::bind(&ClosedLoopController::globalPathCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20);
    execution_frozen_pub_ = create_publisher<std_msgs::msg::Bool>("planning/go2_execution_frozen", 10);
    cmd_timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                   std::bind(&ClosedLoopController::cmdCallback, this));
    last_update_time_ = now();
    RCLCPP_INFO(get_logger(), "Closed-loop controller ready");
  }

private:
  static constexpr double kMaxVYawLimit = 1.0;

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  static Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
  {
    const double norm = value.norm();
    return (norm <= max_norm || norm < 1e-6) ? value : value / norm * max_norm;
  }

  double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des) const
  {
    const double t_look = std::min(traj_duration_, t_cur + time_forward_);
    Eigen::Vector3d direction = traj_[0].evaluateDeBoorT(t_look) - pos_des;
    if (direction.head<2>().squaredNorm() < 1e-4)
      direction = traj_[1].evaluateDeBoorT(t_cur);
    return direction.head<2>().squaredNorm() < 1e-4
        ? odom_yaw_ : std::atan2(direction.y(), direction.x());
  }

  void publishStop(double yaw_rate = 0.0)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = std::clamp(yaw_rate, -max_vyaw_, max_vyaw_);
    cmd_vel_pub_->publish(cmd);
  }

  void publishExecutionFrozen(bool frozen)
  {
    std_msgs::msg::Bool msg;
    msg.data = frozen;
    execution_frozen_pub_->publish(msg);
  }

  static bool validPose(const geometry_msgs::msg::PoseStamped &pose)
  {
    const auto &position = pose.pose.position;
    const auto &orientation = pose.pose.orientation;
    const double quaternion_norm_squared =
        orientation.x * orientation.x + orientation.y * orientation.y +
        orientation.z * orientation.z + orientation.w * orientation.w;
    return !pose.header.frame_id.empty() &&
           std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z) &&
           std::isfinite(orientation.x) && std::isfinite(orientation.y) &&
           std::isfinite(orientation.z) && std::isfinite(orientation.w) &&
           quaternion_norm_squared > 1e-12;
  }

  void globalPathCallback(const nav_msgs::msg::Path::ConstSharedPtr msg)
  {
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN(get_logger(), "[DIAG][CTRL] ignoring empty global path");
      return;
    }
    geometry_msgs::msg::PoseStamped goal = msg->poses.back();
    if (goal.header.frame_id.empty()) goal.header.frame_id = msg->header.frame_id;
    if (!validPose(goal))
    {
      RCLCPP_WARN(get_logger(), "[DIAG][CTRL] ignoring global path with invalid final pose");
      return;
    }
    global_goal_ = std::move(goal);
    have_global_goal_ = true;
    terminal_finished_ = false;
    finished_reported_ = false;
    RCLCPP_INFO(get_logger(),
                "[DIAG][CTRL] received global goal frame=%s position=(%.3f,%.3f,%.3f) yaw=%.3f",
                global_goal_.header.frame_id.c_str(), global_goal_.pose.position.x,
                global_goal_.pose.position.y, global_goal_.pose.position.z,
                tf2::getYaw(global_goal_.pose.orientation));
  }

  bool getGlobalGoal(Eigen::Vector2d &position, double &yaw)
  {
    if (!have_global_goal_ || odom_frame_id_.empty()) return false;
    geometry_msgs::msg::PoseStamped goal = global_goal_;
    try
    {
      if (goal.header.frame_id != odom_frame_id_)
      {
        // Use the latest transform so a dynamic map-to-odometry correction is reflected at the goal.
        goal.header.stamp = builtin_interfaces::msg::Time();
        goal = tf_buffer_->transform(goal, odom_frame_id_, tf2::durationFromSec(0.02));
      }
    }
    catch (const tf2::TransformException &error)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "[DIAG][CTRL] cannot transform global goal from %s to %s: %s",
                           global_goal_.header.frame_id.c_str(), odom_frame_id_.c_str(), error.what());
      return false;
    }
    if (!validPose(goal))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "[DIAG][CTRL] transformed global goal is invalid");
      return false;
    }
    position << goal.pose.position.x, goal.pose.position.y;
    yaw = tf2::getYaw(goal.pose.orientation);
    return std::isfinite(yaw);
  }

  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    bool finite = true;
    for (const auto &point : msg->pos_pts)
      finite = finite && std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
    for (size_t i = 0; i < msg->knots.size(); ++i)
      finite = finite && std::isfinite(msg->knots[i]) &&
               (i == 0 || msg->knots[i] >= msg->knots[i - 1]);
    const size_t expected_knots = msg->pos_pts.size() + static_cast<size_t>(std::max(msg->order, 0)) + 1;
    if (msg->pos_pts.size() <= static_cast<size_t>(std::max(msg->order, 0)) ||
        msg->knots.size() != expected_knots || msg->order <= 0 || !finite)
    {
      RCLCPP_ERROR(get_logger(),
                   "[DIAG][CTRL] invalid_trajectory traj_id=%lld order=%d points=%zu knots=%zu "
                   "expected_knots=%zu finite_and_monotonic=%d",
                   static_cast<long long>(msg->traj_id), msg->order, msg->pos_pts.size(),
                   msg->knots.size(), expected_knots, finite);
      return;
    }
    Eigen::MatrixXd points(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
      points.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];
    UniformBspline position(points, msg->order, 0.1);
    position.setKnot(knots);
    traj_ = {position, position.getDerivative()};
    traj_.push_back(traj_[1].getDerivative());
    traj_duration_ = traj_[0].getTimeSum();
    traj_id_ = msg->traj_id;
    exec_time_ = 0.0;
    last_update_time_ = now();
    receive_traj_ = true;
    terminal_finished_ = false;
    finished_reported_ = false;
    const Eigen::Vector3d first = points.col(0);
    const Eigen::Vector3d last = points.col(points.cols() - 1);
    RCLCPP_INFO(get_logger(),
                "[DIAG][CTRL] received traj_id=%lld duration=%.3f order=%d points=%zu "
                "start=(%.3f,%.3f,%.3f) end=(%.3f,%.3f,%.3f)",
                static_cast<long long>(traj_id_), traj_duration_, msg->order, msg->pos_pts.size(),
                first.x(), first.y(), first.z(), last.x(), last.y(), last.z());
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    odom_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    odom_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    odom_frame_id_ = msg->header.frame_id;
    have_odom_ = true;
    last_odom_receive_time_ = now();
  }

  void cmdCallback()
  {
    if (!receive_traj_ || !have_odom_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "[DIAG][CTRL] waiting trajectory=%d odometry=%d",
                           receive_traj_, have_odom_);
      publishExecutionFrozen(false);
      publishStop();
      return;
    }
    const auto current_time = now();
    if (terminal_finished_)
    {
      publishExecutionFrozen(false);
      publishStop();
      last_update_time_ = current_time;
      return;
    }
    double dt = (current_time - last_update_time_).seconds();
    if (dt < 0.0 || dt > 0.2) dt = 0.0;
    const double t_eval = std::min(exec_time_, traj_duration_);
    Eigen::Vector3d pos_des = traj_[0].evaluateDeBoorT(t_eval);
    const Eigen::Vector2d initial_pos_error(
        pos_des.x() - odom_pos_.x(), pos_des.y() - odom_pos_.y());
    Eigen::Vector2d goal_position;
    double goal_yaw = 0.0;
    const bool have_transformed_goal = getGlobalGoal(goal_position, goal_yaw);
    const double goal_distance = have_transformed_goal
        ? (goal_position - odom_pos_.head<2>()).norm()
        : std::numeric_limits<double>::infinity();
    const bool at_global_goal = have_transformed_goal && goal_distance < finish_dist_;
    if (exec_time_ >= traj_duration_ && initial_pos_error.norm() < finish_dist_ && at_global_goal)
    {
      frozen_ = false;
      publishExecutionFrozen(false);
      const double final_yaw_error = normalizeAngle(goal_yaw - odom_yaw_);
      if (std::abs(final_yaw_error) < finish_yaw_)
      {
        terminal_finished_ = true;
        publishStop();
        if (!finished_reported_)
        {
          RCLCPP_INFO(get_logger(),
                      "[DIAG][CTRL] finished traj_id=%lld position_error=%.3f "
                      "goal_distance=%.3f yaw_error=%.3f",
                      static_cast<long long>(traj_id_), initial_pos_error.norm(), goal_distance,
                      final_yaw_error);
          finished_reported_ = true;
        }
      }
      else
      {
        const double final_yaw_command =
            std::clamp(kp_yaw_ * final_yaw_error, -max_vyaw_, max_vyaw_);
        publishStop(final_yaw_command);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                             "[DIAG][CTRL] aligning final yaw traj_id=%lld error=%.3f cmd=%.3f",
                             static_cast<long long>(traj_id_), final_yaw_error,
                             final_yaw_command);
      }
      last_update_time_ = current_time;
      return;
    }
    const double yaw_error = normalizeAngle(estimateDesiredYaw(t_eval, pos_des) - odom_yaw_);
    const double yaw_command = std::clamp(kp_yaw_ * yaw_error, -max_vyaw_, max_vyaw_);
    if (std::abs(yaw_error) > heading_error_threshold_)
    {
      if (!frozen_)
        RCLCPP_WARN(get_logger(),
                    "[DIAG][CTRL] freeze traj_id=%lld exec_t=%.3f yaw_error=%.3f threshold=%.3f "
                    "odom_yaw=%.3f desired_yaw=%.3f",
                    static_cast<long long>(traj_id_), exec_time_, yaw_error,
                    heading_error_threshold_, odom_yaw_, normalizeAngle(odom_yaw_ + yaw_error));
      frozen_ = true;
      publishExecutionFrozen(true);
      publishStop(yaw_command);
      last_update_time_ = current_time;
      return;
    }

    if (frozen_)
      RCLCPP_INFO(get_logger(), "[DIAG][CTRL] resume traj_id=%lld yaw_error=%.3f",
                  static_cast<long long>(traj_id_), yaw_error);
    frozen_ = false;
    publishExecutionFrozen(false);
    exec_time_ = std::min(traj_duration_, exec_time_ + dt);
    last_update_time_ = current_time;
    pos_des = traj_[0].evaluateDeBoorT(exec_time_);
    const Eigen::Vector3d vel_des = traj_[1].evaluateDeBoorT(exec_time_);
    const Eigen::Vector2d pos_error(pos_des.x() - odom_pos_.x(), pos_des.y() - odom_pos_.y());
    const bool at_local_endpoint = exec_time_ >= traj_duration_ && pos_error.norm() < finish_dist_;
    const Eigen::Vector2d vel_world = clampNorm(
        Eigen::Vector2d(vel_des.x(), vel_des.y()) + kp_pos_ * pos_error,
        std::max(max_vx_, max_vy_));
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    geometry_msgs::msg::Twist command;
    command.linear.x = std::clamp(c * vel_world.x() + s * vel_world.y(), -max_vx_, max_vx_);
    command.linear.y = std::clamp(-s * vel_world.x() + c * vel_world.y(), -max_vy_, max_vy_);
    command.angular.z = yaw_command;
    if (at_local_endpoint)
    {
      command = geometry_msgs::msg::Twist();
      if (!finished_reported_)
      {
        RCLCPP_INFO(get_logger(),
                    "[DIAG][CTRL] finished traj_id=%lld position_error=%.3f",
                    static_cast<long long>(traj_id_), pos_error.norm());
        finished_reported_ = true;
      }
    }
    const double odom_age = (current_time - last_odom_receive_time_).seconds();
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "[DIAG][CTRL] tracking traj_id=%lld exec=%.3f/%.3f pos_error=%.3f "
                         "yaw_error=%.3f odom_age=%.3f cmd=(%.3f,%.3f,%.3f)",
                         static_cast<long long>(traj_id_), exec_time_, traj_duration_, pos_error.norm(),
                         yaw_error, odom_age, command.linear.x, command.linear.y, command.angular.z);
    cmd_vel_pub_->publish(command);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool receive_traj_{false};
  bool have_odom_{false};
  bool frozen_{false};
  bool finished_reported_{false};
  bool have_global_goal_{false};
  bool terminal_finished_{false};
  geometry_msgs::msg::PoseStamped global_goal_;
  std::string odom_frame_id_;
  std::vector<UniformBspline> traj_;
  double traj_duration_{0.0};
  std::int64_t traj_id_{0};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  double odom_yaw_{0.0};
  double exec_time_{0.0};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_receive_time_{0, 0, RCL_ROS_TIME};
  double time_forward_, heading_error_threshold_, kp_pos_, kp_yaw_;
  double max_vx_, max_vy_, max_vyaw_, finish_dist_, finish_yaw_;
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<scan_planner::ClosedLoopController>();
  try
  {
    rclcpp::spin(node);
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(node->get_logger(), "[DIAG][CRASH] controller exception: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  catch (...)
  {
    RCLCPP_FATAL(node->get_logger(), "[DIAG][CRASH] controller non-standard exception");
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
