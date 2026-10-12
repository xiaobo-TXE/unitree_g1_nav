// Copyright 2026 jiewang

#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "global_planner/global_planner_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<global_planner::GlobalPlannerNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("global_planner"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
