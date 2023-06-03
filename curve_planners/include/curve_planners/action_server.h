#pragma once

#include <memory>
#include <variant>

#include <action_msgs/msg/goal_status.hpp>
#include <curve_planners_msgs/action/plan_curve.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "curve_planners/dubins_planner.h"

namespace curve_planners {

using Planner = std::variant<DubinsPlanner>;

class PlanCurveActionServer : public rclcpp::Node {
public:
  using PlanCurve = curve_planners_msgs::action::PlanCurve;
  using GoalHandlePlanCurve = rclcpp_action::ServerGoalHandle<PlanCurve>;

  explicit PlanCurveActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  rclcpp_action::Server<PlanCurve>::SharedPtr action_server_;
  std::map<std::string, Planner> planners_;
  // DubinsPlanner dubins_planner_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const PlanCurve::Goal> goal);

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandlePlanCurve> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandlePlanCurve> goal_handle);

  void execute(const std::shared_ptr<GoalHandlePlanCurve> goal_handle);
}; // class PlanCurveActionServer

} // namespace curve_planners
