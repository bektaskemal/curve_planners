#pragma once

#include <memory>

#include <action_msgs/msg/goal_status.hpp>
#include <curve_planners_msgs/action/plan_curve.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "curve_planners/dubins_planner.h"

namespace curve_planners {
class PlanCurveActionServer : public rclcpp::Node {
public:
  using PlanCurve = curve_planners_msgs::action::PlanCurve;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<PlanCurve>;

  explicit PlanCurveActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("curve_planner_server", options),
        dubins_planner_(1.0) { // TODO: Get min_turning_radius
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<PlanCurve>(
        this, "plan_curve",
        std::bind(&PlanCurveActionServer::handle_goal, this, _1, _2),
        std::bind(&PlanCurveActionServer::handle_cancel, this, _1),
        std::bind(&PlanCurveActionServer::handle_accepted, this, _1));
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
  }

private:
  rclcpp_action::Server<PlanCurve>::SharedPtr action_server_;
  DubinsPlanner dubins_planner_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const PlanCurve::Goal> goal) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Received goal request with x pose "
                                               << goal->start.pose.position.x);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&PlanCurveActionServer::execute, this, _1),
                goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();

    auto result = std::make_shared<PlanCurve::Result>();
    auto start = fromMsg(goal->start);
    auto plan = dubins_planner_.plan(start, fromMsg(goal->goal));

    if (plan) {
      // L 1.2762808416867601
      // S 3.741657386773941
      // R 1.2762808416867601
      // curve_planners::Path path;
      // path.at(0) = {curve_planners::SegmentType::Left, 1.2762808416867601};
      // path.at(1) =
      // {curve_planners::SegmentType::Straight, 3.741657386773941}; path.at(2)
      // = {curve_planners::SegmentType::Right, 1.2762808416867601};
      result->plan = toMsg(plan.value(), start, 1.0);
      path_publisher_->publish(result->plan);
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      return;
    }
    result->success = false;
    goal_handle->abort(result);
    RCLCPP_INFO(this->get_logger(), "Goal aborted");
  }
}; // class PlanCurveActionServer

} // namespace curve_planners

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(curve_planners::PlanCurveActionServer)