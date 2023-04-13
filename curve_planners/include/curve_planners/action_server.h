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
    std::thread{std::bind(&PlanCurveActionServer::execute, this, _1),
                goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();

    auto result = std::make_shared<PlanCurve::Result>();

    std::vector<curve_planners::Pose> path_points;
    path_points.push_back(fromMsg(goal->start));
    for (auto &waypoint : goal->waypoints) {
      path_points.push_back(fromMsg(waypoint));
    }
    path_points.push_back(fromMsg(goal->goal));

    std::vector<Path> plans;
    for (size_t i = 0; i < path_points.size() - 1; i++) {
      auto plan = dubins_planner_.plan(path_points[i], path_points[i + 1]);
      if (!plan) {
        result->success = false;
        goal_handle->succeed(result);
        RCLCPP_ERROR(this->get_logger(), "Planning failed.");
        return;
      }
      plans.push_back(plan.value());
    }
    Path merged_path;
    for (auto &plan : plans) {
      merged_path.insert(merged_path.end(), plan.begin(), plan.end());
    }

    auto res =
        toMsg(merged_path,
              goal->start.header.frame_id); // TODO: Remove assumption that all
                                            // points are in the same frame
    result->plan = res;
    path_publisher_->publish(res);

    if (rclcpp::ok()) {
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