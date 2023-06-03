#include "curve_planners/action_server.h"

namespace curve_planners {

PlanCurveActionServer::PlanCurveActionServer(const rclcpp::NodeOptions &options)
    : Node("curve_planners_node", options) {
  using namespace std::placeholders;

  std::vector<std::string> planner_names;
  this->declare_parameter("planners", planner_names);
  this->get_parameter("planners", planner_names);
  for (auto &name : planner_names) {
    std::string planner_type;
    this->declare_parameter(name + ".type", planner_type);
    this->get_parameter(name + ".type", planner_type);

    if (planner_type == "DubinsPlanner") {
      double curve_radius{};
      this->declare_parameter(name + ".curve_radius", curve_radius);
      this->get_parameter(name + ".curve_radius", curve_radius);
      if (curve_radius <= 0) {
        RCLCPP_ERROR_STREAM(
            this->get_logger(),
            "Curve radius is not set or invalid. Using default 1.0 meters.");
        curve_radius = 1.0;
      }
      planners_.emplace(name, DubinsPlanner(curve_radius));
    } else {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Unknown planner type: " << planner_type);
    }
  }
  if (planners_.empty()) {
    throw std::runtime_error("No planners loaded successfully. Exiting.");
  }

  this->action_server_ = rclcpp_action::create_server<PlanCurve>(
      this, "plan_curve",
      std::bind(&PlanCurveActionServer::handle_goal, this, _1, _2),
      std::bind(&PlanCurveActionServer::handle_cancel, this, _1),
      std::bind(&PlanCurveActionServer::handle_accepted, this, _1));
  path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
}

rclcpp_action::GoalResponse PlanCurveActionServer::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const PlanCurve::Goal> goal) {
  RCLCPP_INFO_STREAM(this->get_logger(), "Received goal request with x pose "
                                             << goal->start.pose.position.x);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlanCurveActionServer::handle_cancel(
    const std::shared_ptr<GoalHandlePlanCurve> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlanCurveActionServer::handle_accepted(
    const std::shared_ptr<GoalHandlePlanCurve> goal_handle) {
  using namespace std::placeholders;
  std::thread{std::bind(&PlanCurveActionServer::execute, this, _1), goal_handle}
      .detach();
}

void PlanCurveActionServer::execute(
    const std::shared_ptr<GoalHandlePlanCurve> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  const auto goal = goal_handle->get_goal();

  auto result = std::make_shared<PlanCurve::Result>();

  // use goal->planner_name to get planner
  auto planner = planners_.find(goal->planner);
  if (planner == planners_.end()) {
    // warn and use first planner if planner not found
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "Planner " << goal->planner
                                  << " not found, using first planner");
    planner = planners_.begin();
  }
  std::vector<curve_planners::Pose> path_points;
  path_points.push_back(fromMsg(goal->start));
  for (auto &waypoint : goal->waypoints) {
    path_points.push_back(fromMsg(waypoint));
  }
  path_points.push_back(fromMsg(goal->goal));

  std::vector<Path> plans;
  for (size_t i = 0; i < path_points.size() - 1; i++) {

    auto plan = std::visit(
        [&](auto &planner) {
          return planner.plan(path_points[i], path_points[i + 1]);
        },
        planner->second);
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

} // namespace curve_planners

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(curve_planners::PlanCurveActionServer)