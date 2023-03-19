#include "curve_planners_msgs/srv/plan_curve.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>

void add(
    const std::shared_ptr<curve_planners_msgs::srv::PlanCurve::Request> request,
    std::shared_ptr<curve_planners_msgs::srv::PlanCurve::Response> response) {
  response->plan = {};
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                     "Start x: " << request->start.pose.position.x); // CHANGE
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("curve_planner_server");

  rclcpp::Service<curve_planners_msgs::srv::PlanCurve>::SharedPtr service =
      node->create_service<curve_planners_msgs::srv::PlanCurve>("plan_curve",
                                                                &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to plan curves.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}