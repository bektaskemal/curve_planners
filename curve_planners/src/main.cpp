
#include "curve_planners/action_server.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<curve_planners::PlanCurveActionServer>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
