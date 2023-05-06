
#include <cmath>
#include <memory>
#include <string>

#include <nav2_util/node_utils.hpp>

#include "nav2_dubins_planner/dubins_planner.hpp"

namespace nav2_dubins_planner {

void DubinsPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent.lock();
  logger_ = node_->get_logger();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // get curve radius
  double curve_radius;
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".curve_radius",
                                               rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".curve_radius", curve_radius);
  std::cerr << "!!!!!!!!curve_radius: " << curve_radius << std::endl;
  planner_ = std::make_unique<curve_planners::DubinsPlanner>(curve_radius);
}

void DubinsPlanner::cleanup() {
  RCLCPP_INFO(logger_, "CleaningUp plugin %s of type NavfnPlanner",
              name_.c_str());
}

void DubinsPlanner::activate() {
  RCLCPP_INFO(logger_, "Activating plugin %s of type NavfnPlanner",
              name_.c_str());
  //   // Add callback for dynamic parameters
  //   auto node = parent_node_.lock();
  //   dyn_params_handler_ = node->add_on_set_parameters_callback(
  //       std::bind(&ThetaStarPlanner::dynamicParametersCallback, this,
  //                 std::placeholders::_1));
}

void DubinsPlanner::deactivate() {
  RCLCPP_INFO(logger_, "Deactivating plugin %s of type NavfnPlanner",
              name_.c_str());
}

auto DubinsPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
                               const geometry_msgs::msg::PoseStamped &goal)
    -> nav_msgs::msg::Path {
  nav_msgs::msg::Path global_path;
  auto start_time = std::chrono::steady_clock::now();

  unsigned int mx_start, my_start, mx_goal, my_goal;
  costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start,
                       my_start);
  costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal,
                       my_goal);
  // Corner case of start and goal beeing on the same cell
  if (mx_start == mx_goal && my_start == my_goal) {
    if (costmap_->getCost(mx_start, my_start) ==
        nav2_costmap_2d::LETHAL_OBSTACLE) {
      RCLCPP_WARN(logger_,
                  "Failed to create a unique pose path because of obstacles");
      return global_path;
    }
    global_path.header.stamp = node_->get_clock()->now();
    global_path.header.frame_id = global_frame_;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = global_path.header;
    pose.pose.position.z = 0.0;
    pose.pose = goal.pose;

    global_path.poses.push_back(pose);
    return global_path;
  }

  auto plans = planner_->planCurves(curve_planners::fromMsg(start),
                                    curve_planners::fromMsg(goal));
  bool found_plan = false;
  for (auto &plan : plans) {
    auto path = curve_planners::toMsg(
        planner_->toPath(plan, curve_planners::fromMsg(start)), global_frame_);
    if (isCollisionFree(path.poses)) {
      global_path = path;
      found_plan = true;
      break;
    }
  }

  if (!found_plan) {
    RCLCPP_WARN(logger_, "Failed to create a path because of "
                         "obstacles or infeasible path");
    return global_path;
  }

  auto stop_time = std::chrono::steady_clock::now();
  auto dur = std::chrono::duration_cast<std::chrono::microseconds>(stop_time -
                                                                   start_time);
  RCLCPP_DEBUG(logger_, "the time taken is : %i",
               static_cast<int>(dur.count()));
  return global_path;
}

auto DubinsPlanner::isCollisionFree(
    const std::vector<geometry_msgs::msg::PoseStamped> &path) -> bool {
  for (auto &pose : path) {
    unsigned int mx, my;
    costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my);
    if (costmap_->getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE) {
      return false;
    }
  }
  return true;
}

} // namespace nav2_dubins_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_dubins_planner::DubinsPlanner,
                       nav2_core::GlobalPlanner)