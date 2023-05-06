#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include <curve_planners/dubins_planner.h>
#include <curve_planners/types.h>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_dubins_planner {

class DubinsPlanner : public nav2_core::GlobalPlanner {
public:
  // plugin configure
  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // plugin cleanup
  void cleanup() override;

  // plugin activate
  void activate() override;

  // plugin deactivate
  void deactivate() override;

  // This method creates path for given start and goal pose.
  nav_msgs::msg::Path
  createPlan(const geometry_msgs::msg::PoseStamped &start,
             const geometry_msgs::msg::PoseStamped &goal) override;

protected:
  auto isCollisionFree(const std::vector<geometry_msgs::msg::PoseStamped> &path)
      -> bool;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_util::LifecycleNode::SharedPtr node_;
  nav2_costmap_2d::Costmap2D *costmap_;
  std::string global_frame_, name_;
  rclcpp::Logger logger_{rclcpp::get_logger("DubinsPlanner")};
  std::unique_ptr<curve_planners::DubinsPlanner> planner_;
};

} // namespace nav2_dubins_planner
