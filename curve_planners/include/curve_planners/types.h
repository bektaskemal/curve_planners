#pragma once

#include <vector>

#include <angles/angles.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/convert.h>
#include <tf2/utils.h>

namespace curve_planners {

struct Pose {
  double x;
  double y;
  double theta;
  Pose operator-(const Pose &other) const {
    double dx = x - other.x;
    double dy = y - other.y;
    double dtheta = angles::normalize_angle(theta - other.theta);

    return {dx, dy, dtheta};
  }
  auto xyAngle() const -> double {
    return angles::normalize_angle(atan2(y, x)); // TODO: Is normalize needed?
  }
  // auto angleTo(const Pose &other) const -> double {
  //   double dx = other.x - x;
  //   double dy = other.y - y;
  //   return angles::normalize_angle(atan2(dy, dx));
  // }
  auto norm() -> double { return std::hypot(x, y); }

  void normalize_theta() { theta = angles::normalize_angle(theta); }
};
using Path = std::vector<Pose>;

auto fromMsg(geometry_msgs::msg::PoseStamped const &pose) -> Pose {
  return {pose.pose.position.x, pose.pose.position.y,
          tf2::getYaw(tf2::Quaternion(
              pose.pose.orientation.x, pose.pose.orientation.y,
              pose.pose.orientation.z, pose.pose.orientation.w))};
}

auto toMsg(Pose const &pose, std::string const &frame_id)
    -> geometry_msgs::msg::PoseStamped {
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = frame_id;
  result.header.stamp = rclcpp::Clock().now();
  result.pose.position.x = pose.x;
  result.pose.position.y = pose.y;
  result.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, pose.theta);
  result.pose.orientation.x = q.getX();
  result.pose.orientation.y = q.getY();
  result.pose.orientation.z = q.getZ();
  result.pose.orientation.w = q.getW();
  return result;
}

auto fromMsg(nav_msgs::msg::Path const &path) -> std::vector<Pose> {
  std::vector<Pose> result(path.poses.size());
  std::transform(path.poses.begin(), path.poses.end(), result.begin(),
                 [](geometry_msgs::msg::PoseStamped const &pose) {
                   return fromMsg(pose);
                 });
  return result;
}

struct DubinsInputs {
  DubinsInputs() = default;
  DubinsInputs(Pose const &start, Pose const &goal, double min_turning_radius) {
    auto delta = goal - start;
    double dist = delta.norm();
    normalized_dist = dist / min_turning_radius;
    theta = delta.xyAngle();
    alpha = angles::normalize_angle_positive(start.theta - theta);
    beta = angles::normalize_angle_positive(goal.theta - theta);
  }
  double alpha;
  double beta;
  double normalized_dist;
  double theta;
};

enum class SegmentType { Left, Right, Straight };
struct Segment {
  SegmentType type;
  double normalized_length;
};
using DubinsPath = std::array<Segment, 3>;

// Convert Path to nav_msgs::msg::Path
auto toMsg(Path const &path, std::string const &frame_id)
    -> nav_msgs::msg::Path {
  nav_msgs::msg::Path result;
  result.header.frame_id = frame_id;
  result.header.stamp = rclcpp::Clock().now();
  result.poses.resize(path.size());
  std::transform(
      path.begin(), path.end(), result.poses.begin(),
      [&frame_id](Pose const &pose) { return toMsg(pose, frame_id); });
  return result;
}
} // namespace curve_planners
