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

enum class SegmentType { Left, Right, Straight };
struct Segment {
  SegmentType type;
  double normalized_length;
};
using Path = std::array<Segment, 3>;

// auto sampleSegmentNormalized(Pose const &start, double t, SegmentType type)
//     -> Pose {
//   Pose res{0, 0, start.theta};
//   if (type == SegmentType::Left) {
//     res.theta += t;
//     res.x = start.x + sin(res.theta) - sin(start.theta);
//     res.y = start.y - cos(res.theta) + cos(start.theta);
//   } else if (type == SegmentType::Right) {
//     res.theta -= t;
//     res.x = start.x - sin(res.theta) + sin(start.theta);
//     res.y = start.y + cos(res.theta) - cos(start.theta);
//   } else if (type == SegmentType::Straight) {
//     res.x = start.x + cos(start.theta) * t;
//     res.y = start.y + sin(start.theta) * t;
//   }
//   res.normalize_theta();
//   return res;
// }

auto samplePose(Pose const &start, double length, SegmentType type,
                double radius) -> Pose {
  Pose res{};
  if (type == SegmentType::Straight) {
    res.x = start.x + length / radius * std::cos(start.theta);
    res.y = start.y + length / radius * std::sin(start.theta);
    res.theta = start.theta;

  } else {
    auto ldx = std::sin(length) / radius;
    auto ldy = 0.0;
    if (type == SegmentType::Left) {
      ldy = (1.0 - std::cos(length)) / radius;
    } else if (type == SegmentType::Right) {
      ldy = (1.0 - std::cos(length)) / -radius;
    }
    auto gdx = std::cos(-start.theta) * ldx + std::sin(-start.theta) * ldy;
    auto gdy = -std::sin(-start.theta) * ldx + std::cos(-start.theta) * ldy;

    res.x = start.x + gdx;
    res.y = start.y + gdy;
    if (type == SegmentType::Left) {
      res.theta = start.theta + length;
    } else if (type == SegmentType::Right) {
      res.theta = start.theta - length;
    }
  }
  res.normalize_theta();
  return res;
}

// convert Path to nav_msgs::msg::Path by sampling points on segments
auto toMsg(Path const &path, Pose const &start, double radius)
    -> nav_msgs::msg::Path {

  nav_msgs::msg::Path result;
  result.header.frame_id = "map";
  result.header.stamp = rclcpp::Clock().now();

  auto step_size = 0.1 / radius;
  std::vector<Pose> poses;
  poses.push_back(start);
  for (auto const &segment : path) {
    std::cout << "segment type: " << (int)segment.type << std::endl;
    std::cout << "segment length: " << segment.normalized_length << std::endl;
    auto origin = poses.back();
    auto length = step_size;
    while (std::abs(length) <= std::abs(segment.normalized_length)) {
      auto pose = samplePose(origin, length, segment.type, radius);
      poses.push_back(pose);
      length += step_size;
    }
    auto pose = samplePose(origin, std::abs(segment.normalized_length),
                           segment.type, radius);
    poses.push_back(pose);
  }

  std::transform(poses.begin(), poses.end(), std::back_inserter(result.poses),
                 [](Pose const &pose) { return toMsg(pose, "map"); });

  return result;
}

} // namespace curve_planners
