
#pragma once

#include <algorithm>
#include <limits>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include <tl_expected/expected.hpp>

#include "curve_planners/types.h"

namespace curve_planners {

/// @brief Dubins path planner
class DubinsPlanner {
public:
  explicit DubinsPlanner(double curve_radius);

  /// @brief Plans shortest dubins path between start and goal
  /// @param start start pose
  /// @param goal goal pose
  /// @return Path if success, error message if failed
  auto plan(Pose const &start, Pose const &goal)
      -> tl::expected<Path, std::string>;

  /// @brief Plans all possible dubins paths between start and goal
  /// @param start start pose
  /// @param goal goal pose
  /// @return Sorted list of successful Dubins paths
  auto planCurves(Pose const &start, Pose const &goal)
      -> std::vector<DubinsPath>;

  /// @brief convert DubinsPath to Path by sampling points on segments
  auto toPath(DubinsPath const &path, Pose const &start) -> Path;

private:
  /// @brief dubins planning functions
  static auto dubins_LSL(DubinsInputs const &inputs)
      -> tl::expected<DubinsPath, std::string>;
  static auto dubins_RSR(DubinsInputs const &inputs)
      -> tl::expected<DubinsPath, std::string>;
  static auto dubins_LSR(DubinsInputs const &inputs)
      -> tl::expected<DubinsPath, std::string>;
  static auto dubins_RSL(DubinsInputs const &inputs)
      -> tl::expected<DubinsPath, std::string>;
  static auto dubins_RLR(DubinsInputs const &inputs)
      -> tl::expected<DubinsPath, std::string>;
  static auto dubins_LRL(DubinsInputs const &inputs)
      -> tl::expected<DubinsPath, std::string>;

  static auto samplePose(Pose const &start, double length, SegmentType type,
                         double radius) -> Pose;

  double curve_radius_;
};
} // namespace curve_planners
