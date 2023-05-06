#include "curve_planners/dubins_planner.h"

namespace curve_planners {

DubinsPlanner::DubinsPlanner(double curve_radius)
    : curve_radius_(curve_radius) {}

auto DubinsPlanner::plan(Pose const &start, Pose const &goal)
    -> tl::expected<Path, std::string> {
  return planCurves(start, goal)
      .and_then([&](DubinsPath const &path) -> tl::expected<Path, std::string> {
        return toPath(path, start);
      });
}

auto DubinsPlanner::planCurves(Pose const &start, Pose const &goal)
    -> tl::expected<DubinsPath, std::string> {
  DubinsInputs inputs(start, goal, curve_radius_);
  std::vector<DubinsPath> plans;

  auto calc_path_length = [](const DubinsPath &path) {
    return std::accumulate(path.begin(), path.end(), 0.0,
                           [](double acc, const Segment &segment) {
                             return acc + std::abs(segment.normalized_length);
                           });
  };
  DubinsPath best_plan;
  double min_length = std::numeric_limits<double>::max();

  auto run_planner_func = [&](auto func) {
    if (auto res = func(inputs); res.has_value()) {
      double length = calc_path_length(res.value());
      std::cout << "length: " << length << std::endl;
      if (length < min_length) {
        min_length = length;
        best_plan = res.value();
      }
    }
  };
  run_planner_func(dubins_LSL);
  run_planner_func(dubins_RSR);
  run_planner_func(dubins_LSR);
  run_planner_func(dubins_RSL);
  run_planner_func(dubins_LRL);
  run_planner_func(dubins_RLR);

  if (min_length == std::numeric_limits<double>::max()) {
    return tl::make_unexpected("Couldn't find a path!");
  }
  return best_plan;
}

auto DubinsPlanner::dubins_LSL(DubinsInputs const &inputs)
    -> tl::expected<DubinsPath, std::string> {
  double p_squared = 2 + std::pow(inputs.normalized_dist, 2) -
                     (2 * std::cos(inputs.alpha - inputs.beta)) +
                     (2 * inputs.normalized_dist *
                      (std::sin(inputs.alpha) - std::sin(inputs.beta)));
  if (p_squared < 0) {
    return tl::make_unexpected(
        "P squared smaller than zero so dubins failed."); // TODO: Check
                                                          // actual meaning of
                                                          // it
  }
  double tmp0 =
      inputs.normalized_dist +
      (std::sin(inputs.alpha) - std::sin(inputs.beta)); // TODO: Better naming
  double tmp1 = std::atan2((std::cos(inputs.beta) - std::cos(inputs.alpha)),
                           tmp0); // TODO: Better naming
  double t = angles::normalize_angle_positive(-inputs.alpha + tmp1);
  double p = sqrt(p_squared);
  double q = angles::normalize_angle_positive(inputs.beta - tmp1);

  return DubinsPath{{{SegmentType::Left, t},
                     {SegmentType::Straight, p},
                     {SegmentType::Left, q}}};
}
auto DubinsPlanner::dubins_RSR(DubinsInputs const &inputs)
    -> tl::expected<DubinsPath, std::string> {

  double tmp0 =
      inputs.normalized_dist -
      (std::sin(inputs.alpha) - std::sin(inputs.beta)); // TODO: Better naming
  double p_squared = 2 + (inputs.normalized_dist * inputs.normalized_dist) -
                     (2 * std::cos(inputs.alpha - inputs.beta)) -
                     (2 * inputs.normalized_dist *
                      (std::sin(inputs.alpha) - std::sin(inputs.beta)));
  if (p_squared < 0) {
    return tl::make_unexpected(
        "P squared smaller than zero so dubins failed."); // TODO: Check
                                                          // actual meaning of
                                                          // it
  }
  double tmp1 = std::atan2((std::cos(inputs.alpha) - std::cos(inputs.beta)),
                           tmp0); // TODO: Better naming
  double t = angles::normalize_angle_positive(inputs.alpha - tmp1);
  double p = sqrt(p_squared);
  double q = angles::normalize_angle_positive(-inputs.beta + tmp1);
  return DubinsPath{{{SegmentType::Right, t},
                     {SegmentType::Straight, p},
                     {SegmentType::Right, q}}};
}
auto DubinsPlanner::dubins_LSR(DubinsInputs const &inputs)
    -> tl::expected<DubinsPath, std::string> {

  double p_squared = -2 + std::pow(inputs.normalized_dist, 2) +
                     (2 * std::cos(inputs.alpha - inputs.beta)) +
                     (2 * inputs.normalized_dist *
                      (std::sin(inputs.alpha) + std::sin(inputs.beta)));
  if (p_squared < 0) {
    return tl::make_unexpected(
        "P squared smaller than zero so dubins failed."); // TODO: Check
                                                          // actual meaning of
                                                          // it
  }

  double p = sqrt(p_squared);
  double tmp = atan2((-std::cos(inputs.alpha) - std::cos(inputs.beta)),
                     (inputs.normalized_dist + std::sin(inputs.alpha) +
                      std::sin(inputs.beta))) -
               std::atan2(-2.0, p); // TODO: Better naming
  double t = angles::normalize_angle_positive(-inputs.alpha + tmp);
  double q = angles::normalize_angle_positive(-inputs.beta + tmp);

  return DubinsPath{{{SegmentType::Left, t},
                     {SegmentType::Straight, p},
                     {SegmentType::Right, q}}};
}

auto DubinsPlanner::dubins_RSL(DubinsInputs const &inputs)
    -> tl::expected<DubinsPath, std::string> {

  double p_squared = -2 + std::pow(inputs.normalized_dist, 2) +
                     (2 * std::cos(inputs.alpha - inputs.beta)) -
                     (2 * inputs.normalized_dist *
                      (std::sin(inputs.alpha) + std::sin(inputs.beta)));
  if (p_squared < 0) {
    return tl::make_unexpected(
        "P squared smaller than zero so dubins failed."); // TODO: Check
                                                          // actual meaning of
                                                          // it
  }

  double p = sqrt(p_squared);
  double tmp = atan2((std::cos(inputs.alpha) + std::cos(inputs.beta)),
                     (inputs.normalized_dist - std::sin(inputs.alpha) -
                      std::sin(inputs.beta))) -
               std::atan2(2.0, p); // TODO: Better naming
  double t = angles::normalize_angle_positive(inputs.alpha - tmp);
  double q = angles::normalize_angle_positive(inputs.beta - tmp);

  return DubinsPath{{{SegmentType::Right, t},
                     {SegmentType::Straight, p},
                     {SegmentType::Left, q}}};
}
auto DubinsPlanner::dubins_RLR(DubinsInputs const &inputs)
    -> tl::expected<DubinsPath, std::string> {

  double tmp = (6.0 - inputs.normalized_dist * inputs.normalized_dist +
                2 * std::cos(inputs.alpha - inputs.beta) +
                2 * inputs.normalized_dist *
                    (std::sin(inputs.alpha) - std::sin(inputs.beta))) /
               8.0; // TODO: Better naming
  if (std::abs(tmp) > 1) {
    return tl::make_unexpected(
        "P squared smaller than zero so dubins failed."); // TODO: Check
                                                          // actual meaning of
                                                          // it
  }

  double p = angles::normalize_angle_positive(2 * M_PI - std::acos(tmp));
  double t = angles::normalize_angle_positive(
      inputs.alpha -
      std::atan2(std::cos(inputs.alpha) - std::cos(inputs.beta),
                 inputs.normalized_dist - std::sin(inputs.alpha) +
                     std::sin(inputs.beta)) +
      angles::normalize_angle_positive(p / 2.));
  double q =
      angles::normalize_angle_positive(inputs.alpha - inputs.beta - t + p);

  return DubinsPath{{{SegmentType::Right, t},
                     {SegmentType::Left, p},
                     {SegmentType::Right, q}}};
}

auto DubinsPlanner::dubins_LRL(DubinsInputs const &inputs)
    -> tl::expected<DubinsPath, std::string> {

  double tmp = (6.0 - inputs.normalized_dist * inputs.normalized_dist +
                2 * std::cos(inputs.alpha - inputs.beta) -
                2 * inputs.normalized_dist *
                    (std::sin(inputs.alpha) - std::sin(inputs.beta))) /
               8.0; // TODO: Better naming
  if (std::abs(tmp) > 1) {
    return tl::make_unexpected(
        "P squared smaller than zero so dubins failed."); // TODO: Check
                                                          // actual meaning of
                                                          // it
  }

  double p = angles::normalize_angle_positive(2 * M_PI - std::acos(tmp));
  double t = angles::normalize_angle_positive(
      -inputs.alpha -
      std::atan2(std::cos(inputs.alpha) - std::cos(inputs.beta),
                 inputs.normalized_dist + std::sin(inputs.alpha) -
                     std::sin(inputs.beta)) +
      angles::normalize_angle_positive(
          p / 2.)); // TODO: Check why this is not normalized in ipa
  double q =
      angles::normalize_angle_positive(inputs.beta - inputs.alpha - t + p);

  return DubinsPath{{{SegmentType::Left, t},
                     {SegmentType::Right, p},
                     {SegmentType::Left, q}}};
}

auto DubinsPlanner::samplePose(Pose const &start, double length,
                               SegmentType type, double radius) -> Pose {
  Pose res{};
  if (type == SegmentType::Straight) {
    res.x = start.x + length * std::cos(start.theta);
    res.y = start.y + length * std::sin(start.theta);
    res.theta = start.theta;

  } else {
    auto ldx = std::sin(length / radius);
    auto ldy = 0.0;
    if (type == SegmentType::Left) {
      ldy = (1.0 - std::cos(length / radius));
    } else if (type == SegmentType::Right) {
      ldy = -(1.0 - std::cos(length / radius));
    }
    auto gdx = std::cos(-start.theta) * ldx + std::sin(-start.theta) * ldy;
    auto gdy = -std::sin(-start.theta) * ldx + std::cos(-start.theta) * ldy;

    res.x = start.x + gdx * radius;
    res.y = start.y + gdy * radius;
    if (type == SegmentType::Left) {
      res.theta = start.theta + length / radius;
    } else if (type == SegmentType::Right) {
      res.theta = start.theta - length / radius;
    }
  }
  res.normalize_theta();
  return res;
}

auto DubinsPlanner::toPath(DubinsPath const &path, Pose const &start) -> Path {

  Path result;
  result.push_back(start);

  auto step_size = 0.1;
  for (auto const &segment : path) {
    auto origin = result.back();
    auto length = step_size;
    while (std::abs(length) <=
           std::abs(segment.normalized_length) * curve_radius_) {
      auto pose = samplePose(origin, length, segment.type, curve_radius_);
      result.push_back(pose);
      length += step_size;
    }
    auto pose =
        samplePose(origin, std::abs(segment.normalized_length) * curve_radius_,
                   segment.type, curve_radius_);
    result.push_back(pose);
  }

  return result;
}
} // namespace curve_planners