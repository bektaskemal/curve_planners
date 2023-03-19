
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

struct DubinsInputs {
  DubinsInputs() = default;
  DubinsInputs(curve_planners::Pose const &start,
               curve_planners::Pose const &goal, double min_turning_radius) {
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

class DubinsPlanner {
public:
  explicit DubinsPlanner(double min_turning_radius)
      : min_turning_radius_(min_turning_radius) {}

  // call all planners, sort by length, return best not-in-collision-path
  // Otherwise return tl::make_unexpected
  auto plan(curve_planners::Pose const &start, curve_planners::Pose const &goal)
      -> tl::expected<Path, std::string> {
    DubinsInputs inputs(start, goal, min_turning_radius_);
    std::vector<Path> plans;

    auto calc_path_length = [](const Path &path) {
      return std::accumulate(path.begin(), path.end(), 0.0,
                             [](double acc, const Segment &segment) {
                               return acc + std::abs(segment.normalized_length);
                             });
    };
    Path best_plan;
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

private:
  static auto dubins_LSL(DubinsInputs const &inputs)
      -> tl::expected<Path, std::string> {
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

    return Path{{{SegmentType::Left, t},
                 {SegmentType::Straight, p},
                 {SegmentType::Left, q}}};
  }
  static auto dubins_RSR(DubinsInputs const &inputs)
      -> tl::expected<Path, std::string> {

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
    return Path{{{SegmentType::Right, t},
                 {SegmentType::Straight, p},
                 {SegmentType::Right, q}}};
  }
  static auto dubins_LSR(DubinsInputs const &inputs)
      -> tl::expected<Path, std::string> {

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

    return Path{{{SegmentType::Left, t},
                 {SegmentType::Straight, p},
                 {SegmentType::Right, q}}};
  }

  static auto dubins_RSL(DubinsInputs const &inputs)
      -> tl::expected<Path, std::string> {

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

    return Path{{{SegmentType::Right, t},
                 {SegmentType::Straight, p},
                 {SegmentType::Left, q}}};
  }
  static auto dubins_RLR(DubinsInputs const &inputs)
      -> tl::expected<Path, std::string> {

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

    return Path{{{SegmentType::Right, t},
                 {SegmentType::Left, p},
                 {SegmentType::Right, q}}};
  }

  static auto dubins_LRL(DubinsInputs const &inputs)
      -> tl::expected<Path, std::string> {

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

    return Path{{{SegmentType::Left, t},
                 {SegmentType::Right, p},
                 {SegmentType::Left, q}}};
  }

  double min_turning_radius_;
};
} // namespace curve_planners
