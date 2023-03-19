#include <gtest/gtest.h>

#include "curve_planners/dubins_planner.h"

TEST(DubinsTest, FirstTest) {
  curve_planners::Pose start{0, 0, 0};
  curve_planners::Pose goal{-1, -2, 1.57};
  double high_radius = 15.0;
  curve_planners::DubinsPlanner planner1{high_radius};
  auto res1 = planner1.plan(start, goal);
  EXPECT_EQ(res1.has_value(), true);
  double path_length{};
  for (auto const &p : res1.value()) {
    path_length += std::abs(p.normalized_length);
    std::cerr << p.normalized_length << std::endl;
  }
  std::cerr << "Path length: " << path_length * high_radius << std::endl;
  double low_radius = 0.2;
  curve_planners::DubinsPlanner planner2{low_radius};
  auto res2 = planner2.plan(start, goal);
  EXPECT_EQ(res2.has_value(), true);

  path_length = 0;
  for (auto const &p : res2.value()) {
    path_length += std::abs(p.normalized_length);
    std::cerr << p.normalized_length << std::endl;
  }
  std::cerr << "Path length: " << path_length * low_radius << std::endl;
}