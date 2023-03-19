cmake_minimum_required(VERSION 3.8)
project(curve_planners)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(curve_planners_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tl_expected REQUIRED)

set (dependencies
  angles
  curve_planners_msgs
  rclcpp
  rclcpp_components
  rclcpp_action
  tf2
  tf2_geometry_msgs
  tf2_ros
  tl_expected
)

include_directories(include)


# add_library(${PROJECT_NAME} SHARED 
#         src/dubins_planner.cpp)

# ament_target_dependencies(${library_name}
#   ${dependencies}
# )

add_executable(${PROJECT_NAME}_node src/main.cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # # the following line skips the linter which checks for copyrights
  # # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # # the following line skips cpplint (only works in a git repo)
  # # comment the line when this package is in a git repo and when
  # # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_dubins_test
  test/dubins_test.cpp
  )
  ament_target_dependencies(${PROJECT_NAME}_dubins_test
    ${dependencies}
  )
  # target_link_libraries(${PROJECT_NAME}_dubins_test
  #   ${PROJECT_NAME}
  # )
endif()

ament_package()