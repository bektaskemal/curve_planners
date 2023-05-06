cmake_minimum_required(VERSION 3.8)
project(curve_planners)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

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

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED 
        src/dubins_planner.cpp)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(${PROJECT_NAME}_node src/main.cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})
target_link_libraries(${PROJECT_NAME}_node ${library_name})

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${library_name} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
        scripts/rviz_relay.py scripts/rviz_relay_wp.py 
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})


# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   ament_add_gtest(${PROJECT_NAME}_dubins_test
#   test/dubins_test.cpp
#   )
#   ament_target_dependencies(${PROJECT_NAME}_dubins_test
#     ${dependencies}
#   )
#   target_link_libraries(${PROJECT_NAME}_dubins_test
#     ${PROJECT_NAME}
#   )
# endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
