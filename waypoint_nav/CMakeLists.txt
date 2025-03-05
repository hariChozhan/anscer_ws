cmake_minimum_required(VERSION 3.8)
project(waypoint_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)  # Add this line
find_package(anscer_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(waypoint_recorder src/waypoint_recorder.cpp)
ament_target_dependencies(waypoint_recorder anscer_msgs rclcpp nav_msgs geometry_msgs)

add_executable(waypoint_planner src/waypoint_planner.cpp)
ament_target_dependencies(waypoint_planner tf2_geometry_msgs tf2 rclcpp rclcpp_action nav2_msgs geometry_msgs)

target_link_libraries(waypoint_planner yaml-cpp)

install(TARGETS
  waypoint_recorder
  waypoint_planner
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install custom messages & services
# install(DIRECTORY msg srv
#   DESTINATION share/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
