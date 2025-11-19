cmake_minimum_required(VERSION 3.8)
project(nav_basics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(lifecycle_nodes_client src/lifecycle_nodes_client.cpp)
ament_target_dependencies(lifecycle_nodes_client rclcpp nav2_msgs geometry_msgs )

add_executable(nav2_action_client src/nav2_action_client.cpp)
ament_target_dependencies(nav2_action_client rclcpp nav2_msgs rclcpp_action )


install(TARGETS
    nav_node
    lifecycle_nodes_client
    nav2_action_client
  DESTINATION lib/${PROJECT_NAME})


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
