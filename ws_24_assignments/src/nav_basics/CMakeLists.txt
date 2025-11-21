cmake_minimum_required(VERSION 3.8)
project(nav_basics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc)

add_executable(lifecycle_nodes_client src/lifecycle_nodes_client.cpp)
ament_target_dependencies(lifecycle_nodes_client 
    rclcpp 
    nav2_msgs
    geometry_msgs
    )

add_executable(nav2_action_client src/nav2_action_client.cpp)
ament_target_dependencies(nav2_action_client 
    rclcpp 
    nav2_msgs
    rclcpp_action 
    tf2_ros
    )

add_executable(perception_node src/perception_node.cpp)
ament_target_dependencies(perception_node
    rclcpp
    sensor_msgs    
    geometry_msgs 
    cv_bridge
    tf2
    tf2_ros
    tf2_geometry_msgs
    std_msgs
    )

target_include_directories(perception_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  )

target_link_libraries(perception_node 
    ${OpenCV_LIBS}
    )

install(TARGETS
    lifecycle_nodes_client
    nav2_action_client
    perception_node
  DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY cfg
    DESTINATION share/${PROJECT_NAME}
    )

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    )

ament_package()
