cmake_minimum_required(VERSION 3.8)
project(pinky2_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

add_executable(auto_parking_node src/auto_parking_node.cpp)
ament_target_dependencies(auto_parking_node
  rclcpp rclcpp_action std_msgs geometry_msgs nav_msgs sensor_msgs nav2_msgs
)

install(TARGETS
  auto_parking_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
