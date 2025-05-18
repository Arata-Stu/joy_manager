cmake_minimum_required(VERSION 3.8)
project(joy_manager)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(rosbag2_interfaces REQUIRED)
find_package(rosbag2_transport REQUIRED)

add_executable(joy_manager_node src/joy_manager_node.cpp)
ament_target_dependencies(joy_manager_node rclcpp sensor_msgs ackermann_msgs rosbag2_interfaces rosbag2_transport)

install(TARGETS joy_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})  

ament_package()

