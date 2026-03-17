cmake_minimum_required(VERSION 3.8)
project(pointcloud_transform)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_executable(pointcloud_transform_node src/pointcloud_transform_node.cpp)
ament_target_dependencies(pointcloud_transform_node rclcpp sensor_msgs geometry_msgs tf2_ros tf2_sensor_msgs)

install(TARGETS pointcloud_transform_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
