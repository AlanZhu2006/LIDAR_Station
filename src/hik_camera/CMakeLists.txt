cmake_minimum_required(VERSION 3.8)
project(hik_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# MVS SDK
set(MVS_SDK_PATH "/opt/MVS")
include_directories(${MVS_SDK_PATH}/include)
link_directories(${MVS_SDK_PATH}/lib/64)

add_executable(hik_camera_node src/hik_camera_node.cpp)

target_include_directories(hik_camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(hik_camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

target_link_libraries(hik_camera_node
  MvCameraControl
  ${OpenCV_LIBS}
)

install(TARGETS hik_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
