cmake_minimum_required(VERSION 3.8)
project(debug_map)

find_package (ament_cmake_auto REQUIRED) 
find_package(yaml-cpp REQUIRED)
ament_auto_find_build_dependencies ()

ament_auto_add_executable(debug_map debug_map.cpp)
target_link_libraries(debug_map yaml-cpp)

ament_auto_package ()
