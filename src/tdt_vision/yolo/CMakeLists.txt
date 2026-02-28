cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0079 NEW)

project(yolo)
add_definitions(-std=c++11)
add_definitions(-DAPI_EXPORTS)
option(CUDA_USE_STATIC_CUDA_RUNTIME ON)
set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
# TODO(Call for PR): make cmake compatible with Windows
find_package(CUDA)
include(../cmake/modules/libnvinferConfig.cmake)

# include and link dirs of cuda and tensorrt, you need adapt them if yours are different
# cuda
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${nvinfer_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)

# Find nvonnxparser library
find_library(nvonnxparser_LIBRARY 
    NAMES nvonnxparser nvonnxparser.so.10
    PATHS /usr/lib/x86_64-linux-gnu /usr/lib /usr/local/lib
    NO_DEFAULT_PATH)
if(NOT nvonnxparser_LIBRARY)
    if(EXISTS "/usr/lib/x86_64-linux-gnu/libnvonnxparser.so.10")
        set(nvonnxparser_LIBRARY "/usr/lib/x86_64-linux-gnu/libnvonnxparser.so.10")
    endif()
endif()
message(STATUS "nvonnxparser library: ${nvonnxparser_LIBRARY}")

file(GLOB_RECURSE SRCS ${PROJECT_SOURCE_DIR}/*.cpp ${PROJECT_SOURCE_DIR}/*.cu)
add_library(yolo  ${SRCS})
target_link_libraries(yolo ${nvinfer_LIBRARY} cudart ${nvonnxparser_LIBRARY} ${OpenCV_LIBS})