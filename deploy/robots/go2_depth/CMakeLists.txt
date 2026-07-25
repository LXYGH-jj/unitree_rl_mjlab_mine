cmake_minimum_required(VERSION 3.12)
project(go2_depth_controller)

option(ENABLE_ROS2_DEPTH_PROVIDER "Enable ROS2 depth image provider for go2_depth" OFF)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)
if(ENABLE_ROS2_DEPTH_PROVIDER)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
    add_compile_definitions(ISAACLAB_ENABLE_ROS2_DEPTH_PROVIDER)
endif()

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set(ONNXRUNTIME_DIR "${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(ONNXRUNTIME_DIR "${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-aarch64-1.22.0")
endif()

include_directories(
  /usr/include/eigen3
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
  include
  ${ONNXRUNTIME_DIR}/include
  ${PROJECT_SOURCE_DIR}/../../include/
)

link_libraries(
  unitree_sdk2 ddsc ddscxx rt pthread
  libboost_program_options.a libyaml-cpp.a fmt
  ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so.1.22.0
)

file(GLOB_RECURSE ADD_SRC_LIST
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

add_library(go2_depth_controller_lib ${ADD_SRC_LIST})
if(ENABLE_ROS2_DEPTH_PROVIDER)
    ament_target_dependencies(go2_depth_controller_lib rclcpp sensor_msgs)
endif()
link_libraries(go2_depth_controller_lib)

add_executable(go2_depth_ctrl main.cpp)
if(ENABLE_ROS2_DEPTH_PROVIDER)
    ament_target_dependencies(go2_depth_ctrl rclcpp sensor_msgs)
endif()

add_executable(depth_camera_debug_test tools/depth_camera_debug_test.cpp)
if(ENABLE_ROS2_DEPTH_PROVIDER)
    ament_target_dependencies(depth_camera_debug_test rclcpp sensor_msgs)
endif()
