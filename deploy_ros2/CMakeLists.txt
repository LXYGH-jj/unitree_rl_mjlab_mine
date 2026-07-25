cmake_minimum_required(VERSION 3.12)
project(go2_rl_mjlab_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(yaml-cpp REQUIRED)

find_path(UNITREE_SDK2_INCLUDE_DIR
  NAMES unitree/dds_wrapper/common/unitree_joystick.hpp
  PATHS
    /usr/local/include
    /usr/include
    /home/vi/lxy/b2+z1/unitree_sdk2/include
)
if(NOT UNITREE_SDK2_INCLUDE_DIR)
  message(FATAL_ERROR "Could not find Unitree SDK2 headers required for UnitreeJoystick.")
endif()

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(ONNXRUNTIME_DIR "${PROJECT_SOURCE_DIR}/../deploy/thirdparty/onnxruntime-linux-x64-1.22.0")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(ONNXRUNTIME_DIR "${PROJECT_SOURCE_DIR}/../deploy/thirdparty/onnxruntime-linux-aarch64-1.22.0")
else()
  message(FATAL_ERROR "Unsupported processor: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

include_directories(
  /usr/include/eigen3
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
  include
  ../deploy/include
  ${UNITREE_SDK2_INCLUDE_DIR}
  ${ONNXRUNTIME_DIR}/include
)

add_executable(go2_ros2_ctrl
  src/main.cpp
  src/State_RLBase.cpp
  src/motor_crc.cpp
)

ament_target_dependencies(go2_ros2_ctrl
  rclcpp
  unitree_go
  unitree_api
)

target_link_libraries(go2_ros2_ctrl
  yaml-cpp
  fmt
  ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so.1.22.0
)

install(TARGETS go2_ros2_ctrl
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
