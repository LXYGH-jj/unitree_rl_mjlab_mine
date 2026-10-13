cmake_minimum_required(VERSION 3.12)
project(b2z1_controller)

set(CMAKE_CXX_STANDARD 17)
set(B2Z1_DEPLOY_ROOT "${PROJECT_SOURCE_DIR}/../..")
set(B2Z1_THIRDPARTY_DIR "${B2Z1_DEPLOY_ROOT}/thirdparty")

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)
find_package(ZLIB REQUIRED)

set(UNITREE_SDK2_INCLUDE_DIR "$ENV{UNITREE_SDK2_INCLUDE_DIR}")
if(NOT UNITREE_SDK2_INCLUDE_DIR AND DEFINED ENV{UNITREE_SDK2_ROOT})
    set(UNITREE_SDK2_INCLUDE_DIR "$ENV{UNITREE_SDK2_ROOT}/include")
endif()
if(UNITREE_SDK2_INCLUDE_DIR)
    message(STATUS "Using UNITREE_SDK2_INCLUDE_DIR=${UNITREE_SDK2_INCLUDE_DIR}")
endif()

set(Z1_SDK_ROOT "$ENV{Z1_SDK_ROOT}")
if(NOT Z1_SDK_ROOT)
    if(EXISTS "${B2Z1_THIRDPARTY_DIR}/z1_sdk/include/unitree_arm_sdk/control/unitreeArm.h")
        set(Z1_SDK_ROOT "${B2Z1_THIRDPARTY_DIR}/z1_sdk")
    endif()
endif()
if(NOT Z1_SDK_ROOT)
    message(FATAL_ERROR
        "Z1 SDK root not found. Put it under deploy/thirdparty/z1_sdk or set Z1_SDK_ROOT.")
endif()

set(Z1_SDK_LIB "$ENV{Z1_SDK_LIB}")
if(NOT Z1_SDK_LIB)
    set(Z1_SDK_LIB "${Z1_SDK_ROOT}/lib/libZ1_SDK_${CMAKE_SYSTEM_PROCESSOR}.so")
endif()
if(NOT EXISTS "${Z1_SDK_LIB}")
    message(FATAL_ERROR
        "Z1 SDK library not found. Put libZ1_SDK_${CMAKE_SYSTEM_PROCESSOR}.so under deploy/thirdparty/z1_sdk/lib or set Z1_SDK_LIB.")
endif()
message(STATUS "Using Z1_SDK_ROOT=${Z1_SDK_ROOT}")
message(STATUS "Using Z1_SDK_LIB=${Z1_SDK_LIB}")

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set(ONNXRUNTIME_DIR "${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(ONNXRUNTIME_DIR "${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-aarch64-1.22.0")
endif()

include_directories(
  /usr/include/eigen3
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
  ${UNITREE_SDK2_INCLUDE_DIR}
  ${Z1_SDK_ROOT}/include
  ${PROJECT_SOURCE_DIR}/../../interactive_b2z1/include
  ${PROJECT_SOURCE_DIR}/../../interactive_b2z1/sim_closed_loop
  include
  ${ONNXRUNTIME_DIR}/include
  ${PROJECT_SOURCE_DIR}/../../include/
  ${PROJECT_SOURCE_DIR}/../../thirdparty/cnpy
)

file(GLOB CNPY_SRC ${PROJECT_SOURCE_DIR}/../../thirdparty/cnpy/*.cpp)

add_library(cnpy_lib STATIC ${CNPY_SRC})

link_libraries(
  unitree_sdk2
  ddsc
  ddscxx
  rt
  pthread
  Boost::program_options
  yaml-cpp
  fmt
  ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so.1.22.0
  ${Z1_SDK_LIB}
  cnpy_lib
  ZLIB::ZLIB
)

file(GLOB_RECURSE ADD_SRC_LIST
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})
# Reuse the parity-tested Interactive observation and ONNX adapter.
# No duplicate SDK2 channel / FSM / keyboard is built here.
target_sources(${PROJECT_NAME}_lib PRIVATE
  ${B2Z1_DEPLOY_ROOT}/interactive_b2z1/src/interactive_observation.cpp
  ${B2Z1_DEPLOY_ROOT}/interactive_b2z1/src/interactive_feedback_adapter.cpp
  ${B2Z1_DEPLOY_ROOT}/interactive_b2z1/src/interactive_pipeline.cpp
  ${B2Z1_DEPLOY_ROOT}/interactive_b2z1/src/interactive_snapshot_store.cpp
  ${B2Z1_DEPLOY_ROOT}/interactive_b2z1/sim_closed_loop/joint_target_guard.cpp
)
target_compile_definitions(${PROJECT_NAME}_lib PUBLIC
  B2Z1_DEPLOY_ROOT="${B2Z1_DEPLOY_ROOT}"
  UNITREE_RL_MJLAB_ENABLE_Z1_SDK
)
link_libraries(${PROJECT_NAME}_lib)

add_executable(b2z1_ctrl main.cpp)
add_executable(b2z1_infer_joint_map tools/infer_joint_map.cpp)


# M1.9a: OFFLINE Z1 SDK kinematics probe (no LowCmd, DDS or motors)
add_executable(b2z1_z1_kin_probe tools/z1_kin_probe.cpp)
target_include_directories(b2z1_z1_kin_probe PRIVATE ${Z1_SDK_ROOT}/include)
target_link_libraries(b2z1_z1_kin_probe PRIVATE ${Z1_SDK_LIB})
