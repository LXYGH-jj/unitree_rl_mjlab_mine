cmake_minimum_required(VERSION 3.16)
project(b2z1_interactive_sim_closed_loop LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
get_filename_component(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(DEPLOY_ROOT "${REPO_ROOT}/deploy")
set(IB2Z1 "${DEPLOY_ROOT}/interactive_b2z1")
if(NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  message(FATAL_ERROR "M1.5 SIM-only test restricted to x86_64 PC; refuse embedded/hardware builds")
endif()
set(ORT "${DEPLOY_ROOT}/thirdparty/onnxruntime-linux-x64-1.22.0")
if(NOT EXISTS "${ORT}/lib/libonnxruntime.so.1.22.0")
  message(FATAL_ERROR "ONNX runtime 1.22 not found")
endif()
find_library(UNITREE_SDK2_LIBRARY NAMES unitree_sdk2 REQUIRED)
find_library(DDS_C_LIBRARY NAMES ddsc PATHS /usr/local/lib NO_DEFAULT_PATH REQUIRED)
find_library(DDS_CXX_LIBRARY NAMES ddscxx PATHS /usr/local/lib NO_DEFAULT_PATH REQUIRED)
find_package(Threads REQUIRED)
find_package(fmt REQUIRED)
add_executable(b2z1_interactive_sim_closed_loop
  main.cpp joint_target_guard.cpp
  "${IB2Z1}/src/interactive_observation.cpp"
  "${IB2Z1}/src/interactive_feedback_adapter.cpp"
  "${IB2Z1}/src/interactive_snapshot_store.cpp"
  "${IB2Z1}/src/interactive_pipeline.cpp")
target_include_directories(b2z1_interactive_sim_closed_loop PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}" "${IB2Z1}/include" "${DEPLOY_ROOT}/include"
  "${ORT}/include" /usr/local/include/ddscxx /usr/local/include/iceoryx/v2.0.2)
if(DEFINED ENV{UNITREE_SDK2_INCLUDE_DIR} AND NOT "$ENV{UNITREE_SDK2_INCLUDE_DIR}" STREQUAL "")
  target_include_directories(b2z1_interactive_sim_closed_loop PRIVATE "$ENV{UNITREE_SDK2_INCLUDE_DIR}")
endif()
target_link_libraries(b2z1_interactive_sim_closed_loop PRIVATE
  "${ORT}/lib/libonnxruntime.so.1.22.0"
  "${UNITREE_SDK2_LIBRARY}" "${DDS_C_LIBRARY}" "${DDS_CXX_LIBRARY}"
  Threads::Threads fmt::fmt rt)
set_target_properties(b2z1_interactive_sim_closed_loop PROPERTIES
  BUILD_RPATH "${ORT}/lib;/usr/local/lib")
target_compile_options(b2z1_interactive_sim_closed_loop PRIVATE -Wall -Wextra -Wpedantic)

add_executable(test_m15_target_guard tests/test_joint_target_guard.cpp joint_target_guard.cpp)
target_include_directories(test_m15_target_guard PRIVATE "${IB2Z1}/include" "${CMAKE_CURRENT_LIST_DIR}")
target_compile_options(test_m15_target_guard PRIVATE -Wall -Wextra -Wpedantic)
enable_testing()
add_test(NAME m15_target_guard COMMAND test_m15_target_guard)

# M1.5m: read-only DDS telemetry; SIM-only on loopback. No LowCmd publisher.
add_executable(b2z1_sim_state_logger sim_state_logger.cpp)
target_include_directories(b2z1_sim_state_logger PRIVATE
  /usr/local/include/ddscxx /usr/local/include/iceoryx/v2.0.2)
if(DEFINED ENV{UNITREE_SDK2_INCLUDE_DIR} AND NOT "$ENV{UNITREE_SDK2_INCLUDE_DIR}" STREQUAL "")
  target_include_directories(b2z1_sim_state_logger PRIVATE "$ENV{UNITREE_SDK2_INCLUDE_DIR}")
endif()
target_link_libraries(b2z1_sim_state_logger PRIVATE
  "${UNITREE_SDK2_LIBRARY}" "${DDS_C_LIBRARY}" "${DDS_CXX_LIBRARY}"
  Threads::Threads fmt::fmt rt)
set_target_properties(b2z1_sim_state_logger PROPERTIES BUILD_RPATH "/usr/local/lib")
target_compile_options(b2z1_sim_state_logger PRIVATE -Wall -Wextra -Wpedantic)
