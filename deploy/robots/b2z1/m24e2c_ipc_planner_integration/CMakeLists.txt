cmake_minimum_required(VERSION 3.16)
project(m24e2c_ipc_planner_integration LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(B2B "${CMAKE_CURRENT_LIST_DIR}/../m24e2b_z1_lowcmd_adapter")
if(NOT EXISTS "${B2B}/include/z1_lowcmd_plan.h" OR NOT EXISTS "${B2B}/src/z1_lowcmd_plan.cpp")
  message(FATAL_ERROR "M2.4e2b planner missing beside M2.4e2c")
endif()
add_executable(m24e2c_z1_integrated_mock_worker
  src/z1_worker_integrated.cpp
  "${B2B}/src/z1_lowcmd_plan.cpp")
target_include_directories(m24e2c_z1_integrated_mock_worker PRIVATE "${B2B}/include")
target_compile_options(m24e2c_z1_integrated_mock_worker PRIVATE -Wall -Wextra -Wpedantic -Werror)
