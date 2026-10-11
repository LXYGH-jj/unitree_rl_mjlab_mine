cmake_minimum_required(VERSION 3.16)
project(b2z1_interactive_shadow_sim LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

get_filename_component(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(DEPLOY_ROOT "${REPO_ROOT}/deploy")
set(IB2Z1_ROOT "${DEPLOY_ROOT}/interactive_b2z1")

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(ORT_DIR "${DEPLOY_ROOT}/thirdparty/onnxruntime-linux-x64-1.22.0")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(ORT_DIR "${DEPLOY_ROOT}/thirdparty/onnxruntime-linux-aarch64-1.22.0")
else()
  message(FATAL_ERROR "Unsupported CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()
if(NOT EXISTS "${ORT_DIR}/include/onnxruntime_cxx_api.h")
  message(FATAL_ERROR "ONNX Runtime headers not found: ${ORT_DIR}")
endif()
find_library(UNITREE_SDK2_LIBRARY NAMES unitree_sdk2)
find_library(DDS_C_LIBRARY NAMES ddsc PATHS /usr/local/lib NO_DEFAULT_PATH)
find_library(DDS_CXX_LIBRARY NAMES ddscxx PATHS /usr/local/lib NO_DEFAULT_PATH)
foreach(REQUIRED_LIB UNITREE_SDK2_LIBRARY DDS_C_LIBRARY DDS_CXX_LIBRARY)
  if(NOT ${REQUIRED_LIB})
    message(FATAL_ERROR "Missing library: ${REQUIRED_LIB}; verify Unitree SDK2 installation")
  endif()
endforeach()
find_package(Threads REQUIRED)
find_package(fmt REQUIRED)
message(STATUS "Shadow DDS C=${DDS_C_LIBRARY}; CXX=${DDS_CXX_LIBRARY}")

add_executable(b2z1_interactive_shadow_sim
  main.cpp
  "${IB2Z1_ROOT}/src/interactive_observation.cpp"
  "${IB2Z1_ROOT}/src/interactive_feedback_adapter.cpp"
  "${IB2Z1_ROOT}/src/interactive_snapshot_store.cpp"
  "${IB2Z1_ROOT}/src/interactive_pipeline.cpp"
)
target_include_directories(b2z1_interactive_shadow_sim PRIVATE
  "${IB2Z1_ROOT}/include"
  "${DEPLOY_ROOT}/include"
  "${ORT_DIR}/include"
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
)
if(DEFINED ENV{UNITREE_SDK2_INCLUDE_DIR} AND NOT "$ENV{UNITREE_SDK2_INCLUDE_DIR}" STREQUAL "")
  target_include_directories(b2z1_interactive_shadow_sim PRIVATE "$ENV{UNITREE_SDK2_INCLUDE_DIR}")
endif()
target_link_libraries(b2z1_interactive_shadow_sim PRIVATE
  "${ORT_DIR}/lib/libonnxruntime.so.1.22.0"
  "${UNITREE_SDK2_LIBRARY}"
  "${DDS_C_LIBRARY}"
  "${DDS_CXX_LIBRARY}"
  Threads::Threads
  fmt::fmt
  rt
)
set_target_properties(b2z1_interactive_shadow_sim PROPERTIES
  BUILD_RPATH "${ORT_DIR}/lib;/usr/local/lib"
)
target_compile_options(b2z1_interactive_shadow_sim PRIVATE -Wall -Wextra -Wpedantic)
