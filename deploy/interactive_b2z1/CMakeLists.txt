cmake_minimum_required(VERSION 3.12)
project(interactive_b2z1_obs_parity LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(interactive_observation STATIC src/interactive_observation.cpp)
target_include_directories(interactive_observation PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(interactive_observation PRIVATE -Wall -Wextra -Wpedantic)

add_executable(interactive_obs_fixture tests/obs_fixture.cpp)
target_link_libraries(interactive_obs_fixture PRIVATE interactive_observation)

enable_testing()
add_test(NAME obs_fixture_selftest COMMAND interactive_obs_fixture --selftest)
