cmake_minimum_required(VERSION 3.12)
project(m24e2a_z1_mock_worker LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_executable(m24e2a_z1_mock_worker src/z1_worker_mock.cpp)
target_compile_options(m24e2a_z1_mock_worker PRIVATE -Wall -Wextra -Wpedantic -Werror)
# No vendor headers, libraries, ROS2, DDS, Z1 networking, or motor transports.
