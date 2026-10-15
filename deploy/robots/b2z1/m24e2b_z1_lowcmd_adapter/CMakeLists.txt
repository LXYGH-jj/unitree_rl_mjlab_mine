cmake_minimum_required(VERSION 3.12)
project(m24e2b_z1_lowcmd_adapter LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_library(m24e2b_planner STATIC src/z1_lowcmd_plan.cpp)
target_include_directories(m24e2b_planner PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(m24e2b_planner PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_executable(m24e2b_planner_test tests/test_plan.cpp)
target_link_libraries(m24e2b_planner_test PRIVATE m24e2b_planner)
enable_testing()
add_test(NAME pure_planner COMMAND m24e2b_planner_test)
# NOTE: No SDK adapter is linked into a program at this milestone.
# SDK header compile is done explicitly with -fsyntax-only, never SDK construction.
