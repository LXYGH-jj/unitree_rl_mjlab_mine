cmake_minimum_required(VERSION 3.16)
project(m24d2_z1_loopback_lab LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Intentionally standalone: MUST NOT link to physical b2z1_ctrl, vendor SDK,
# DDS libraries, or any physical Z1 command sender.
set(B2Z1_DIR "${CMAKE_CURRENT_LIST_DIR}/..")
set(SAFETY "${B2Z1_DIR}/m24b2_jointctrl_offline")
set(SESSION "${B2Z1_DIR}/m24c3_jointctrl_runtime")
set(TRANSPORT "${B2Z1_DIR}/m24d1_z1_transport")
add_executable(m24d2_loopback_test
    tests/test_loopback.cpp
    src/z1_loopback_rx_lab.cpp
    "${SAFETY}/src/z1_jointctrl_safety.cpp"
    "${SESSION}/src/z1_jointctrl_session.cpp"
    "${TRANSPORT}/src/z1_rx_validation.cpp"
    "${TRANSPORT}/src/z1_transport_session.cpp"
)
target_include_directories(m24d2_loopback_test PRIVATE
    include "${SAFETY}/include" "${SESSION}/include" "${TRANSPORT}/include")
target_compile_options(m24d2_loopback_test PRIVATE -Wall -Wextra -Werror)
find_package(Threads REQUIRED)
target_link_libraries(m24d2_loopback_test PRIVATE Threads::Threads)
enable_testing()
add_test(NAME m24d2_loopback COMMAND m24d2_loopback_test)
