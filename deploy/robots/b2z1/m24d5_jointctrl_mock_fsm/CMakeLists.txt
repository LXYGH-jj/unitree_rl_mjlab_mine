cmake_minimum_required(VERSION 3.16)
project(m24d5_jointctrl_mock_sim LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Threads REQUIRED)
set(BASE "${CMAKE_CURRENT_LIST_DIR}/..")
add_executable(m24d5_sim_gate_test
    tests/test_sim_gate.cpp src/m24d5_sim_gate.cpp
    ${BASE}/m24d4_z1_transport_contract/src/z1_transport_contract.cpp
    ${BASE}/m24d1_z1_transport/src/z1_rx_validation.cpp
    ${BASE}/m24d1_z1_transport/src/z1_transport_session.cpp
    ${BASE}/m24c3_jointctrl_runtime/src/z1_jointctrl_session.cpp
    ${BASE}/m24b2_jointctrl_offline/src/z1_jointctrl_safety.cpp)
target_include_directories(m24d5_sim_gate_test PRIVATE
    include
    ${BASE}/m24d4_z1_transport_contract/include
    ${BASE}/m24d1_z1_transport/include
    ${BASE}/m24c3_jointctrl_runtime/include
    ${BASE}/m24b2_jointctrl_offline/include)
target_link_libraries(m24d5_sim_gate_test PRIVATE Threads::Threads)
enable_testing()
add_test(NAME m24d5_sim_gate COMMAND m24d5_sim_gate_test)
