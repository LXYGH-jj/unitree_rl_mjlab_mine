cmake_minimum_required(VERSION 3.16)
project(m24d4_mock_only_contract LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(PARENT ${ROOT}/..)
add_executable(m24d4_mock_contract_test
  tests/test_mock_contract.cpp
  src/z1_transport_contract.cpp
  ${PARENT}/m24b2_jointctrl_offline/src/z1_jointctrl_safety.cpp
  ${PARENT}/m24c3_jointctrl_runtime/src/z1_jointctrl_session.cpp
  ${PARENT}/m24d1_z1_transport/src/z1_rx_validation.cpp
  ${PARENT}/m24d1_z1_transport/src/z1_transport_session.cpp
)
target_include_directories(m24d4_mock_contract_test PRIVATE
  include
  ${PARENT}/m24b2_jointctrl_offline/include
  ${PARENT}/m24c3_jointctrl_runtime/include
  ${PARENT}/m24d1_z1_transport/include
)
target_compile_options(m24d4_mock_contract_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
find_package(Threads REQUIRED)
target_link_libraries(m24d4_mock_contract_test PRIVATE Threads::Threads)
enable_testing()
add_test(NAME m24d4_mock_contract COMMAND m24d4_mock_contract_test)
set_tests_properties(m24d4_mock_contract PROPERTIES TIMEOUT 10)
