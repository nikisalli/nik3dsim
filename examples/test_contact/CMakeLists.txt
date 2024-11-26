cmake_minimum_required (VERSION 3.10.0)
project (test_contact CXX)
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable(test_contact main.cpp)

target_link_libraries (test_contact nik3dsim)