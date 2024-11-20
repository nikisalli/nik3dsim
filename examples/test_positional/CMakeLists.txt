cmake_minimum_required (VERSION 3.10.0)
project (test_positional CXX)
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable(test_positional main.cpp)

target_link_libraries (test_positional nik3dsim)