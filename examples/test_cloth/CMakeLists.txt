cmake_minimum_required (VERSION 3.10.0)
project (test_cloth CXX)
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable(test_cloth main.cpp)

target_link_libraries (test_cloth nik3dsim)