cmake_minimum_required (VERSION 3.10.0)
project (test_collision_bench CXX)
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable(test_collision_bench main.cpp)

target_link_libraries (test_collision_bench nik3dsim stdc++)