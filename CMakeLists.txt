cmake_minimum_required(VERSION 3.10.0)
project(nik3dsim)

include_directories(${CMAKE_SOURCE_DIR}/lib/nik3dsim/include)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

add_subdirectory(lib/nik3dsim)
add_subdirectory(examples/test)
add_subdirectory(examples/test_hinge)
add_subdirectory(examples/test_positional)
add_subdirectory(examples/test_positional_bench)