cmake_minimum_required(VERSION 3.10.0)

# Get the name of the directory containing CMakeLists.txt
get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME} CXX)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} nik3dsim)