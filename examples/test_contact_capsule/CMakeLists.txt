cmake_minimum_required (VERSION 3.10.0)
project (test_contact_capsule CXX)
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable(test_contact_capsule main.cpp)

target_link_libraries (test_contact_capsule nik3dsim)