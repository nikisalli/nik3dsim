cmake_minimum_required(VERSION 3.10.0)
project(nik3dsim)

include_directories(${CMAKE_SOURCE_DIR}/lib/nik3dsim/include)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

# Add the library first
add_subdirectory(lib/nik3dsim)

# Find and add all example directories
file(GLOB EXAMPLE_DIRS "examples/test*")
foreach(DIR ${EXAMPLE_DIRS})
    if(IS_DIRECTORY ${DIR})
        add_subdirectory(${DIR})
    endif()
endforeach()
