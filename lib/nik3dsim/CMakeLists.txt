cmake_minimum_required(VERSION 3.5.0)
project(nik3dsim LANGUAGES CXX)

# Add option for SDL2
option(NIK3DSIM_WITH_SDL2 "Build with SDL2 support" ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20 -mavx -fPIC")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(NIK3DSIM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(NIK3DSIM_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Find SDL2 if enabled
if(NIK3DSIM_WITH_SDL2)
    find_package(SDL2 REQUIRED)
    add_definitions(-DNIK3DSIM_WITH_SDL2)
endif()

# find all source files in the src directory ending in .cpp or .hpp
file(GLOB_RECURSE NIK3DSIM_SRCS 
    ${NIK3DSIM_SOURCE_DIR}/*.cpp 
    ${NIK3DSIM_SOURCE_DIR}/*.hpp 
    ${NIK3DSIM_SOURCE_DIR}/*.h 
    ${NIK3DSIM_SOURCE_DIR}/*.c
)

include_directories("${NIK3DSIM_SOURCE_DIR}")
include_directories("${NIK3DSIM_INCLUDE_DIR}")

# Add SDL2 include directories if enabled
if(NIK3DSIM_WITH_SDL2)
    include_directories(${SDL2_INCLUDE_DIRS})
endif()

# Shared library
add_library(nik3dsim SHARED ${NIK3DSIM_SRCS} ${NIK3DSIM_INCLUDE_DIR})
if(NIK3DSIM_WITH_SDL2)
    target_link_libraries(nik3dsim ${SDL2_LIBRARIES})
endif()

# Static library
add_library(nik3dsim_static STATIC ${NIK3DSIM_SRCS} ${NIK3DSIM_INCLUDE_DIR})
if(NIK3DSIM_WITH_SDL2)
    target_link_libraries(nik3dsim_static ${SDL2_LIBRARIES})
endif()

target_include_directories(nik3dsim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# add tests for make test
# if (NOT DEFINED NIK3DSIM_BUILD_TESTS)
#     set(NIK3DSIM_BUILD_TESTS ON)
# endif()
#
# if (NIK3DSIM_BUILD_TESTS)
#     enable_testing()
#     add_subdirectory(test)
# endif()