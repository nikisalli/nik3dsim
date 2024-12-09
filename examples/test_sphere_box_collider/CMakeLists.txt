cmake_minimum_required(VERSION 3.10.0)
project(test_sphere_box_collider CXX)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# Find required packages
find_package(SDL2 REQUIRED)
find_package(OpenGL REQUIRED)

# Add executable
add_executable(test_sphere_box_collider main.cpp)

# Include directories
target_include_directories(test_sphere_box_collider PRIVATE
    ${SDL2_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
)

# Link libraries
target_link_libraries(test_sphere_box_collider
    ${SDL2_LIBRARIES}
    ${OPENGL_LIBRARIES}
    GLU
)

# If your code is part of the nik3dsim library:
target_link_libraries(test_sphere_box_collider nik3dsim)