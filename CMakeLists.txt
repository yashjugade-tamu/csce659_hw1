cmake_minimum_required(VERSION 3.15)

project(PhysicallyBasedBall)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Raylib paths
set(RAYLIB_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/include")
set(RAYLIB_LIBRARY "${CMAKE_SOURCE_DIR}/lib/libraylib.a")

# Our executable
add_executable(PhysicallyBasedBall
    src/main.cpp
)

# Include raylib headers
target_include_directories(PhysicallyBasedBall PRIVATE
    ${RAYLIB_INCLUDE_DIR}
)

# Link raylib and required Windows libraries
target_link_libraries(PhysicallyBasedBall PRIVATE
    ${RAYLIB_LIBRARY}
    opengl32
    gdi32
    winmm
    user32
    shell32
)