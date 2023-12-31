cmake_minimum_required(VERSION 3.6)

set(CMAKE_CONFIGURATION_TYPES Debug Release CACHE TYPE INTERNAL FORCE)

project(exercise_3)

# Options
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../Libs CACHE PATH "Path to lib folder")
set(Eigen3_DIR ${LIBRARY_DIR}/Eigen/share/eigen3/cmake CACHE PATH "Path to installed Eigen")

# Set C++ flags
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
# set(CMAKE_BUILD_TYPE Debug)


find_package(Eigen3 REQUIRED)

# Define header and source files
set(HEADERS
    Eigen.h
    ProcrustesAligner.h
    SimpleMesh.h
)

set(SOURCES
    main.cpp
)

add_executable(exercise_3 ${HEADERS} ${SOURCES})
target_link_libraries(exercise_3 Eigen3::Eigen)

# Visual Studio properties
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT exercise_3)
set_property(TARGET exercise_3 PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/)
