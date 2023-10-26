cmake_minimum_required(VERSION 3.16.3)

project(orca)

include(CheckCXXCompilerFlag)
include(CMakePrintHelpers)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(Eigen3 REQUIRED)
add_subdirectory(external/yaml-cpp)
add_subdirectory(external/fmt)

if (NOT TARGET doctest)
    add_subdirectory(external/doctest)
endif()

add_library(${PROJECT_NAME} STATIC
    include/orca.hpp
    external/kdtree/kdtree.c
    src/orca.cpp
    src/orca_manager.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
    include
    external/kdtree
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    Eigen3::Eigen
    yaml-cpp::yaml-cpp
    fmt::fmt
)

add_custom_command(
    TARGET ${PROJECT_NAME} PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${CMAKE_CURRENT_SOURCE_DIR}/config
    ${CMAKE_CURRENT_BINARY_DIR}/config)

# Test
add_executable(SimulationOrca test/SimulationOrca.cpp)
target_include_directories(SimulationOrca PRIVATE 
    ${DOCTEST_INCLUDE_DIR})
target_link_libraries(SimulationOrca orca doctest)

enable_testing()
add_test(name SimulationOrca command SimulationOrca)