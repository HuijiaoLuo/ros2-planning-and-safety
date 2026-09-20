cmake_minimum_required(VERSION 3.20)

project(robotics_planning_cpp VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ROBOTICS_PLANNING_ENABLE_OPENMP "Enable OpenMP support for future kernels" OFF)

add_library(robotics_planning
    src/grid.cpp
    src/planners.cpp
    src/benchmark.cpp
)

target_include_directories(robotics_planning
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(MSVC)
    target_compile_options(robotics_planning PRIVATE /W4 /permissive-)
else()
    target_compile_options(robotics_planning PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(ROBOTICS_PLANNING_ENABLE_OPENMP)
    find_package(OpenMP REQUIRED)
    target_link_libraries(robotics_planning PUBLIC OpenMP::OpenMP_CXX)
    target_compile_definitions(robotics_planning PUBLIC ROBOTICS_PLANNING_HAS_OPENMP=1)
endif()

add_executable(planning_demo apps/planning_demo.cpp)
target_link_libraries(planning_demo PRIVATE robotics_planning)

add_executable(planning_tests tests/planning_tests.cpp)
target_link_libraries(planning_tests PRIVATE robotics_planning)

enable_testing()
add_test(NAME planning_tests COMMAND planning_tests)

