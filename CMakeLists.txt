cmake_minimum_required(VERSION 3.16)
project(RuleGeometryEngine VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(cmake/rge_options.cmake)

add_library(rge
    src/geom/polygon_view.cpp
    src/spatial/iou.cpp
    src/spatial/contains.cpp
    src/spatial/distance.cpp
    src/spatial/intersects.cpp
    src/spatial/side_of_line.cpp
    src/spatial/polygon_ops.cpp
)

target_include_directories(rge
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

rge_set_target_options(rge)

option(RGE_BUILD_TESTS "Build unit tests" ON)
if(RGE_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

option(RGE_BUILD_EXAMPLES "Build examples" ON)
if(RGE_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
