cmake_minimum_required(VERSION 3.0.2)
project(ssc_planner)

find_package(PkgConfig QUIET)
pkg_check_modules(PC_DEPEND
    sglog
    sgtime
    jsoncpp
)

find_package(OpenCV REQUIRED)
include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include/template
    ${CMAKE_SOURCE_DIR}/src/common
)

add_library(${PROJECT_NAME} STATIC
    ssc_map.cc
)

target_link_libraries(${PROJECT_NAME}
    ${SGLog_LIBRARIES}
    common
    opencv_core
    opencv_highgui
)