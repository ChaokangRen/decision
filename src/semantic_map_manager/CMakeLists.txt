cmake_minimum_required(VERSION 3.0.2)
project(semantic_manager)

find_package(PkgConfig QUIET)
pkg_check_modules(PC_DEPEND
    sglog
    sgtime
)

find_package(OpenCV REQUIRED)
include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include/template
    ${CMAKE_SOURCE_DIR}/src/common
    ${CMAKE_SOURCE_DIR}/src/thirdparty/json
)

add_library(${PROJECT_NAME} STATIC
    config_loader.cc
    semantic_map_manager.cc
)

target_link_libraries(${PROJECT_NAME}
    ${SGLog_LIBRARIES}
    common
    opencv_core
    opencv_highgui
)