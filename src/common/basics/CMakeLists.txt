cmake_minimum_required(VERSION 3.0.2)
project(common)

find_package(PkgConfig QUIET)
pkg_check_modules(PC_DEPEND
    eigen3
    sglog
    sgtime
    jsoncpp
)


message("depend inc: ${PC_DEPEND_INCLUDE_DIRS}")
message("depend lib: ${PC_DEPEND_LIBRARIES}")

include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include/template
    ${PC_DEPEND_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} STATIC
    semantics.cc 
)

target_link_libraries(${PROJECT_NAME}
    ${PC_DEPEND_LIBRARIES}
)