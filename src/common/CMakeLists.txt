cmake_minimum_required(VERSION 3.0.2)
project(common)

find_package(PkgConfig QUIET)
pkg_check_modules(PC_DEPEND
    eigen3
    sglog
    sgtime
    jsoncpp
)

find_package(OpenCV REQUIRED)

message("depend inc: ${PC_DEPEND_INCLUDE_DIRS}")
message("depend lib: ${PC_DEPEND_LIBRARIES}")


include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include/template
    ${PC_DEPEND_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/src/thirdparty/ooqp/include/ooqp
    ${CMAKE_SOURCE_DIR}/src/thirdparty
)

link_directories(${CMAKE_SOURCE_DIR}/src/thridparty/ooqp/lib)

add_library(${PROJECT_NAME} STATIC
    basics/semantics.cc 
    basics/shapes.cc
    basics/rss_checker.cc
    math/calculations.cc 
    spline/lookup_table.cc
    lane/lane.cc
    state/state_transformer.cc
    solver/ooqp_interface.cc 
    solver/qp_solver.cc
    spline/spline_generator.cc
)

target_link_libraries(${PROJECT_NAME}
    ${PC_DEPEND_LIBRARIES}
    opencv_core
    opencv_highgui
    ooqpgensparse
    ooqpsparse 
    ooqpgondzio 
    ooqpbase blas ma27 gfortran
)