cmake_minimum_required(VERSION 3.1)
project(FrenetTrajectoryOptimization)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "COMPILING FrenetTrajectoryOptimization")

find_package(PythonLibs 2.7 REQUIRED)
find_package(Eigen3 REQUIRED)

file(GLOB ${PROJECT_NAME}_SRC_FILES
    "src/*.cpp") 

add_library(Lib${PROJECT_NAME} STATIC
    ${${PROJECT_NAME}_SRC_FILES}
    "${PROJECT_SOURCE_DIR}/3rdparty/matplotlib-cpp/matplotlibcpp.h"
    "${PROJECT_SOURCE_DIR}/3rdparty/json.hpp"
    )

set(${PROJECT_NAME}_REQUIRED_LIBRARIES
    Eigen3::Eigen
    ${PYTHON_LIBRARIES})    


target_link_libraries(Lib${PROJECT_NAME} ${${PROJECT_NAME}_REQUIRED_LIBRARIES})
target_include_directories(Lib${PROJECT_NAME} 
    PRIVATE ${PYTHON_INCLUDE_DIRS}
    PUBLIC "${PROJECT_SOURCE_DIR}/include"
    PUBLIC "${PROJECT_SOURCE_DIR}/3rdparty/"
)

include_directories("${PROJECT_SOURCE_DIR}/3rdparty/" ${EIGEN3_INCLUDE_DIR} ${PYTHON_INCLUDE_DIRS})
add_subdirectory("scripts")