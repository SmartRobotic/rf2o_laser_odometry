cmake_minimum_required(VERSION 3.4...3.18)
project(CLaserOdometry2D)

find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)

set (PYBIND11_PYTHON_VERSION "3.12")

# Add all relevant source files here
add_library(CLaserOdometry2D MODULE src/pywrap.cpp src/CLaserOdometry2D.cpp)

# Link against pybind11 and Eigen
target_link_libraries(CLaserOdometry2D PRIVATE pybind11::module Eigen3::Eigen)

# Include directories for pybind11 and Eigen
target_include_directories(CLaserOdometry2D PRIVATE ${PYBIND11_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# Set target properties for Python module
set_target_properties(CLaserOdometry2D PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}" SUFFIX "${PYTHON_MODULE_EXTENSION}")
