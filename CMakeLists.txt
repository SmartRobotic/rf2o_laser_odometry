cmake_minimum_required(VERSION 3.4...3.18)
project(CLaserOdometry2D)

find_package(Python3 3.12 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)

# Add all relevant source files here
pybind11_add_module(CLaserOdometry2D MODULE src/pywrap.cpp src/CLaserOdometry2D.cpp)

# Link against pybind11 and Eigen
target_link_libraries(CLaserOdometry2D PRIVATE pybind11::module Eigen3::Eigen)

# Include directories for pybind11 and Eigen
target_include_directories(CLaserOdometry2D PRIVATE ${PYBIND11_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# Set target properties for Python module
set_target_properties(CLaserOdometry2D PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}" SUFFIX "${PYTHON_MODULE_EXTENSION}")

# Get the Python site-packages directory
execute_process(
    COMMAND ${Python3_EXECUTABLE} -c "import site; print(site.getsitepackages()[0])"
    OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Install the module to the Python site-packages directory
install(TARGETS CLaserOdometry2D DESTINATION ${PYTHON_SITE_PACKAGES})