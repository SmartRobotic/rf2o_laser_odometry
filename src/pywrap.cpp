// bindings.cpp

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>  // For Eigen support
#include "../include/rf2o_laser_odometry/CLaserOdometry2D.h" // Include your header file

namespace py = pybind11;
using namespace rf2o;

PYBIND11_MODULE(CLaserOdometry2D, m) {
    // Expose the LaserScan struct
    py::class_<LaserScan>(m, "LaserScan")
        .def(py::init<>())
        .def_readwrite("ranges", &LaserScan::ranges)
        .def_readwrite("angle_min", &LaserScan::angle_min)
        .def_readwrite("angle_max", &LaserScan::angle_max)
        .def_readwrite("angle_increment", &LaserScan::angle_increment);

    // Expose the Pose struct and its nested Orientation struct
    py::class_<Pose::Orientation>(m, "Orientation")
        .def(py::init<>())
        .def_readwrite("w", &Pose::Orientation::w)
        .def_readwrite("x", &Pose::Orientation::x)
        .def_readwrite("y", &Pose::Orientation::y)
        .def_readwrite("z", &Pose::Orientation::z);

    py::class_<Pose>(m, "Pose")
        .def(py::init<>())
        .def_readwrite("x", &Pose::x)
        .def_readwrite("y", &Pose::y)
        .def_readwrite("theta", &Pose::theta)
        .def_readwrite("orientation", &Pose::orientation);

    // Expose the CLaserOdometry2D class
    py::class_<CLaserOdometry2D>(m, "CLaserOdometry2D")
        .def(py::init<>())
        .def("testab", &CLaserOdometry2D::testab)
        .def("init", &CLaserOdometry2D::init)
        .def("is_initialized", &CLaserOdometry2D::is_initialized)
        .def("odometryCalculation", &CLaserOdometry2D::odometryCalculation)
        .def("setLaserPose", &CLaserOdometry2D::setLaserPose)
        .def("getIncrement", [](const CLaserOdometry2D& self) {
            // Convert Eigen::Isometry3d to Python-friendly format
            Eigen::Isometry3d increment = self.getIncrement();
            
            
            // Extract translation and rotation as numpy arrays
            Eigen::Vector3d translation = increment.translation();
            Eigen::Matrix3d rotation = increment.rotation();

            // Return as a dictionary or tuple
            return py::make_tuple(translation, rotation);
        })
        .def("getIncrementCovariance", &CLaserOdometry2D::getIncrementCovariance)
        .def("getPose", py::overload_cast<>(&CLaserOdometry2D::getPose, py::const_))
        .def("getPoseMutable", py::overload_cast<>(&CLaserOdometry2D::getPose));
}
