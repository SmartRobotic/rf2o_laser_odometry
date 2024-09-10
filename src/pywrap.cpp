// bindings.cpp

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>  // For Eigen support
#include "../include/rf2o_laser_odometry/CLaserOdometry2D.h" // Include your header file

namespace py = pybind11;
using namespace rf2o;

PYBIND11_MODULE(CLaserOdometry2D, m) {
    m.doc() = "Python bindings for the CLaserOdometry2D class and related structures";

    // Expose the LaserScan struct
    py::class_<LaserScan>(m, "LaserScan", "Structure representing a laser scan")
        .def(py::init<>(), "Default constructor")
        .def_readwrite("ranges", &LaserScan::ranges, "Range measurements")
        .def_readwrite("angle_min", &LaserScan::angle_min, "Minimum angle of the scan")
        .def_readwrite("angle_max", &LaserScan::angle_max, "Maximum angle of the scan")
        .def_readwrite("angle_increment", &LaserScan::angle_increment, "Angle increment between measurements");

    // Expose the Pose struct and its nested Orientation struct
    py::class_<Pose::Orientation>(m, "Orientation", "Structure representing orientation in 3D space")
        .def(py::init<>(), "Default constructor")
        .def_readwrite("w", &Pose::Orientation::w, "W component of the quaternion")
        .def_readwrite("x", &Pose::Orientation::x, "X component of the quaternion")
        .def_readwrite("y", &Pose::Orientation::y, "Y component of the quaternion")
        .def_readwrite("z", &Pose::Orientation::z, "Z component of the quaternion");

    py::class_<Pose>(m, "Pose", "Structure representing a 2D pose with orientation")
        .def(py::init<>(), "Default constructor")
        .def_readwrite("x", &Pose::x, "X coordinate")
        .def_readwrite("y", &Pose::y, "Y coordinate")
        .def_readwrite("theta", &Pose::theta, "Orientation angle in radians")
        .def_readwrite("orientation", &Pose::orientation, "Orientation as a quaternion");

    // Expose the CLaserOdometry2D class
    py::class_<CLaserOdometry2D>(m, "CLaserOdometry2D", "Class for 2D laser odometry")
        .def(py::init<>(), "Default constructor")
        .def("testab", &CLaserOdometry2D::testab, "Test function")
        .def("init", &CLaserOdometry2D::init, "Initialize the odometry system")
        .def("is_initialized", &CLaserOdometry2D::is_initialized, "Check if the odometry system is initialized")
        .def("odometryCalculation", &CLaserOdometry2D::odometryCalculation, "Perform odometry calculation")
        .def("setLaserPose", &CLaserOdometry2D::setLaserPose, "Set the pose of the laser")
        .def("getIncrement", [](const CLaserOdometry2D& self) {
            // Convert Eigen::Isometry3d to Python-friendly format
            Eigen::Isometry3d increment = self.getIncrement();
            
            // Extract translation and rotation as numpy arrays
            Eigen::Vector3d translation = increment.translation();
            Eigen::Matrix3d rotation = increment.rotation();

            // Return as a dictionary or tuple
            return py::make_tuple(translation, rotation);
        }, "Get the increment as a tuple of translation and rotation")
        .def("getIncrementCovariance", &CLaserOdometry2D::getIncrementCovariance, "Get the covariance of the increment")
        .def("getPose", py::overload_cast<>(&CLaserOdometry2D::getPose, py::const_), "Get the current pose (const)")
        .def("getPoseMutable", py::overload_cast<>(&CLaserOdometry2D::getPose), "Get the current pose (mutable)");
}
