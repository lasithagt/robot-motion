
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>
#include <vector>

#include <ros/ros.h>
#include <robot_class.hpp>
#include <robot_interface.hpp>
#include <robot_kinematics.hpp>
#include <robot_motion_library.hpp>

#include <iiwa_msgs/JointVelocity.h>
#include <iiwa_msgs/JointPosition.h>

namespace py = pybind11;

void start_spinners() {
  ros::AsyncSpinner spinner(6); // Use 4 threads
  spinner.start();
}

PYBIND11_MODULE(pybindings, m) {
    m.doc() = "pybind11 example plugin";

    using namespace pybind11::literals;

    py::class_<robot_interface::robotABSTRACT> robot(m, "robotABSTRACT");
    robot
        .def(py::init<>());

    py::class_<robot_interface::robotKUKA_SIM>(m, "robotKUKA_SIM", robot)
        .def(py::init<>())
        .def("initPy", &robot_interface::robotKUKA_SIM::initPy)
        .def("getPosition", &robot_interface::robotKUKA_SIM::getJointPosition)
        .def("getTime", &robot_interface::robotKUKA_SIM::getRobotTimePy)
        .def("setPosition", &robot_interface::robotKUKA_SIM::setJointPosition);

    m.def("startSpin", &start_spinners, "A function which adds two numbers");

    // py::class_<robot_interface::robotKUKA>(m, "robotKUKA", robot)
    //     .def(py::init<>())
    //     .def("init"<ros::NodeHandle &, ros::NodeHandle &>())
    //     .def("getPosition", &robot_interface::robotKUKA::getJointPosition)
    //     .def("getTime", &robot_interface::robotKUKA::getRobotTime)
    //     .def("setPosition", &robot_interface::robotKUKA::setJointPosition);

    // py::class_<ros::NodeHandle>(m, "NodeHandle")
    //     .def(py::init());



        // .def("init", &robot_interface::robotKUKA_SIM::init, "nh"_a)
        //.def("initPy", &iiwa_ros::iiwaRosGazebo::initPy, "rate"_a=60.0)
        //.def("setJointPosition", &robot_interface::robotKUKA_SIM::setJointPosition, "position"_a);
        //.def("setJointVelocityPy", &iiwa_ros::iiwaRosGazebo::setJointVelocityPy, "velocity"_a=(std::vector<double>){0, 0, 0, 0, 0, 0, 0});

    // return m.ptr();
}



