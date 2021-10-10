# Robot Motion Stack
This repository is consists of most of the motion primitives needed for motion generation in real-time. Main components are:
- **Robot Models** - Models of different robots expressed with [orocos-kdl](https://github.com/orocos/orocos_kinematics_dynamics.git)
- **Robot Interface** - Robot interface for gazebo and real. Abstract class and it's derivatives
- **Robot Control** - P2P motion generation, Spline generation
- **Modern Robotics** - ROS wrapper for [ModernRoboticsCpp](https://github.com/Le0nX/ModernRoboticsCpp.git)
- **IK solvers** - Differential Kinematic Solvers for generation of inverse kinematic solutions using Exponential Product formulation.

## How to use
- Robot interface is defined in ```robot_interface/include/robot_interface.hpp``` with following attributes. ```robot_interface/src/robot_class.cpp``` and ```robot_interface/src/robot_interface_sim.cpp``` are the implementations of the interface for real and simulation. Currently, [kuka_msgs]() are used specific to serial robots.

- For motion generation, ```robot_control/scripts/KUKA_TESTS.cpp``` is an example for the sequential implementation and is commented. The motion sequence is as follows:
  - Given the current position of the current configuration, move the robot to the desired starting configuration.
  - Given the desired way-points, spline to generate continuous motion and command around ~200Hz.

- Force control with admittance control - ```robot_interface/include/robot_interface_force_control.hpp``` is the interface for force control equipped with a ATI F/T sensor. The [netft_utils](https://github.com/UTNuclearRoboticsPublic/netft_utils.git) is used to read force/torque data. TODO: An example implementation  

<!-- ## Acknowledgement -->
