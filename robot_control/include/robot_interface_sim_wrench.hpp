#ifndef ROBOT_INTERFACE_SIM_WRENCH_HPP
#define ROBOT_INTERFACE_SIM_WRENCH_HPP

#include <ros/ros.h>
#include "robot_interface/robot_class.hpp"
#include "robot_abstract.h"

namespace robot_interface {

  // This emulates the wrench control in the real KUKA robot.
  class robotKUKA_SIM_WRENCH : public robotKUKA_SIM {
  public:

    std::shared_ptr<RobotAbstract> m_robot;
    Eigen::VectorXd m_gravityTorque;
    Eigen::MatrixXd m_jacobian;


    robotKUKA_SIM_WRENCH(const std::shared_ptr<RobotAbstract>& robot, bool enable_ft) : robotKUKA_SIM(enable_ft), m_robot(robot) {
      m_gravityTorque.resize(7);
      m_jacobian.resize(6, 7);
    }

    void setJointTorque(const iiwa_msgs::JointPosition& position, const iiwa_msgs::JointTorque& torque, const geometry_msgs::WrenchStamped& wrench)
    {
      // calculate the gravity vector for gravty compensation
      // m_robot.getGravityVector(double* q, gravi);

      // caluculate the joint torque corresponding to the wrench
      // m_robot.getSpatialJacobian(double* q, jacobian)
      ROS_INFO_STREAM("Commanding in WRENCH MODE...");


      robotKUKA_SIM::setJointTorque(torque);
    }


  };

}

#endif