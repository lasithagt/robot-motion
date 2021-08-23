#ifndef ROBOT_INTERFACE_SIM_WRENCH_HPP
#define ROBOT_INTERFACE_SIM_WRENCH_HPP

#include <ros/ros.h>
#include "robot_interface/robot_interface_force_control.hpp"
#include "robot_abstract.h"
#include "robot_interface/robot_interface_sim.hpp"

namespace robot_interface {

  // This emulates the wrench control in the real KUKA robot.
  class robotKUKA_SIM_WRENCH : public robotKUKAForceControl, public robotKUKA_SIM
  {

  public:
    robotKUKA_SIM_WRENCH(const std::shared_ptr<RobotAbstract>& robot, const FT_Transform& T, bool enable_ft)
     : robotKUKA_SIM::robotKUKA_SIM(enable_ft), robotKUKAForceControl::robotKUKAForceControl(robot, T, enable_ft) {}

    void init(ros::NodeHandle& nh_states, ros::NodeHandle& nh_command) override
    {
      robotKUKA_SIM::init(nh_states, nh_command);
      robotKUKAForceControl::init(nh_states, nh_command);
    }



    void setJointPositionWrench(const iiwa_msgs::JointPosition& position_desired, const iiwa_msgs::Wrench& wrench) override
    {
      // get current joint position
      this->getJointPosition(position_current);

      // calculate the gravity vector for gravty compensation
      // m_robot->getGravityVector(position_current.position.quantity.data(), m_gravityTorque);

      // caluculate the joint torque corresponding to the wrench
      ROS_INFO_STREAM("Commanding in WRENCH MODE...");

      // command with gravity compensation
      for (int i = 0;i < 7;i++)
      {
        torque_cmd.torque.quantity.at(i) =  (position_desired.position.quantity.at(i) - position_current.position.quantity.at(i));
      }
      robotKUKA_SIM::setJointPosition(position_desired);
    }



    void setJointPositionTorque(const iiwa_msgs::JointPosition& position_desired, const iiwa_msgs::JointTorque& torque_ff) override
    {

      // caluculate the joint torque corresponding to the wrench
      // ROS_INFO_STREAM("Commanding in POSITION + TORQUE MODE...");

      // robotKUKA::setJointTorque(torque_ff);
      robotKUKA_SIM::setJointPosition(position_desired);
    }

  };

}

#endif