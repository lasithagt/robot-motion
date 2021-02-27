#ifndef ROBOT_INTERFACE_WRENCH_HPP
#define ROBOT_INTERFACE_WRENCH_HPP

#include <ros/ros.h>
#include "robot_interface/robot_class.hpp"
#include "robot_interface/robot_interface_force_control.hpp"
#include "robot_abstract.h"

namespace robot_interface {

  // This emulates the wrench control in the real KUKA robot.
  class robotKUKA_WRENCH : public robotKUKA, public robotKUKAForceControl 
  {

  public:
    robotKUKA_WRENCH(const std::shared_ptr<RobotAbstract>& robot, const FT_Transform& T, bool enable_ft) : robotKUKAForceControl::robotKUKAForceControl(robot, T, enable_ft) {}

    void init(ros::NodeHandle& nh_states, ros::NodeHandle& nh_command) override
    {
      robotKUKA::init(nh_states, nh_command);
      robotKUKAForceControl::init(nh_states, nh_command);
    }


    void setJointPositionWrench(const iiwa_msgs::JointPosition& position_desired, const iiwa_msgs::Wrench& wrench) override
    {
      // get current joint position
      // this->getJointPosition(position_current);

      // caluculate the joint torque corresponding to the wrench
      ROS_INFO_STREAM("Commanding in WRENCH MODE...");
      robotKUKA::setJointPosition(position_desired);
    }



    void setJointPositionTorque(const iiwa_msgs::JointPosition& position_desired, const iiwa_msgs::JointTorque& torque_ff) override
    {
      // get current joint position
      // this->getJointPosition(position_current);

      // caluculate the joint torque corresponding to the wrench
      ROS_INFO_STREAM("Commanding in POSITION + TORQUE MODE...");

      robotKUKA::setJointTorque(torque_ff);
      // for (auto &it : torque_ff.torque.quantity) {std::cout << it << "\n" << std::endl;}
      robotKUKA::setJointPosition(position_desired);
    }


  };

}

#endif