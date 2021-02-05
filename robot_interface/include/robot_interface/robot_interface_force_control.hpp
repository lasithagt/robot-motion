#ifndef ROBOT_INTERFACE_FORCE_CONTROL_HPP
#define ROBOT_INTERFACE_FORCE_CONTROL_HPP

#include <ros/ros.h>
#include "robot_interface/robot_class.hpp"
#include "robot_abstract.h"

namespace robot_interface {

  struct FT_Transform
  {
    Eigen::Matrix<double, 3, 3> R;
    Eigen::Matrix<double, 3,  1> p;
    FT_Transform() = default;

  };

    // This emulates the wrench control in the real KUKA robot.
  class robotKUKAForceControl : virtual public robotABSTRACT  
  {
  
  public:

    std::shared_ptr<RobotAbstract> m_robot;
    FT_Transform forceTranform;
    Eigen::MatrixXd m_jacobian;
    iiwa_msgs::JointTorque torque_cmd;
    iiwa_msgs::JointPosition position_current;
    geometry_msgs::WrenchStamped wrench_raw;

    // force torque sensor
    robotStateHandler<geometry_msgs::WrenchStamped> handler_state_wrench_;


    robotKUKAForceControl(const std::shared_ptr<RobotAbstract>& robot, const FT_Transform& T, bool enable_ft) : m_robot(robot), forceTranform(T)
    {
      ROS_INFO_STREAM("Initializing Robot in Force Control mode...");
      m_jacobian.resize(6, 7);
      torque_cmd.torque.quantity.resize(7);
      position_current.position.quantity.resize(7);

    }

    void init(ros::NodeHandle& nh_states, ros::NodeHandle& nh_command) override
    {

      handler_state_wrench_.init("/netft_data", nh_states);

      // if (handler_state_wrench_.getNumPublishers() < 1)
      // {
      //   ROS_WARN_STREAM("Could not find the FT sensor topic...");
      // }
      
    } 

    bool getStateWrench(iiwa_msgs::Wrench& value) 
    {
      // convert the ATI FT to KUKA frame.  
      auto valid = handler_state_wrench_.get(wrench_raw);

      value.header.stamp = ros::Time::now();
      value.x = wrench_raw.wrench.force.x;
      value.y = wrench_raw.wrench.force.y;
      value.z = wrench_raw.wrench.force.z;
      
      return valid;
    }

  };

}

#endif