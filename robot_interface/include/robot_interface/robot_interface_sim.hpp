#ifndef ROBOT_INTERFACE_SIM_HPP
#define ROBOT_INTERFACE_SIM_HPP

#include "robot_class.hpp"
#include <std_msgs/Time.h>

namespace robot_interface {

  // Abstract class for KUKA gazebo simulation
  class robotKUKA_SIM : virtual public robotABSTRACT {
  public:
    robotKUKA_SIM(bool enable_ft);

    void init(ros::NodeHandle& nh_states, ros::NodeHandle& nh_command) override;

    void initPy() 
    {
      int argc = 1;
      char** argv = new char *[1];
      ros::init(argc, argv, "CommandRobot");
      ros::NodeHandle nh_states("~");
      ros::NodeHandle nh_command("~");

      init(nh_states, nh_command);

    }
    
    double getRobotTimePy(); 

    bool getCartesianPose(geometry_msgs::PoseStamped& value);
    bool getJointPosition(iiwa_msgs::JointPosition& value);
    bool getJointTorque(iiwa_msgs::JointTorque& value);
    bool getCartesianWrench(geometry_msgs::WrenchStamped& value);
    bool getJointVelocity(iiwa_msgs::JointVelocity& value);
    bool getJointPositionCommand(iiwa_msgs::JointPosition& value);
    void setJointPosition(const iiwa_msgs::JointPosition& position);
    void setJointTorque(const iiwa_msgs::JointTorque& torque);
    bool getRobotTime(double& value);



    bool getRobotIsConnected() {
      ros::Duration diff = (ros::Time::now() - last_update_time);
      return (diff < ros::Duration(0.25));
    }


    protected:
      bool end_effector_ft_;
      sensor_msgs::JointState joint_state_;
      rosgraph_msgs::Clock time_;

      robotStateHandler<sensor_msgs::JointState> handler_state_joint_;
      robotStateHandler<geometry_msgs::WrenchStamped> handler_state_wrench_;
      robotStateHandler<rosgraph_msgs::Clock> handler_state_time_;

      robotCommandHandler<std_msgs::Float64> handler_command_joint_p1_;
      robotCommandHandler<std_msgs::Float64> handler_command_joint_p2_;
      robotCommandHandler<std_msgs::Float64> handler_command_joint_p3_;
      robotCommandHandler<std_msgs::Float64> handler_command_joint_p4_;
      robotCommandHandler<std_msgs::Float64> handler_command_joint_p5_;
      robotCommandHandler<std_msgs::Float64> handler_command_joint_p6_;
      robotCommandHandler<std_msgs::Float64> handler_command_joint_p7_;


      robotCommandHandler<std_msgs::Float64> handler_command_torque_p1_;
      robotCommandHandler<std_msgs::Float64> handler_command_torque_p2_;
      robotCommandHandler<std_msgs::Float64> handler_command_torque_p3_;
      robotCommandHandler<std_msgs::Float64> handler_command_torque_p4_;
      robotCommandHandler<std_msgs::Float64> handler_command_torque_p5_;
      robotCommandHandler<std_msgs::Float64> handler_command_torque_p6_;
      robotCommandHandler<std_msgs::Float64> handler_command_torque_p7_;

      robotCommandHandler<std_msgs::Time> handler_command_time_;

  };

}

#endif