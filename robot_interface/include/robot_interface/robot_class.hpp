/**
 * @file robot_class.hpp
 *
 * @brief This message displayed in Doxygen Files index
 *
 * @ingroup PackageName
 * (Note: this needs exactly one @defgroup somewhere)
 *
 * @author Lasitha Wijayarathne
 * Contact: lasitha@gatech.edu
 *
 */

#pragma once

#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <memory>
#include <string.h> 
// #include "kuka_lib.h"

#include <iostream>
#include <fstream>

#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointTorque.h>
#include <iiwa_msgs/Wrench.h>
#include <iiwa_msgs/JointVelocity.h>
#include <iiwa_msgs/JointPositionVelocity.h>
#include <std_msgs/Time.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <rosgraph_msgs/Clock.h>

#include <geometry_msgs/WrenchStamped.h>
#include <memory>
#include <boost/program_options.hpp>

#include <algorithm>
#include "robot_interface.hpp"



// Abstract class for robot KUKA arm
namespace robot_interface {
 
  class robotABSTRACT {
  public:
    robotABSTRACT(){};
    virtual ~robotABSTRACT(){};
    virtual void init(ros::NodeHandle& nh_states, ros::NodeHandle& nh_command){};
    virtual void setJointPosition(const iiwa_msgs::JointPosition& position) {};
    virtual void setJointVelocity(const iiwa_msgs::JointVelocity& velocity) {};
    virtual void setJointTorque(const iiwa_msgs::JointTorque& torque) {};
    virtual void setWrench(const iiwa_msgs::Wrench& wrench) {};

    virtual bool getCartesianPose(geometry_msgs::PoseStamped& value) {return 0;}
    virtual bool getJointPosition(iiwa_msgs::JointPosition& value) {return 0;}
    virtual bool getJointTorque(iiwa_msgs::JointTorque& value) {return 0;}
    virtual bool getCartesianWrench(geometry_msgs::WrenchStamped& value) {return 0;}
    virtual bool getJointVelocity(iiwa_msgs::JointVelocity& value) {return 0;}
    virtual bool getJointCommVelocity(iiwa_msgs::JointVelocity& value) {return 0;}
    virtual bool getJointPositionCommand(iiwa_msgs::JointPosition& value){return 0;}
    virtual bool getRobotTime(double& value) {return 0;};

    bool getRobotIsConnected() {
      ros::Duration diff = (ros::Time::now() - last_update_time);
      return (diff < ros::Duration(0.25));
    }
    
    
  };


  class robotKUKA : public robotABSTRACT {
  public:
    
    robotKUKA();
 
    void init(ros::NodeHandle& nh_states, ros::NodeHandle& nh_command);

    void initPy() {
        // int argc = 1;
        // char** argv = new char *[1];
        // ros::init(argc, argv, "CommandRobot");
        ros::NodeHandle nh_states("~");
        ros::NodeHandle nh_command("~");

        // bool isGazebo = true;
        // double ros_rate;

        init(nh_states, nh_command);
        // nh.getParam("isGazebo", isGazebo);
        // nh.param("ros_rate", ros_rate, rate);
        // return nh;
    }
    bool getCartesianPose(geometry_msgs::PoseStamped& value);
    bool getJointPosition(iiwa_msgs::JointPosition& value);
    bool getJointTorque(iiwa_msgs::JointTorque& value);
    // bool getCartesianWrench(geometry_msgs::WrenchStamped& value);
    bool getJointVelocity(iiwa_msgs::JointVelocity& value);
    // bool getJointCommVelocity(iiwa_msgs::JointVelocity& value);
    bool getJointPositionCommand(iiwa_msgs::JointPosition& value);
    void setJointPosition(const iiwa_msgs::JointPosition& position);
    // void setJointVelocity(const iiwa_msgs::JointVelocity& velocity);
 	  void setJointTorque(const iiwa_msgs::JointTorque& torque);
    void setWrench(const iiwa_msgs::Wrench& wrench);
	  bool getRobotTime(double& time);

    bool getRobotIsConnected() 
    {
      ros::Duration diff = (ros::Time::now() - last_update_time);
      ROS_INFO_STREAM("exiting...");

      return (diff < ros::Duration(0.25));
    }


  protected:
    // Subscribed topics / states
    std_msgs::Time time_;
    robotStateHandler<geometry_msgs::PoseStamped> handler_state_pose_;
    robotStateHandler<std_msgs::Time> handler_state_time_;
    robotStateHandler<iiwa_msgs::JointPosition> handler_state_joint_position_;
    robotStateHandler<iiwa_msgs::JointPosition> handler_state_joint_position_command_;
    robotStateHandler<iiwa_msgs::JointTorque> handler_state_joint_torque_;
    robotStateHandler<geometry_msgs::WrenchStamped> handler_state_wrench_;
    robotStateHandler<iiwa_msgs::JointVelocity> handler_state_joint_velocity_;
    robotStateHandler<iiwa_msgs::JointVelocity> handler_state_comm_joint_velocity_;
    
    // Commands
    robotCommandHandler<iiwa_msgs::JointPosition> handler_command_torque_;
    robotCommandHandler<iiwa_msgs::Wrench> handler_command_wrench_;
    robotCommandHandler<iiwa_msgs::JointPosition> handler_command_joint_position_;
    robotCommandHandler<iiwa_msgs::JointVelocity> handler_command_joint_velocity_;

  };



// // Abstract class for KUKA gazebo simulation
// class robotKUKA_SIM : public robotABSTRACT {
// public:
  

//   robotKUKA_SIM();

//   void init(ros::NodeHandle& nh_states, ros::NodeHandle& nh_command);

//   void initPy() 
//   {
//     int argc = 1;
//     char** argv = new char *[1];
//     ros::init(argc, argv, "CommandRobot");
//     ros::NodeHandle nh_states("~");
//     ros::NodeHandle nh_command("~");

//     init(nh_states, nh_command);
//     // ros::AsyncSpinner spinner(6); // Use 4 threads
//     // spinner.start();
//   }
  
//   double getRobotTimePy(); 

//   bool getCartesianPose(geometry_msgs::PoseStamped& value);
//   bool getJointPosition(iiwa_msgs::JointPosition& value);
//   bool getJointTorque(iiwa_msgs::JointTorque& value);
//   // bool getCartesianWrench(geometry_msgs::WrenchStamped& value);
//   bool getJointVelocity(iiwa_msgs::JointVelocity& value);
//   // bool getJointCommVelocity(iiwa_msgs::JointVelocity& value);
//   bool getJointPositionCommand(iiwa_msgs::JointPosition& value);
//   void setJointPosition(const iiwa_msgs::JointPosition& position);
//   // void setJointVelocity(const iiwa_msgs::JointVelocity& velocity);
//   void setJointTorque(const iiwa_msgs::JointTorque& torque);
//   bool getRobotTime(double& value);

//   bool getRobotIsConnected() {
//     ros::Duration diff = (ros::Time::now() - last_update_time);
//     return (diff < ros::Duration(0.25));
//   }


//   protected:
//     sensor_msgs::JointState position_;
//     rosgraph_msgs::Clock time_;
//     robotStateHandler<sensor_msgs::JointState> handler_state_joint_;
//     robotStateHandler<rosgraph_msgs::Clock> handler_state_time_;

//     robotCommandHandler<std_msgs::Float64> handler_command_joint_p1_;
//     robotCommandHandler<std_msgs::Float64> handler_command_joint_p2_;
//     robotCommandHandler<std_msgs::Float64> handler_command_joint_p3_;
//     robotCommandHandler<std_msgs::Float64> handler_command_joint_p4_;
//     robotCommandHandler<std_msgs::Float64> handler_command_joint_p5_;
//     robotCommandHandler<std_msgs::Float64> handler_command_joint_p6_;
//     robotCommandHandler<std_msgs::Float64> handler_command_joint_p7_;
// };
 
// abstract class for dual manipulator system
class robotDUALARM : public robotABSTRACT {
public:
  

  robotDUALARM ();

  void init(ros::NodeHandle& nh_states, ros::NodeHandle& nh_command);
  bool getCartesianPose(geometry_msgs::PoseStamped& value);
  bool getJointPosition(iiwa_msgs::JointPosition& value);
  bool getJointTorque(iiwa_msgs::JointTorque& value);
  bool getCartesianWrench(geometry_msgs::WrenchStamped& value);
  bool getJointVelocity(iiwa_msgs::JointVelocity& value);
  bool getJointCommVelocity(iiwa_msgs::JointVelocity& value);
  bool getJointPositionCommand(iiwa_msgs::JointPosition& value);
  void setJointPosition(const iiwa_msgs::JointPosition& position);
  void setJointVelocity(const iiwa_msgs::JointVelocity& velocity);
  void setJointTorque(const iiwa_msgs::JointTorque& torque);
  bool getRobotTime(double& value);
  
  bool getRobotIsConnected() {
    ros::Duration diff = (ros::Time::now() - last_update_time);
    return (diff < ros::Duration(0.25));
  }

protected:
    sensor_msgs::JointState position_;
    rosgraph_msgs::Clock time_;
    robotStateHandler<sensor_msgs::JointState> handler_state_joint_;
    robotStateHandler<rosgraph_msgs::Clock> handler_state_time_;

    robotCommandHandler<std_msgs::Float64> handler_command_joint_p1_K;
    robotCommandHandler<std_msgs::Float64> handler_command_joint_p2_K;
    robotCommandHandler<std_msgs::Float64> handler_command_joint_p3_K;
    robotCommandHandler<std_msgs::Float64> handler_command_joint_p4_K;
    robotCommandHandler<std_msgs::Float64> handler_command_joint_p5_K;
    robotCommandHandler<std_msgs::Float64> handler_command_joint_p6_K;
    robotCommandHandler<std_msgs::Float64> handler_command_joint_p7_K;

    robotCommandHandler<std_msgs::Float64> handler_command_joint_p1_L;
    robotCommandHandler<std_msgs::Float64> handler_command_joint_p2_L;
    robotCommandHandler<std_msgs::Float64> handler_command_joint_p3_L;
    robotCommandHandler<std_msgs::Float64> handler_command_joint_p4_L;
    robotCommandHandler<std_msgs::Float64> handler_command_joint_p5_L;
    robotCommandHandler<std_msgs::Float64> handler_command_joint_p6_L;
    robotCommandHandler<std_msgs::Float64> handler_command_joint_p7_L;

    robotCommandHandler<std_msgs::Float64> handler_command_joint_p1_R;
    robotCommandHandler<std_msgs::Float64> handler_command_joint_p2_R;
    robotCommandHandler<std_msgs::Float64> handler_command_joint_p3_R;
    robotCommandHandler<std_msgs::Float64> handler_command_joint_p4_R;
    robotCommandHandler<std_msgs::Float64> handler_command_joint_p5_R;
    robotCommandHandler<std_msgs::Float64> handler_command_joint_p6_R;
    robotCommandHandler<std_msgs::Float64> handler_command_joint_p7_R;
  
};



}








