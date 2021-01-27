/**
 * @file robot_class.cpp
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

#include "robot_class.hpp"


using namespace std;                        

namespace robot_interface {
  
  ros::Time last_update_time;
  
  robotKUKA::robotKUKA() {

  }

  void robotKUKA::init(ros::NodeHandle& nh_states, ros::NodeHandle& nh_command) {	

      // handler_state_pose_.init("/kuka/state/KUKAActualTorque");
      handler_state_time_.init("/kuka/state/KUKATime", nh_states);
      handler_state_joint_torque_.init("/kuka/state/KUKAActualTorque", nh_states);
      handler_state_joint_position_.init("/kuka/state/KUKAJointPosition", nh_states);
      handler_state_joint_position_command_.init("/kuka/state/KUKAJointPositionCommand", nh_states);

      // handler_state_wrench_.init("iiwa/state/CartesianWrench");
      handler_state_joint_velocity_.init("/kuka/state/KUKAJointVelocity", nh_states);
      // handler_state_comm_joint_velocity_.init("iiwa/command/JointVelocity", nh_command);
      
      // handler_command_pose_.init("iiwa/command/CartesianPose", nh);
      handler_command_joint_position_.init("/kuka/command/command_position", nh_command);
      handler_command_wrench_.init("/kuka/command/command_wrench", nh_command);
      // handler_command_joint_position_velocity_.init("iiwa/command/JointPositionVelocity", nh);
      // handler_command_joint_velocity_.init("iiwa/command/JointVelocity", nh);

      last_update_time = ros::Time::now();
      
    }
  

  bool robotKUKA::getCartesianPose(geometry_msgs::PoseStamped& value) {
    // return handler_state_pose_.get(value);
  }

  bool robotKUKA::getJointPosition(iiwa_msgs::JointPosition& value) {
    return handler_state_joint_position_.get(value);
  }

  bool robotKUKA::getJointPositionCommand(iiwa_msgs::JointPosition& value) {
    return handler_state_joint_position_command_.get(value);
  }

  bool robotKUKA::getJointTorque(iiwa_msgs::JointTorque& value) {
    return handler_state_joint_torque_.get(value);
  }

  bool robotKUKA::getJointVelocity(iiwa_msgs::JointVelocity& value) {
    return handler_state_joint_velocity_.get(value);
  }

  void robotKUKA::setJointPosition(const iiwa_msgs::JointPosition& position)  {
    handler_command_joint_position_.set(position);
    handler_command_joint_position_.publishIfNew();
  }

  void robotKUKA::setJointTorque(const iiwa_msgs::JointTorque& torque)  {
    // TODO
  }

  void robotKUKA::setWrench(const iiwa_msgs::Wrench& wrench)  {
    
    handler_command_wrench_.set(wrench);
    handler_command_wrench_.publishIfNew();
  }

  bool robotKUKA::getRobotTime(double& value) {

    bool temp = handler_state_time_.get(time_);
    value = static_cast<double>(time_.data.sec) + static_cast<double>(time_.data.nsec)*static_cast<double>(pow(10, -9));
    
    return temp;  
  }


  // robotKUKA_SIM::robotKUKA_SIM() {

  // }

  // void robotKUKA_SIM::init(ros::NodeHandle& nh_states, ros::NodeHandle& nh_command) { 

  //     // handler_state_pose_.init("/kuka/state/KUKAActualTorque");

  //     handler_state_joint_.init("/iiwa/joint_states", nh_states);
  //     handler_state_time_.init("/clock", nh_states);

  //     handler_command_joint_p1_.init("/iiwa/EffortJointInterface_J1_controller/command", nh_command);
  //     handler_command_joint_p2_.init("/iiwa/EffortJointInterface_J2_controller/command", nh_command);
  //     handler_command_joint_p3_.init("/iiwa/EffortJointInterface_J3_controller/command", nh_command);
  //     handler_command_joint_p4_.init("/iiwa/EffortJointInterface_J4_controller/command", nh_command);
  //     handler_command_joint_p5_.init("/iiwa/EffortJointInterface_J5_controller/command", nh_command);
  //     handler_command_joint_p6_.init("/iiwa/EffortJointInterface_J6_controller/command", nh_command);
  //     handler_command_joint_p7_.init("/iiwa/EffortJointInterface_J7_controller/command", nh_command);


  //     last_update_time = ros::Time::now();
  //     ROS_INFO_STREAM("Initialized Simulation Subscribers... ");

      
  //   }


  // bool robotKUKA_SIM::getCartesianPose(geometry_msgs::PoseStamped& value) {
  //   // return handler_state_pose_.get(value);
  // }

  // bool robotKUKA_SIM::getJointPosition(iiwa_msgs::JointPosition& value) {

  //   bool temp = handler_state_joint_.get(position_);
  //   if (temp) {
  //     memcpy(value.position.quantity.data(), position_.position.data(), 7*sizeof(double));
  //   } 
  //   return temp;

  // }

  // bool robotKUKA_SIM::getJointPositionCommand(iiwa_msgs::JointPosition& value) {
  //   // return handler_state_joint_position_command_.get(value);
  // }

  // bool robotKUKA_SIM::getJointTorque(iiwa_msgs::JointTorque& value) {
  //   // return handler_state_joint_torque_.get(value);
  // }

  // bool robotKUKA_SIM::getJointVelocity(iiwa_msgs::JointVelocity& value) {
  //   // return handler_state_joint_velocity_.get(value); TODO
  // }

  // void robotKUKA_SIM::setJointPosition(const iiwa_msgs::JointPosition& position)  {

  //   std_msgs::Float64 temp;
  //   temp.data = position.position.quantity.at(0);
  //   handler_command_joint_p1_.set(temp);
  //   temp.data = position.position.quantity.at(1);
  //   handler_command_joint_p2_.set(temp);
  //   temp.data = position.position.quantity.at(2);
  //   handler_command_joint_p3_.set(temp);
  //   temp.data = position.position.quantity.at(3);
  //   handler_command_joint_p4_.set(temp);
  //   temp.data = position.position.quantity.at(4);
  //   handler_command_joint_p5_.set(temp);
  //   temp.data = position.position.quantity.at(5);
  //   handler_command_joint_p6_.set(temp);
  //   temp.data = position.position.quantity.at(6);
  //   handler_command_joint_p7_.set(temp);

  //   handler_command_joint_p1_.publishIfNew();
  //   handler_command_joint_p2_.publishIfNew();
  //   handler_command_joint_p3_.publishIfNew();
  //   handler_command_joint_p4_.publishIfNew();
  //   handler_command_joint_p5_.publishIfNew();
  //   handler_command_joint_p6_.publishIfNew();
  //   handler_command_joint_p7_.publishIfNew();
  // }

  // void robotKUKA_SIM::setJointTorque(const iiwa_msgs::JointTorque& torque)  {
  //   // TODO
  // }

  // bool robotKUKA_SIM::getRobotTime(double& value) {

  //   bool temp = handler_state_time_.get(time_);
  //   value = static_cast<double>(time_.clock.sec) + static_cast<double>(time_.clock.nsec)*static_cast<double>(pow(10, -9));
  //   return temp;  
  // }
  // double robotKUKA_SIM::getRobotTimePy() {
  //   double value;
  //   getRobotTime(value);
  //   return value;
  // }

  /*

  robotDUALARM::robotDUALARM() {

  }

  void robotDUALARM::init(ros::NodeHandle& nh_states, ros::NodeHandle& nh_command) { 

      // handler_state_pose_.init("/kuka/state/KUKAActualTorque");

      handler_state_joint_.init("/iiwa/joint_states", nh_states);
      handler_state_time_.init("/clock", nh_states);

      handler_command_joint_p1_.init("/iiwa/EffortJointInterface_J1_controller/command", nh_command);
      handler_command_joint_p2_.init("/iiwa/EffortJointInterface_J2_controller/command", nh_command);
      handler_command_joint_p3_.init("/iiwa/EffortJointInterface_J3_controller/command", nh_command);
      handler_command_joint_p4_.init("/iiwa/EffortJointInterface_J4_controller/command", nh_command);
      handler_command_joint_p5_.init("/iiwa/EffortJointInterface_J5_controller/command", nh_command);
      handler_command_joint_p6_.init("/iiwa/EffortJointInterface_J6_controller/command", nh_command);
      handler_command_joint_p7_.init("/iiwa/EffortJointInterface_J7_controller/command", nh_command);

      last_update_time = ros::Time::now();

      
    }


  bool robotDUALARM::getCartesianPose(geometry_msgs::PoseStamped& value) {
    // return handler_state_pose_.get(value);
  }

  bool robotDUALARM::getJointPosition(iiwa_msgs::JointPosition& value) {

    bool temp = handler_state_joint_.get(position_);
    if (temp) {
      memcpy(value.position.quantity.data(), position_.position.data(), 7*sizeof(double));
    } 
    return temp;

  }

  bool robotDUALARM::getJointPositionCommand(iiwa_msgs::JointPosition& value) {
    // return handler_state_joint_position_command_.get(value);
  }

  bool robotDUALARM::getJointTorque(iiwa_msgs::JointTorque& value) {
    // return handler_state_joint_torque_.get(value);
  }

  bool robotDUALARM::getJointVelocity(iiwa_msgs::JointVelocity& value) {
    // return handler_state_joint_velocity_.get(value);
  }

  void robotDUALARM::setJointPosition(const iiwa_msgs::JointPosition& position)  {

    std_msgs::Float64 temp;
    temp.data = position.position.quantity.at(0);
    handler_command_joint_p1_.set(temp);
    temp.data = position.position.quantity.at(1);
    handler_command_joint_p2_.set(temp);
    temp.data = position.position.quantity.at(2);
    handler_command_joint_p3_.set(temp);
    temp.data = position.position.quantity.at(3);
    handler_command_joint_p4_.set(temp);
    temp.data = position.position.quantity.at(4);
    handler_command_joint_p5_.set(temp);
    temp.data = position.position.quantity.at(5);
    handler_command_joint_p6_.set(temp);
    temp.data = position.position.quantity.at(6);
    handler_command_joint_p7_.set(temp);

    handler_command_joint_p1_.publishIfNew();
    handler_command_joint_p2_.publishIfNew();
    handler_command_joint_p3_.publishIfNew();
    handler_command_joint_p4_.publishIfNew();
    handler_command_joint_p5_.publishIfNew();
    handler_command_joint_p6_.publishIfNew();
    handler_command_joint_p7_.publishIfNew();
  }

  void robotDUALARM::setJointTorque(const iiwa_msgs::JointTorque& torque)  {
    // TODO
  }

  bool robotDUALARM::getRobotTime(double& value) {

    bool temp = handler_state_time_.get(time_);
    
    value = static_cast<double>(time_.clock.sec) + static_cast<double>(time_.clock.nsec)*static_cast<double>(pow(10, -9));
    
    return temp;  
  }
  */

}


 

