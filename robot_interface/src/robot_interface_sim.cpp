#include "robot_interface_sim.hpp"

namespace robot_interface {

  robotKUKA_SIM::robotKUKA_SIM(bool enableFT) : end_effector_ft_(enableFT) {}

  void robotKUKA_SIM::init(ros::NodeHandle& nh_states, ros::NodeHandle& nh_command)
  { 

      handler_state_joint_.init("/iiwa/joint_states", nh_states);

      handler_state_time_.init("/clock", nh_states);
      handler_command_time_.init("/iiwa/command/time", nh_states);


      handler_command_joint_p1_.init("/iiwa/PositionJointInterface_J1_controller/command", nh_command);
      handler_command_joint_p2_.init("/iiwa/PositionJointInterface_J2_controller/command", nh_command);
      handler_command_joint_p3_.init("/iiwa/PositionJointInterface_J3_controller/command", nh_command);
      handler_command_joint_p4_.init("/iiwa/PositionJointInterface_J4_controller/command", nh_command);
      handler_command_joint_p5_.init("/iiwa/PositionJointInterface_J5_controller/command", nh_command);
      handler_command_joint_p6_.init("/iiwa/PositionJointInterface_J6_controller/command", nh_command);
      handler_command_joint_p7_.init("/iiwa/PositionJointInterface_J7_controller/command", nh_command);

      // handler_command_torque_p1_.init("/iiwa/EffortJointInterface_J1_controller/command", nh_command);
      // handler_command_torque_p2_.init("/iiwa/EffortJointInterface_J2_controller/command", nh_command);
      // handler_command_torque_p3_.init("/iiwa/EffortJointInterface_J3_controller/command", nh_command);
      // handler_command_torque_p4_.init("/iiwa/EffortJointInterface_J4_controller/command", nh_command);
      // handler_command_torque_p5_.init("/iiwa/EffortJointInterface_J5_controller/command", nh_command);
      // handler_command_torque_p6_.init("/iiwa/EffortJointInterface_J6_controller/command", nh_command);
      // handler_command_torque_p7_.init("/iiwa/EffortJointInterface_J7_controller/command", nh_command);

      // initialize ft sensor readout.
      if (end_effector_ft_)
      {
        handler_state_wrench_.init("", nh_states);
      }


      last_update_time = ros::Time::now();
      ROS_INFO_STREAM("Initialized Simulation Subscribers... ");

  }

  bool robotKUKA_SIM::getCartesianWrench(geometry_msgs::WrenchStamped& value) 
  {
    // bool temp = handler_state_joint_.get(joint_state_);
    // if (temp) 
    // {
    //   memcpy(value.wrench.quantity.data(), joint_state_.effort.data(), 7*sizeof(double));
    // } 
    // return temp;
  }


  bool robotKUKA_SIM::getCartesianPose(geometry_msgs::PoseStamped& value) {
    // TODO
  }


  bool robotKUKA_SIM::getJointPosition(iiwa_msgs::JointPosition& value) 
  {

    bool temp = handler_state_joint_.get(joint_state_);
    // if (temp) 
    {
      value.header = joint_state_.header;
      memcpy(value.position.quantity.data(), joint_state_.position.data(), 7*sizeof(double));
    } 
    return temp;
  }


  bool robotKUKA_SIM::getJointPositionCommand(iiwa_msgs::JointPosition& value) {
    // TODO
  }

  bool robotKUKA_SIM::getJointTorque(iiwa_msgs::JointTorque& value) {
    bool temp = handler_state_joint_.get(joint_state_);
    if (temp) 
    {
      memcpy(value.torque.quantity.data(), joint_state_.effort.data(), 7*sizeof(double));
    } 
    return temp;
  }

  bool robotKUKA_SIM::getJointVelocity(iiwa_msgs::JointVelocity& value) 
  {
    bool temp = handler_state_joint_.get(joint_state_);
    // if (temp) 
    {
      memcpy(value.velocity.quantity.data(), joint_state_.velocity.data(), 7*sizeof(double));
    } 
    return temp;
  }

  void robotKUKA_SIM::setJointPosition(const iiwa_msgs::JointPosition& position)  
  {


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

    static std_msgs::Time command_time;
    command_time.data = ros::Time::now();
    handler_command_time_.set(command_time);
    handler_command_time_.publishIfNew();
  }

  void robotKUKA_SIM::setJointTorque(const iiwa_msgs::JointTorque& torque)  
  {
    std_msgs::Float64 temp;
    temp.data = torque.torque.quantity.at(0);
    handler_command_torque_p1_.set(temp);
    temp.data = torque.torque.quantity.at(1);
    handler_command_torque_p2_.set(temp);
    temp.data = torque.torque.quantity.at(2);
    handler_command_torque_p3_.set(temp);
    temp.data = torque.torque.quantity.at(3);
    handler_command_torque_p4_.set(temp);
    temp.data = torque.torque.quantity.at(4);
    handler_command_torque_p5_.set(temp);
    temp.data = torque.torque.quantity.at(5);
    handler_command_torque_p6_.set(temp);
    temp.data = torque.torque.quantity.at(6);
    handler_command_torque_p7_.set(temp);

    handler_command_torque_p1_.publishIfNew();
    handler_command_torque_p2_.publishIfNew();
    handler_command_torque_p3_.publishIfNew();
    handler_command_torque_p4_.publishIfNew();
    handler_command_torque_p5_.publishIfNew();
    handler_command_torque_p6_.publishIfNew();
    handler_command_torque_p7_.publishIfNew();
  }

  bool robotKUKA_SIM::getRobotTime(double& value) {

    bool temp = handler_state_time_.get(time_);
    value = static_cast<double>(time_.clock.sec) + static_cast<double>(time_.clock.nsec)*static_cast<double>(pow(10, -9));
    return temp;  
  }

  double robotKUKA_SIM::getRobotTimePy() {
    double value;
    getRobotTime(value);
    return value;
  }

}