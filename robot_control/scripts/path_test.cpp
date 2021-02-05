
#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <memory>
#include <string.h> 

#include <iostream>
#include <fstream>

#include <std_msgs/Time.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>

#include <robot_motion_library.hpp>
#include "robot_interface/robot_interface_sim.hpp"

#include "kuka_model.h"
#include "models.h"
#include "robot_interface_sim_wrench.hpp"
#include "robot_interface_real_wrench.hpp"

#include <utils.hpp>
#include <kuka_lib.h>

#include <algorithm>



#define KUKA_N_JOINTS 7 
#ifndef M_PI
#define M_PI 3.14159265358979
#endif

double degTorad(double deg)
{
	return deg * M_PI / 180;
}

// define global variables
bool init           = false;
bool init_time_bool = false;
bool init_f         = false;
bool sim            = false; // TODO make this dynamically update

// robot_interface::robotABSTRACT* robot_;
robot_interface::robotKUKAForceControl* robot_;


int main (int argc, char** argv) {


	const std::string node_name = "kuka_TEST";
	ros::init(argc, argv, node_name);
   	ros::NodeHandle nh_states("~");
   	ros::NodeHandle nh_command("~");


   
	// Publishers 
	ros::Publisher robot_motion_ready  = nh_command.advertise<std_msgs::Bool>("/kuka/command/active", 1);
	ros::Publisher pub_switch_dir	   = nh_states.advertise<geometry_msgs::PointStamped>("/state/switch", 1);
	

	// orocos kdl robot initialization
	ros_kuka::KUKAModelKDLInternalData robotParams;
	robotParams.numJoints = 7;
	robotParams.Kv = Eigen::MatrixXd(7,7);
	robotParams.Kp = Eigen::MatrixXd(7,7);

	Eigen::Matrix<double,4,4> T = Eigen::MatrixXd::Identity(4,4);
	T(1,1) = -1; T(2,2) = -1;
	robotParams.baseTransform = T;

	// Define the robot and contact model 
	ros_kuka::KukaDHKdl robot = ros_kuka::KukaDHKdl();
	KDL::Chain chain = robot();

	std::shared_ptr<RobotAbstract> kuka_robot = std::shared_ptr<RobotAbstract>(new ros_kuka::KUKAModelKDL(chain, robotParams));
	kuka_robot->initRobot();


	robot_interface::FT_Transform forceTransform;

	// Subscribers
	if (!sim) 
	{
		robot_ = new robot_interface::robotKUKA_WRENCH(kuka_robot, forceTransform, false);
		robot_->init(nh_states, nh_command);
		ROS_INFO_STREAM("Initialized Real Robot...");
	} else 
	{
		robot_ = new robot_interface::robotKUKA_SIM_WRENCH(kuka_robot, forceTransform, false);
		robot_->init(nh_states, nh_command);
		ROS_INFO_STREAM("Initialized Gazebo Simulation...");
	}

	
	std::string vel_prof_name = "cubic_polynomial";
	const double rate_interp = 500.0;
	ros::Rate rate_(rate_interp);


	// joint values
	double q[7]; 
	int term_FM = 0;
	double hz = 500; double dt = 1 / hz;
	int F_contact = 0;


	iiwa_msgs::JointPosition position;
	position.position.quantity.resize(7);

	iiwa_msgs::JointPosition position_ref;
	position_ref.position.quantity.resize(7);

	iiwa_msgs::JointPosition position_cmd;
	position_cmd.position.quantity.resize(7);

	double duration = 10.0;
	double duration_init = 15.0;
	KDL::JntArray q_final(7); 
	int f_count = 0; 	


	Eigen::VectorXd qdot(7);

	ROS_INFO_STREAM("Initializing the spinner...");
	// run the spinner
	ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();

	// wait until everything is initialized
    ros::Duration(1.0).sleep();

    ROS_INFO_STREAM("Finished initializing the spinner...");
    ROS_INFO_STREAM(robot_->getRobotIsConnected());

    double curr_time = 0.0;

    robot_->getRobotTime(curr_time);
    ROS_INFO_STREAM(curr_time);
    std_msgs::Bool command_active;

    command_active.data = true;
    robot_motion_ready.publish(command_active);

    robot_motion_primitives::robot_motion robot_position = robot_motion_primitives::robot_motion(*robot_, kuka_robot, 7, 200, vel_prof_name);

    // variables for the force modulation
    Eigen::Vector3d v_ref(0,0,1);	
    Eigen::VectorXd q_next(7);
    double q_f[7];


	double qd[7] = {0,0,0,0,0,0,0};
	double qdd[7] = {0,0,0,0,0,0,0};
	Eigen::Matrix<double, 3, 3> poseM;
	Eigen::Vector3d poseP;
	Eigen::Vector3d vel;
	Eigen::Vector3d accel;

	position.position.quantity.at(0) = degTorad(90);
	position.position.quantity.at(1) = 0*degTorad(29.10);
	position.position.quantity.at(2) = 0*degTorad(22.75);
	position.position.quantity.at(3) = 0*degTorad(57.32);
	position.position.quantity.at(4) = 0*degTorad(0);
	position.position.quantity.at(5) = 0*degTorad(28.60);
	position.position.quantity.at(6) = 0*degTorad(0.02);

	kuka_robot->getForwardKinematics(position.position.quantity.data(), qd, qdd, poseM, poseP, vel, accel, false);

	Eigen::MatrixXd jacobian(6,7);

	kuka_robot->getSpatialJacobian(position.position.quantity.data(), jacobian);

	Eigen::VectorXd qdot_(7);
	qdot_(1) = 2;
	auto temp = jacobian * qdot_;

	ROS_INFO_STREAM(temp);
	ROS_INFO_STREAM("\n");
	ROS_INFO_STREAM(poseM);
	ROS_INFO_STREAM("position");
	ROS_INFO_STREAM(poseP);



	// if (robot_->getRobotIsConnected()) 
	// {
	// 	ROS_INFO_STREAM("Connected to Robot..");
	// 	// set these temporally
	// 	q_final.data(0) = 0; q_final.data(1) = 0.5; q_final.data(2) = 0.4; q_final.data(3) = 0.5*2; q_final.data(4) = 0; 
	// 	q_final.data(5) = 0.5; q_final.data(6) = 0;

	// 	robot_position.move_joint_to_wrench(q_final, position_ref, duration_init, true);
	// 	// robot_position.move_joint_to(q_final, position_ref, duration_init);
	// 	ROS_INFO_STREAM("Moved to the initial point...");

	// }

    ROS_INFO_STREAM("Completed initial positioning...");

    // compute the orientation error
    ROS_INFO_STREAM("Sleeping for 1 seconds...");
    ros::Duration(1.0).sleep();

    

    robot_->getJointPosition(position);
    position_cmd = position;

	KDL::JntArray q_curr(7);

	// get current time from the robot
	robot_->getRobotTime(curr_time);

	// double qd[7] = {0,0,0,0,0,0,0};
	// double qdd[7] = {0,0,0,0,0,0,0};
	// Eigen::Matrix<double, 3, 3> poseM;
	// Eigen::Vector3d poseP;
	// Eigen::Vector3d vel;
	// Eigen::Vector3d accel;

    // check if robot object is initialized first. for command_position --> shared pointer
    while (ros::ok() && !ros::isShuttingDown()) {

    	// robot_->getJointPosition(position);
    	// kuka_robot->getForwardKinematics(position.position.quantity.data(), qd, qdd, poseM, poseP, vel, accel, false);


    	// ROS_INFO_STREAM(poseM);
    	// ROS_INFO_STREAM("position");
    	// ROS_INFO_STREAM(poseP);

		// ROS_INFO_STREAM("Updating the reference trajectory");

		// // commented out such it will not move
		// memcpy(position_ref.position.quantity.data(), q_curr.data.data(), 7*sizeof(double));


		// for (int i=0;i < 7;i++) {
		// 	position_cmd.position.quantity[i] = position_ref.position.quantity[i];
		// }

		// // Set the next position
		// robot_->setJointPosition(position_cmd);

		// // get the curr time
		// robot_->getRobotTime(curr_time);
		// new_time = curr_time - init_time;

		// rate_.sleep();

	}

	command_active.data = false;
    robot_motion_ready.publish(command_active);

	ROS_INFO_STREAM("Shutting down...");
	ros::shutdown();
	

	return 0;

}





