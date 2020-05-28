
#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <memory>
#include <string.h> 
#include <cmath>

#include <iostream>
#include <fstream>

#include <std_msgs/Time.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>

#include <robot_kinematics.hpp>
#include <robot_motion_library.hpp>
#include <robot_class.hpp>
#include <utils.hpp>
#include <kuka_lib.h>

#include <boost/program_options.hpp>

#include <algorithm>
#include <nlopt.hpp>
#include <iomanip>


#define KUKA_N_JOINTS 7 
#ifndef M_PI
#define M_PI 3.14159265358979
#endif

// define global variables
bool init = false;
bool init_time_bool = false;
bool init_f = false;
bool sim = false;

Eigen::Vector3d f_data;
Eigen::Vector3d kd_data;
Eigen::Vector3d f_data_prev;
robot_interface::robotABSTRACT* robot_;



void read_data(int &n_data_q, std::unique_ptr<Eigen::MatrixXf> &data_q) {

   	ROS_INFO_STREAM("Starting reading the trajectory file."); 
    // n_data_q = read_txt_files::readFile_nlines(getenv("HOME") + std::string("/Documents/ROS_ws/src/kuka-motion/data/desired_position_wrench.txt"));
    n_data_q = read_txt_files::readFile_nlines(getenv("HOME") + std::string("/Documents/ROS_ws/src/kuka-motion/data/identification_friction_linear.txt"));
    // n_data_q = read_txt_files::readFile_nlines(getenv("HOME") + std::string("/Documents/ROS_ws/src/kuka-motion/data/identification_stiffness.txt"));


    ROS_INFO_STREAM("Number of data points");
    ROS_INFO_STREAM(n_data_q);

    data_q.reset(new Eigen::MatrixXf);
    data_q.get()->resize(KUKA_N_JOINTS+6, n_data_q); 

    // int ret = read_txt_files::readFile(*(data_q.get()), getenv("HOME") + std::string("/Documents/ROS_ws/src/kuka-motion/data/desired_position_wrench.txt"));
    int ret = read_txt_files::readFile(*(data_q.get()), getenv("HOME") + std::string("/Documents/ROS_ws/src/kuka-motion/data/identification_friction_linear.txt"));
    // int ret = read_txt_files::readFile(*(data_q.get()), getenv("HOME") + std::string("/Documents/ROS_ws/src/kuka-motion/data/identification_stiffness.txt"));

    ROS_INFO_STREAM("Finished reading the trajectory file.");
}



int main (int argc, char** argv) {


	const std::string node_name = "kuka_FM";
	ros::init(argc, argv, node_name);
   	ros::NodeHandle nh_states("~");
   	ros::NodeHandle nh_command("~");

   	// Read from file. Make to read from the paramserver 
   	int n_data_q_w;
    std::unique_ptr<Eigen::MatrixXf> data_q_w;

    
    read_data(n_data_q_w, data_q_w);
    Eigen::MatrixXf* data_ = data_q_w.get();

   
	// Publishers 
	ros::Publisher robot_motion_ready  = nh_command.advertise<std_msgs::Bool>("/kuka/command/active", 1);
	ros::Publisher pub_switch_dir	   = nh_states.advertise<geometry_msgs::PointStamped>("/state/switch", 1);
	

	// Subscribers
	if (!sim) {
		robot_ = new robot_interface::robotKUKA();
	} else {
		robot_ = new robot_interface::robotKUKA_SIM();
	}
	
	std::string vel_prof_name = "cubic_polynomial";

	const double rate_interp = 500.0;
	ros::Rate rate_(rate_interp);


	// joint values
	double q[7]; 
	double hz = 500; double dt = 1 / hz;


	// define the robot class, this includes FWK and Jacobian functions
	robot_def::robot kuka_; 
	robot_->init(nh_states, nh_command);
	ROS_INFO_STREAM("Initialized Robot...");


	iiwa_msgs::JointPosition position;
	position.position.quantity.resize(7);

	iiwa_msgs::JointPosition position_ref;
	position_ref.position.quantity.resize(7);

	iiwa_msgs::Wrench wrench_ref;
	wrench_ref.wrench.quantity.resize(7);


	double duration = 30.0;
	double duration_init = 30.0;
	KDL::JntArray q_final(7); 
	int f_count = 0; 	

	q_final.data(0) = (*data_)(0,f_count); q_final.data(1) = (*data_)(1,f_count); q_final.data(2) = (*data_)(2,f_count); q_final.data(3) = (*data_)(3,f_count); q_final.data(4) = (*data_)(4,f_count); 
	q_final.data(5) = (*data_)(5,f_count); q_final.data(6) = (*data_)(6,f_count);
    

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

    robot_motion_primitives::robot_motion robot_position = robot_motion_primitives::robot_motion(*robot_, kuka_, 7, 600, vel_prof_name);


    // moving the robot to the initial position
	if (robot_->getRobotIsConnected()) {
		ROS_INFO_STREAM("Moved to the initial point...");

		q_final.data(0) = (*data_)(0,f_count); q_final.data(1) = (*data_)(1,f_count); q_final.data(2) = (*data_)(2,f_count); q_final.data(3) = (*data_)(3,f_count); q_final.data(4) = (*data_)(4,f_count); 
		q_final.data(5) = (*data_)(5,f_count); q_final.data(6) = (*data_)(6,f_count);
		bool is_wrench = true;
		robot_position.move_joint_to_wrench(q_final, position_ref, duration_init, is_wrench);
		
		ROS_INFO_STREAM("Completed initial positioning...");
	}
    
    // compute the orientation error
    ROS_INFO_STREAM("Sleeping for 1 seconds...");
    ros::Duration(1.0).sleep();

    robot_->getJointPosition(position);

	KDL::JntArray q_curr(7);


	// get current time from the robot
	curr_time = 0.0;
	robot_->getRobotTime(curr_time);
	const double init_time = curr_time;

	double new_time = curr_time - init_time;
	memcpy(q_curr.data.data(), position.position.quantity.data(),  7*sizeof(double));


    ROS_INFO_STREAM("Starting the Trajectory...");
    int iter = 0;

    // check if robot object is initialized first. for command_position --> shared pointer
    while (ros::ok() && !ros::isShuttingDown() && (new_time < duration) && (iter<n_data_q_w)) {


        for (int i = 0; i < 7;i++) {
            q_curr(i) = (*data_)(i,iter);
        }

        iter++;

        ROS_INFO_STREAM("Updated the reference trajectory");

        // commented out such it will not move
		memcpy(position_ref.position.quantity.data(), q_curr.data.data(), 7*sizeof(double));

		// Set the next position
		position_ref.header.stamp = ros::Time::now();
		ROS_INFO_STREAM(position_ref);
		robot_->setJointPosition(position_ref);
		
		wrench_ref.wrench.quantity[0] = 0;
		wrench_ref.wrench.quantity[1] = 0;
		wrench_ref.wrench.quantity[2] = 4;
		wrench_ref.header.stamp = ros::Time::now();


		// robot_->setWrench(wrench_ref);

    	// get the curr time
    	robot_->getRobotTime(curr_time);
		new_time = curr_time - init_time;

		
		rate_.sleep();

	}

	command_active.data = false;
    robot_motion_ready.publish(command_active);

	ROS_INFO_STREAM("Shutting down...");
	ros::shutdown();
	

	return 0;

}





