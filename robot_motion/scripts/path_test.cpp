
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
#include "robot_interface/robot_class.hpp"
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
bool sim = true;

Eigen::Vector3d f_data;
Eigen::Vector3d kd_data;
Eigen::Vector3d f_data_prev;
robot_interface::robotABSTRACT* robot_;


void getKD(const geometry_msgs::PointStamped::ConstPtr& msg) {
	kd_data(0) = msg->point.x;
	kd_data(1) = msg->point.y;
	kd_data(2) = msg->point.z;

}

void read_data(int &n_data_q, std::unique_ptr<Eigen::MatrixXf> &data_q) {
   	ROS_INFO_STREAM("Starting reading the trajectory file."); 
    n_data_q = read_txt_files::readFile_nlines(getenv("HOME") + std::string("/Documents/ROS_ws/src/kuka_motion/robot_motion/data/desired_position_FM_dynamic_circle.txt"));

    ROS_INFO_STREAM(n_data_q);

    data_q.reset(new Eigen::MatrixXf);
    data_q.get()->resize(KUKA_N_JOINTS, n_data_q); 

    int ret = read_txt_files::readFile(*(data_q.get()), getenv("HOME") + std::string("/Documents/ROS_ws/src/kuka_motion/robot_motion/data/desired_position_FM_dynamic_circle.txt"));
    ROS_INFO_STREAM("Finished reading the trajectory file.");
}



int main (int argc, char** argv) {


	const std::string node_name = "kuka_FM";
	ros::init(argc, argv, node_name);
   	ros::NodeHandle nh_states("~");
   	ros::NodeHandle nh_command("~");

   	// Read from file. Make to read from the paramserver 
   	int n_data_q;
    std::unique_ptr<Eigen::MatrixXf> data_q;
    
    read_data(n_data_q, data_q);
    Eigen::MatrixXf* data_ = data_q.get();
   
	// Publishers 
	ros::Publisher robot_motion_ready  = nh_command.advertise<std_msgs::Bool>("/kuka/command/active", 1);
	// ros::Publisher netft_pub		   = nh_states.advertise<geometry_msgs::WrenchStamped>("/netft/state", 1);
	ros::Publisher pub_switch_dir	   = nh_states.advertise<geometry_msgs::PointStamped>("/state/switch", 1);
	

	// Subscribers
	if (!sim) {
		robot_ = new robot_interface::robotKUKA();
	} else {
		robot_ = new robot_interface::robotKUKA_SIM();
	}

	ros::Subscriber state_stiffness = nh_states.subscribe("/stiffness_id/KD", 1, getKD);

	
	std::string vel_prof_name = "cubic_polynomial";

	const double rate_interp = 500.0;
	ros::Rate rate_(rate_interp);


	// joint values
	double q[7]; 
	int term_FM = 0;
	double hz = 500; double dt = 1 / hz;
	int F_contact = 0;


	// define the robot class, this includes FWK and Jacobian functions
	robot_def::robot kuka_; 

	robot_->init(nh_states, nh_command);
	ROS_INFO_STREAM("Initialized Robot...");


	iiwa_msgs::JointPosition position;
	position.position.quantity.resize(7);

	iiwa_msgs::JointPosition position_ref;
	position_ref.position.quantity.resize(7);

	iiwa_msgs::JointPosition position_cmd;
	position_cmd.position.quantity.resize(7);

	double duration = 10.0;
	double duration_init = 40.0;
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
    // robot_position.move_joint_to(q_final, position, duration_init);

    // variables for the force modulation
    Eigen::Vector3d v_ref(0,0,1);	
    Eigen::VectorXd q_next(7);
    double q_f[7];


	if (robot_->getRobotIsConnected()) {
		q_final.data(0) = (*data_)(0,f_count); q_final.data(1) = (*data_)(1,f_count); q_final.data(2) = (*data_)(2,f_count); q_final.data(3) = (*data_)(3,f_count); q_final.data(4) = (*data_)(4,f_count); 
		q_final.data(5) = (*data_)(5,f_count); q_final.data(6) = (*data_)(6,f_count);

		// set these temporally
		// q_final.data(0) = 0; q_final.data(1) = 0.8727; q_final.data(2) = 0; q_final.data(3) = 0.8727*2; q_final.data(4) = 0; 
		// q_final.data(5) = 0.8727; q_final.data(6) = 0;

		robot_position.move_joint_to(q_final, position_ref, duration_init);
		ROS_INFO_STREAM("Moved to the initial point...");

	}

    ROS_INFO_STREAM("Completed initial positioning...");
    // compute the orientation error
    ROS_INFO_STREAM("Sleeping for 1 seconds...");
    ros::Duration(1.0).sleep();

    


    robot_->getJointPosition(position);
    position_cmd = position;

	KDL::JntArray q_curr(7);

	curr_time = 0.0;

	// get current time from the robot
	robot_->getRobotTime(curr_time);
	const double init_time = curr_time;

	double new_time = curr_time - init_time;
	memcpy(q_curr.data.data(), position.position.quantity.data(),  7*sizeof(double));


    std::vector<ecl::CubicSpline> splines_;
    robot_position.move_spline_to_ret(*data_, position, splines_, duration);
    ROS_INFO_STREAM("Starting the Trajectory...");

    // check if robot object is initialized first. for command_position --> shared pointer
    while (ros::ok() && !ros::isShuttingDown() && (new_time < splines_[0].domain().back()) && (new_time < duration)) {


        for (int i = 0; i < 7;i++) {
            q_curr(i) = splines_[i](new_time);
        }

        ROS_INFO_STREAM("Updating the reference trajectory");

        // commented out such it will not move
		memcpy(position_ref.position.quantity.data(), q_curr.data.data(), 7*sizeof(double));
    	

		for (int i=0;i < 7;i++) {
			position_cmd.position.quantity[i] = position_ref.position.quantity[i];
		}

		// ROS_INFO_STREAM(position_ref.position);
		// Set the next position
		robot_->setJointPosition(position_cmd);

    	// get the curr time
    	robot_->getRobotTime(curr_time);
		new_time = curr_time - init_time;
		// Eigen::Quaterniond rotatedP = q * p * q.inverse(); 

		// robot_position.move_constant_orientation(position, qdot, vel_O, term_cond) ;
		
		rate_.sleep();

	}

	command_active.data = false;
    robot_motion_ready.publish(command_active);

	ROS_INFO_STREAM("Shutting down...");
	ros::shutdown();
	

	return 0;

}





