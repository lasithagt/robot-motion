
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


#define KUKA_N_JOINTS 7 
#ifndef M_PI
#define M_PI 3.14159265358979
#endif

// define global variables
bool init = false;
bool init_time_bool = false;
bool init_f = false;
double f_thresh = -2;


Eigen::Vector3d f_data;
geometry_msgs::WrenchStamped netft_;


void getFT(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
	init_f = true;
	f_data(0) = msg->wrench.force.x;
	f_data(1) = msg->wrench.force.y;
	f_data(2) = msg->wrench.force.z;
}


void read_data(int &n_data_q, std::unique_ptr<Eigen::MatrixXf> &data_q) {

   	ROS_INFO_STREAM("Starting reading the trajectory file."); 
    n_data_q = read_txt_files::readFile_nlines(getenv("HOME") + std::string("/Documents/ROS_ws/src/kuka-motion/data/torsional_test.txt"));

    ROS_INFO_STREAM(n_data_q);

    data_q.reset(new Eigen::MatrixXf);
    data_q.get()->resize(KUKA_N_JOINTS, n_data_q); 

    int ret = read_txt_files::readFile(*(data_q.get()), getenv("HOME") + std::string("/Documents/ROS_ws/src/kuka-motion/data/torsional_test.txt"));
    ROS_INFO_STREAM("Finished reading the trajectory file.");
}

// example terminal conditions
bool term_cond(double time) {
	if (time < 3.0) {

		return false;
	}
	return true;
}



int main (int argc, char** argv) {

	const std::string node_name = "torsional_test";
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
	ros::Publisher netft_pub		   = nh_states.advertise<geometry_msgs::WrenchStamped>("/netft/state", 1);
	ros::Publisher pub_switch_dir	   = nh_states.advertise<geometry_msgs::PointStamped>("/state/switch", 1);
	// Subscribers
	ros::Subscriber state_netft        = nh_states.subscribe("/netft_data", 1, getFT);
	

	// type of polynomial
	std::string vel_prof_name = "cubic_polynomial";

	const double rate_interp = 500.0;

	// joint values
	double q[7]; 
	double hz = 500; double dt = 1 / hz;
	
	// define the robot class, this includes FWK and Jacobian functions
	robot_def::robot kuka_; 

	// robot interface for simulation with gazebo
	// robot_interface::robotKUKA_SIM robot_;
	robot_interface::robotKUKA robot_;
	robot_.init(nh_states, nh_command);

	iiwa_msgs::JointPosition position;
	position.position.quantity.resize(7);

	double duration_task = 200.0;
	double duration_init = 100.0;
	KDL::JntArray q_init(7);  KDL::JntArray q_final(7); 

	q_init.data(0) = -2.3; q_init.data(1) = 0; q_init.data(2) = -2.3; q_init.data(3) = 0; q_init.data(4) = -2.3; 
	q_init.data(5) = 0; q_init.data(6) = -2.3;
    

	// run the spinner
	ros::AsyncSpinner spinner(6); // Use 4 threads
    spinner.start();

    ros::Duration(2.0).sleep();

    double curr_time = 0.0;

    robot_.getRobotTime(curr_time);
    ROS_INFO_STREAM(curr_time);
    std_msgs::Bool command_active; command_active.data = true;

    // make the command active
    robot_motion_ready.publish(command_active);
    robot_motion_primitives::robot_motion robot_position = robot_motion_primitives::robot_motion(robot_, kuka_, 7, 600, vel_prof_name);
    robot_position.move_joint_to(q_init, position, duration_init);

	// q_final.data(0) = (*data_)(0,1); q_final.data(1) = (*data_)(1,1); q_final.data(2) = (*data_)(2,1); q_final.data(3) = (*data_)(3,1); q_final.data(4) = (*data_)(4,1); 
	// q_final.data(5) = (*data_)(5,1); q_final.data(6) = (*data_)(6,1);

	q_final.data(0) = 0; q_final.data(1) = 0; q_final.data(2) = 0; q_final.data(3) = 0; q_final.data(4) = 0; 
	q_final.data(5) = 0; q_final.data(6) = 0;
	
    while (ros::ok() || !ros::isShuttingDown()) {
    	ROS_INFO_STREAM("Starting in 10s..");
    	ros::Duration(20).sleep();

    	for (int i=0;i<5;i++) {

			if (robot_.getRobotIsConnected()) {


				robot_position.move_joint_to(q_final, position, duration_task);
				ROS_INFO_STREAM("Going back...");
				robot_position.move_joint_to(q_init, position, duration_init);

	    		ROS_INFO_STREAM("Wait for 30s...");
				ros::Duration(30).sleep();

			}
		}
		
	}

	command_active.data = false;
    robot_motion_ready.publish(command_active);

	ROS_INFO_STREAM("Shutting down...");
	ros::shutdown();
	

	return 0;

}





