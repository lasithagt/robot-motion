
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
Eigen::Vector3d f_data_prev;
geometry_msgs::WrenchStamped netft_;
geometry_msgs::PointStamped switch_iter;


void getFT(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
	init_f = true;
	f_data(0) = msg->wrench.force.x;
	f_data(1) = msg->wrench.force.y;
	f_data(2) = msg->wrench.force.z;
}


void read_data(int &n_data_q, std::unique_ptr<Eigen::MatrixXf> &data_q) {

   	ROS_INFO_STREAM("Starting reading the trajectory file."); 
    n_data_q = read_txt_files::readFile_nlines(getenv("HOME") + std::string("/Documents/ROS_ws/src/kuka-motion/data/desired_position_0.03.txt"));

    ROS_INFO_STREAM(n_data_q);

    data_q.reset(new Eigen::MatrixXf);
    data_q.get()->resize(KUKA_N_JOINTS, n_data_q); 

    int ret = read_txt_files::readFile(*(data_q.get()), getenv("HOME") + std::string("/Documents/ROS_ws/src/kuka-motion/data/desired_position_0.03.txt"));
    ROS_INFO_STREAM("Finished reading the trajectory file.");
}

bool term_cond(double time) {
	if (time < 3.0) {

		return false;
	}
	return true;
}

bool term_cond_2(double time) {
	if (time < 5.0) {
		return false;
	}
	return true;
}

bool term_cond_3(double time) {
	if (time < 0.5) {
		return false;
	}
	return true;
}


int main (int argc, char** argv) {

	const std::string node_name = "optical_sphere_trace";
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
	

	std::string vel_prof_name = "cubic_polynomial";

	const double rate_interp = 500.0;

	// create an array of evaluating points. These are points at which splines should be evaluated.
	int n_linspace = 36;
	int* eval_array = utils::linspace(10, n_data_q, n_linspace);

	// joint values
	double q[7]; 
	int term_FM = 0;
	double hz = 500; double dt = 1 / hz;
	int F_contact = 0;
	double vel_O = 0.001; // Linear velocity when doing orientation control

	// change the direction of movement.
	bool dir_1 = true; bool dir_2 = false; bool wait = true; double wait_start_time = 0.0; int dir_iter = 1; 
	int f_count = 10; int track_pos = 0;
	
	// define the robot class, this includes FWK and Jacobian functions
	robot_def::robot kuka_; 

	// robot interface for simulation with gazebo
	robot_interface::robotKUKA_SIM robot_;
	// robot_interface::robotKUKA robot_;
	robot_.init(nh_states, nh_command);

	iiwa_msgs::JointPosition position;
	position.position.quantity.resize(7);

	double duration = 6.0;
	double duration_init = 10.0;
	KDL::JntArray q_final(7); 
	q_final.data(0) = (*data_)(0,f_count); q_final.data(1) = (*data_)(1,f_count); q_final.data(2) = (*data_)(2,f_count); q_final.data(3) = (*data_)(3,f_count); q_final.data(4) = (*data_)(4,f_count); 
	q_final.data(5) = (*data_)(5,f_count); q_final.data(6) = (*data_)(6,f_count);
    

	Eigen::VectorXd qdot(7);

	// run the spinner
	ros::AsyncSpinner spinner(6); // Use 4 threads
    spinner.start();

    ros::Duration(2.0).sleep();

    double curr_time = 0.0;

    robot_.getRobotTime(curr_time);
    ROS_INFO_STREAM(curr_time);
    std_msgs::Bool command_active;

    command_active.data = true;
    robot_motion_ready.publish(command_active);

    robot_motion_primitives::robot_motion robot_position = robot_motion_primitives::robot_motion(robot_, kuka_, 7, 600, vel_prof_name);
    robot_position.move_joint_to(q_final, position, duration_init);


    while (ros::ok() || !ros::isShuttingDown()) {

		if (dir_1 || track_pos == n_linspace-1) {

			if (track_pos == n_linspace-1) {
				robot_position.move_constant_orientation(position, qdot, -vel_O, term_cond_2);
				dir_1 = false; dir_2 = true;
				f_count = eval_array[track_pos--];
			} else {
				f_count = eval_array[track_pos++];
			}

			
		} else if (dir_2 || track_pos == 0) {

			if (track_pos == 0) {
				robot_position.move_constant_orientation(position, qdot, -vel_O, term_cond_2);
				dir_1 = true; dir_2 = false;
				f_count = eval_array[track_pos++];
			} else {
				f_count = eval_array[track_pos--];
			}
		}

		if (robot_.getRobotIsConnected()) {
			q_final.data(0) = (*data_)(0,f_count); q_final.data(1) = (*data_)(1,f_count); q_final.data(2) = (*data_)(2,f_count); q_final.data(3) = (*data_)(3,f_count); q_final.data(4) = (*data_)(4,f_count); 
			q_final.data(5) = (*data_)(5,f_count); q_final.data(6) = (*data_)(6,f_count);
			robot_position.move_joint_to(q_final, position, duration);

    		// f_data_prev = f_data;
			Eigen::MatrixXf data = (*data_).block(0,10,7,100); 
    		robot_position.robot_motion::move_spline_to(data, position, duration);
			// robot_position.move_constant_orientation(position, qdot, vel_O, term_cond) ;
			// ros::Duration(1.5).sleep();
			// robot_position.move_constant_orientation(position, qdot, -vel_O, term_cond_3) ;
			ros::Duration(1.5).sleep();

		}
		
	}

	command_active.data = false;
    robot_motion_ready.publish(command_active);

	ROS_INFO_STREAM("Shutting down...");
	ros::shutdown();
	

	return 0;

}





