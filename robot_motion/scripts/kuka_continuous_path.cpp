
// Example code for kuka torque control.

#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <memory>
#include <string.h> 
#include "kuka_lib.h"

#include <iostream>
#include <fstream>
#include <boost/tokenizer.hpp>

#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointTorque.h>
#include <iiwa_msgs/JointVelocity.h>
#include <iiwa_msgs/JointPositionVelocity.h>
#include <std_msgs/Time.h>
#include <std_msgs/Bool.h>


#include <memory>
#include <boost/program_options.hpp>

#include <algorithm>
#include <boost/lexical_cast.hpp>

#include <utils.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979
#endif


KDL::JntArray q0(7);

bool init = false;
bool init_time_bool = false;
KDL::JntArray q_position(7);
KDL::JntArray q_position_Ipo(7);
KDL::JntArray q_velocity(7);
KDL::JntArray q_actual_torque(7);
KDL::JntArray q_ext_torque(7);
double init_time = 0.0;
std_msgs::Time curr_kuka_time;
std_msgs::Time init_kuka_time;


void getKUKATorque(const iiwa_msgs::JointTorque::ConstPtr& msg) {
	memcpy(q_actual_torque.data.data(), msg->torque.quantity.data(), 7*sizeof(double));
}

void getKUKAExtTorque(const iiwa_msgs::JointTorque::ConstPtr& msg) {
	memcpy(q_ext_torque.data.data(), msg->torque.quantity.data(), 7*sizeof(double));
}

void getKUKAJoint(const iiwa_msgs::JointPosition::ConstPtr& msg) {
	// Do not let it run util it knows the initial position
	if (init == false) {
		init = true;
	}	
	memcpy(q_position.data.data(), msg->position.quantity.data(), 7*sizeof(double));
}

void getKUKAJointIpo(const iiwa_msgs::JointPosition::ConstPtr& msg) {
	memcpy(q_position_Ipo.data.data(), msg->position.quantity.data(), 7*sizeof(double));
}

// Gets the KUKA joint velocity
void getKUKAVelocity(const iiwa_msgs::JointVelocity::ConstPtr& msg) {
	// Do not let it run util it knows the initial position
	if (init == false) {
		init = true;
	}	
	memcpy(q_velocity.data.data(), msg->velocity.quantity.data(), 7*sizeof(double));
}

void getKUKATime(const std_msgs::Time::ConstPtr& msg) {

	if (init_time_bool == false) {
		init_time_bool = true;
		init_kuka_time.data.sec  = (msg->data.sec);
		init_kuka_time.data.nsec = (msg->data.nsec);
		curr_kuka_time.data.sec  = (msg->data.sec);
		curr_kuka_time.data.nsec = (msg->data.nsec);
		init_time = static_cast<double>(init_kuka_time.data.sec) + static_cast<double>(init_kuka_time.data.nsec)*static_cast<double>(pow(10, -9));
	} else {
		curr_kuka_time.data.sec  = (msg->data.sec);// - (init_kuka_time.data.sec);
		curr_kuka_time.data.nsec = (msg->data.nsec);//- (init_kuka_time.data.nsec);
	}
}


using namespace joint_space_vel_prof;

int main (int argc, char** argv) {

	const std::string node_name = "kuka_continuous_path";
	ros::init(argc, argv, node_name);
   	ros::NodeHandle nh("~");
   	
   	float pub_rate_hz;

    //----------------------------------------------------------------------------------------------------------------------------------------
   	// Read from file. Make to read from the paramserver
   	int n_data_q;
   	ROS_INFO_STREAM("Starting reading the trajectory file."); 
    n_data_q = read_txt_files::readFile_nlines(getenv("HOME") + std::string("/Documents/ROS_ws/kuka_ws/src/kuka_motion_lib/data/desired_position.txt"));


    ROS_INFO_STREAM(n_data_q);
    Eigen::MatrixXf data_q(7, n_data_q);


    int ret = read_txt_files::readFile(data_q, getenv("HOME") + std::string("/Documents/ROS_ws/kuka_ws/src/kuka_motion_lib/data/desired_position.txt"));
    ROS_INFO_STREAM("Finished reading the trajectory file.");
    // -----------------------------------------------------------------------------------------------------------------------------------------

    

	// Publishers 
	ros::Publisher pub_position 	   = nh.advertise<iiwa_msgs::JointPosition>("/kuka/command/command_position", 1);
	ros::Publisher kuka_motion_ready   = nh.advertise<std_msgs::Bool>("/kuka/command/active", 1);

	// Subscribers
	ros::Subscriber state_torque       = nh.subscribe("/kuka/state/KUKAActualTorque", 1, getKUKATorque); 
	ros::Subscriber state_ext_torque   = nh.subscribe("/kuka/state/KUKAExtTorque", 1, getKUKATorque); 
	ros::Subscriber state_position     = nh.subscribe("/kuka/state/KUKAJointPosition", 1, getKUKAJoint);
	ros::Subscriber state_position_Ipo = nh.subscribe("/kuka/state/KUKAJointPositionIpo", 1, getKUKAJointIpo);  
	ros::Subscriber state_kuka_time    = nh.subscribe("/kuka/state/KUKATime", 1, getKUKATime);
	ros::Subscriber state_velocity     = nh.subscribe("/kuka/state/KUKAJointVelocity", 1, getKUKAVelocity);
	

	std::unique_ptr<velocityProfile> vel_prof;

	double m_vel[] = {1,1,1,1,1,1,1};
	std::string vel_prof_name = "cubic_polynomial";
	joint_space_vel_prof::getJointSpaceVelProfile(vel_prof_name, vel_prof, m_vel);

	KDL::JntArray q_curr(7); 
	KDL::JntArray qdot_curr(7);
	KDL::JntArray q_pos(7);
	KDL::JntArray qf(7); 
	// std_msgs::Int16 command_active = 0;
	 
	q0(0) = 1; q0(1) = 0; q0(2) = 0; q0(3) = 0; q0(4) = 0; q0(5) = 0; q0(6) = 0;
	qf(0) = data_q(0,0); qf(1) = data_q(1,0); qf(2) = data_q(2,0); qf(3) = data_q(3,0); qf(4) = data_q(4,0); qf(5) = data_q(5,0); qf(6) = data_q(6,0);
	double duration = 10;

	// Wait for the initial update. Makse sure the it is initilized properly
	while (!init || !init_time_bool) {ros::spinOnce();}
	
	// To make sure current position it recived
	if (init && init_time_bool) {
		q0.data = q_position.data;
		vel_prof->solve(q0, qf, duration);
		ROS_INFO_STREAM("Initial position recived and path is planned...");
	}


	// Set the data publishing rate
	ros::Rate rate(1000);
	double curr_time = 0.0;
	iiwa_msgs::JointPosition command_position;
	command_position.position.quantity.resize(7);

	// spline interpolation
	std::vector<alglib::spline1dinterpolant> splines;
	splines.resize(7);
	const int n_data = n_data_q;
	double start_time_1 = 15.0;
	double stop_time_1 = 60.0;

	// build splines for each joint
	// spline interpolation for direction 1
	std::vector<alglib::spline1dinterpolant> splines_p1;
	splines_p1.resize(7);
	spline_traj::spline_piecewise_interp(splines_p1, data_q, start_time_1, stop_time_1);

	// spline interpolation for direction 2
	Eigen::MatrixXf data_q2(7, n_data_q);
	// data_q2 = data_q;
	// data_q2.colwise().reverse();

	for (int i = 0;i < n_data_q; i++) {
		data_q2.col(i) = data_q.col(n_data-1-i);
	}

	std::vector<alglib::spline1dinterpolant> splines_p2;
	splines_p2.resize(7);
	double start_time_2 = 70.0;
	double stop_time_2 = 110.0;
	spline_traj::spline_piecewise_interp(splines_p2, data_q2, start_time_2, stop_time_2);

	int step = 1;
	bool dir_1 = true; bool dir_2 = false;
	std_msgs::Bool command_active;
	command_active.data = false;

	while(ros::ok()) {
		/* Increment the time */
		curr_time = static_cast<double>(curr_kuka_time.data.sec) + static_cast<double>(curr_kuka_time.data.nsec)*static_cast<double>(pow(10, -9)) - init_time;
		ROS_INFO_STREAM(curr_time);

		if (curr_time < duration) {
			vel_prof->get_desired_joint_pos(q_curr, curr_time);
			memcpy(q_pos.data.data(), q_curr.data.data(), 7*sizeof(double));
			memcpy(command_position.position.quantity.data(), q_pos.data.data(), 7*sizeof(double));
			command_active.data = true;

		} else if(curr_time < (stop_time_1) && curr_time > start_time_1) {
			for (int i = 0;i < 7;i++) {
				q_pos(i) = (double)spline1dcalc(splines_p1[i], curr_time);
			}
			memcpy(command_position.position.quantity.data(), q_pos.data.data(), 7*sizeof(double));
			command_active.data = true;

		} else if(curr_time < (stop_time_2) && curr_time > start_time_2) {
			for (int i = 0;i < 7;i++) {
				q_pos(i) = (double)spline1dcalc(splines_p2[i], curr_time);
			}
			memcpy(command_position.position.quantity.data(), q_pos.data.data(), 7*sizeof(double));
			command_active.data = true;	

		} else {

			command_active.data = true;
		}



		//Publish the generated torque and the time stamp 

		command_position.header.stamp = ros::Time::now();

		kuka_motion_ready.publish(command_active);
		pub_position.publish(command_position);
		rate.sleep();
		ros::spinOnce();

	}

	return 0;

	}





