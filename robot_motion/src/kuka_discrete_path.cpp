
// Example code for kuka torque control.

#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <memory>
#include <string.h> 
#include "kuka_lib.h"
#include <cmath>

#include <iostream>
#include <fstream>
#include <boost/tokenizer.hpp>

#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointTorque.h>
#include <iiwa_msgs/JointVelocity.h>
#include <iiwa_msgs/JointPositionVelocity.h>
#include <std_msgs/Time.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <robot_kinematics.hpp>
#include <kuka_motion_library.hpp>

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
bool init_f = false;
KDL::JntArray q_position(7);
KDL::JntArray q_position_state_command(7);
KDL::JntArray q_velocity(7);
KDL::JntArray q_actual_torque(7);
KDL::JntArray q_ext_torque(7);
Eigen::Vector3d f_data;
Eigen::Vector3d f_data_prev;
geometry_msgs::WrenchStamped netft_;
geometry_msgs::PointStamped switch_iter;

double init_time = 0.0;
std_msgs::Time curr_kuka_time;
std_msgs::Time init_kuka_time;


void getKUKATorque(const iiwa_msgs::JointTorque::ConstPtr& msg) {
	memcpy(q_actual_torque.data.data(), msg->torque.quantity.data(), 7*sizeof(double));
}

void getKUKAExtTorque(const iiwa_msgs::JointTorque::ConstPtr& msg) {
	memcpy(q_ext_torque.data.data(), msg->torque.quantity.data(), 7*sizeof(double));
}

void getKUKAJoint(const ros::MessageEvent<iiwa_msgs::JointPosition const>& event) {
	// Do not let it run util it knows the initial position
	ros::Time receipt_time = event.getReceiptTime();
	const iiwa_msgs::JointPosition::ConstPtr& msg = event.getMessage();
	double dt = abs(receipt_time.toSec() - ros::Time::now().toSec());

	if ((init == false) && dt<0.1) {
		init = true;
	}	
	memcpy(q_position.data.data(), msg->position.quantity.data(), 7*sizeof(double));
}

void getKUKAJointCommand(const iiwa_msgs::JointPosition::ConstPtr& msg) {
	memcpy(q_position_state_command.data.data(), msg->position.quantity.data(), 7*sizeof(double));
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

void getFT(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
	init_f = true;
	f_data(0) = msg->wrench.force.x;
	f_data(1) = msg->wrench.force.y;
	f_data(2) = msg->wrench.force.z;
}


/* returns 1 if force is computed */
int constant_orientation(robot_def::robot kuka, const double* q, Eigen::VectorXd &qdot, double vel) {
	int term_cond = 0;

	robot_motion_primitives::move_null_orientation(kuka, q, qdot, vel);

	if (qdot.norm()>1.5) {
		term_cond = 1;
	}

	return term_cond;
}

// publishes data to respective publishers
void publish_ft_data() {

}

using namespace joint_space_vel_prof;
int main (int argc, char** argv) {

	const std::string node_name = "kuka_continuous_path";
	ros::init(argc, argv, node_name);
   	ros::NodeHandle nh("~");
   	ros::NodeHandle nh_comm("~");

   	float pub_rate_hz;

    //----------------------------------------------------------------------------------------------------------------------------------------
   	// Read from file. Make to read from the paramserver
   	int n_data_q;
   	ROS_INFO_STREAM("Starting reading the trajectory file."); 
    n_data_q = read_txt_files::readFile_nlines(getenv("HOME") + std::string("/Documents/ROS_ws/kuka_ws/src/kuka_motion_lib/data/desired_position_0.03.txt"));


    ROS_INFO_STREAM(n_data_q);
    Eigen::MatrixXf data_q(7, n_data_q);


    int ret = read_txt_files::readFile(data_q, getenv("HOME") + std::string("/Documents/ROS_ws/kuka_ws/src/kuka_motion_lib/data/desired_position_0.03.txt"));
    ROS_INFO_STREAM("Finished reading the trajectory file.");
    // -----------------------------------------------------------------------------------------------------------------------------------------

   
	// Publishers 
	ros::Publisher pub_position 	   = nh_comm.advertise<iiwa_msgs::JointPosition>("/kuka/command/command_position", 1);
	ros::Publisher kuka_motion_ready   = nh_comm.advertise<std_msgs::Bool>("/kuka/command/active", 1);
	ros::Publisher netft_pub		   = nh.advertise<geometry_msgs::WrenchStamped>("/netft/state", 1);
	ros::Publisher pub_switch_dir	   = nh.advertise<geometry_msgs::PointStamped>("/state/switch", 1);

	// Subscribers
	ros::Subscriber state_torque       = nh.subscribe("/kuka/state/KUKAActualTorque", 1, getKUKATorque); 
	ros::Subscriber state_ext_torque   = nh.subscribe("/kuka/state/KUKAExtTorque", 1, getKUKATorque); 
	ros::Subscriber state_position     = nh.subscribe("/kuka/state/KUKAJointPosition", 1, getKUKAJoint);
	ros::Subscriber state_position_com = nh.subscribe("/kuka/state/KUKAJointPositionCommand", 1, getKUKAJointCommand);  
	ros::Subscriber state_kuka_time    = nh.subscribe("/kuka/state/KUKATime", 1, getKUKATime);
	ros::Subscriber state_velocity     = nh.subscribe("/kuka/state/KUKAJointVelocity", 1, getKUKAVelocity);
	ros::Subscriber state_netft        = nh.subscribe("/netft_data", 1, getFT);
	

	std::unique_ptr<velocityProfile> vel_prof;

	double m_vel[] = {1,1,1,1,1,1,1};
	// std::string vel_prof_name = "cubic_polynomial";
	std::string vel_prof_name = "quintic_polynomial";
	joint_space_vel_prof::getJointSpaceVelProfile(vel_prof_name, vel_prof, m_vel);

	// variable to be used in this 
	KDL::JntArray q_curr(7); 
	KDL::JntArray qdot_curr(7);
	KDL::JntArray q_pos_command(7); // qdot for invariant orientation 
	KDL::JntArray q_pos(7);
	KDL::JntArray qf(7); 
	 
	q0(0) = 1; q0(1) = 0; q0(2) = 0; q0(3) = 0; q0(4) = 0; q0(5) = 0; q0(6) = 0;
	qf(0) = data_q(0,10); qf(1) = data_q(1,10); qf(2) = data_q(2,10); qf(3) = data_q(3,10); qf(4) = data_q(4,10); qf(5) = data_q(5,10); qf(6) = data_q(6,10);
	double duration_init = 10;

	// Wait for the initial update. Makse sure the it is initilized properly
	while (!init || !init_time_bool) {ros::spinOnce();}
	
	// To make sure current position it recived
	if (init && init_time_bool) {
		q0.data = q_position.data;
		vel_prof->solve(q0, qf, duration_init);
		ROS_INFO_STREAM("Initial position recived and path is planned...");
	}


	// Set the data publishing rate
	double hz = 800.0;
	ros::Rate rate(hz);
	double curr_time = 0.0;
	iiwa_msgs::JointPosition command_position;
	command_position.position.quantity.resize(7);


	std_msgs::Bool command_active;
	command_active.data = false;
	// ---------------------------------------------------------------------------------
	std::vector<ecl::CubicSpline> splines;
	splines.resize(7);

	std::vector<alglib::spline1dinterpolant> splines_p1;
	splines_p1.resize(7);
	// ---------------------------------------------------------------------------------

	int sample_points = 10;
	int n_linspace = 36;

	// create an array of evaluating points. These are points at which splines should be evaluated.
	int* eval_array = spline_traj::linspace(10, n_data_q, n_linspace);

	double start_time = 0.0; // to keep track of the time for the spline
	double move_time = 1.5;
	double start_contact = 0.0;
	int track_pos = 0;
	int i = 0, f = 0;
	Eigen::MatrixXf temp(7,2);

	robot_def::robot kuka; // define the robot class
	Eigen::VectorXd qdot_O(7);
	double q[7]; // joint values
	int term_FM = 0;
	int F_contact = 0;
	double dt = 1 / hz;
	double vel_O = 0.005;
	double f_thresh = -2;

	// change the direction of movement.
	bool dir_1 = true; bool dir_2 = false; bool wait = true; double wait_start_time = 0.0; int dir_iter = 1; 

	while(ros::ok()) {

		/* Increment the time */
		curr_time = static_cast<double>(curr_kuka_time.data.sec) + static_cast<double>(curr_kuka_time.data.nsec)*static_cast<double>(pow(10, -9)) - init_time;
		// ROS_INFO_STREAM(curr_time);

		// brings to the initial pose of the robot
		if (curr_time < duration_init) {
			vel_prof->get_desired_joint_pos(q_curr, curr_time);
			memcpy(q_pos.data.data(), q_curr.data.data(), 7*sizeof(double));
			memcpy(command_position.position.quantity.data(), q_pos.data.data(), 7*sizeof(double));
			command_active.data = true;
			start_time = curr_time;
			wait_start_time = curr_time;

		} else {

			if ((track_pos == 0) || (track_pos == n_linspace-1) || (curr_time >= start_time + move_time)) {
				// ---------------------------------------BEGIN Force modulation sub routine ------------------------------------------------- //
				term_FM = 0; // Set the flag to not true
				F_contact = 0;
				f_data_prev = f_data; //

				while(wait==false && term_FM == 0) {
					memcpy(q, q_position_state_command.data.data(), 7*sizeof(double));
					term_FM = constant_orientation(kuka, q, qdot_O, vel_O*((f_data(2)-f_data_prev(2))-(f_thresh)));

					memcpy(q_pos_command.data.data(), command_position.position.quantity.data(), 7*sizeof(double));

					q_pos.data = q_pos_command.data + qdot_O * dt; 

					memcpy(command_position.position.quantity.data(), q_pos.data.data(), 7*sizeof(double));

					// ROS_INFO_STREAM("\nForce_Z: ");
					// ROS_INFO_STREAM(f_data);



					if (F_contact!=1 && ((curr_time > start_time + 14) || ((f_data(2)-f_data_prev(2)) < f_thresh) || (f_data.norm() > 10))) {
						F_contact = 1;
						start_contact = curr_time;
					}

					if (F_contact == 1 && (curr_time > start_contact+1)) {
						if (track_pos!= 0 && track_pos!=n_linspace-1) {
							term_FM = 1;	
						} else if(curr_time < start_contact+8) {
							// memcpy(q, q_position_state_command.data.data(), 7*sizeof(double));
							term_FM = constant_orientation(kuka, q, qdot_O, -2*vel_O);
							memcpy(q_pos_command.data.data(), command_position.position.quantity.data(), 7*sizeof(double));
							ROS_INFO_STREAM("switch...");
							q_pos.data = q_pos_command.data + qdot_O * 0.0033; 
							memcpy(command_position.position.quantity.data(), q_pos.data.data(), 7*sizeof(double));

							
						// } else if(curr_time < start_contact+14) { 
							// memcpy(q_pos_command.data.data(), command_position.position.quantity.data(), 7*sizeof(double));
						} else {
							term_FM = 1;
						}
						
					}

					command_position.header.stamp = ros::Time::now();
					command_active.data = true;
					kuka_motion_ready.publish(command_active);
					pub_position.publish(command_position);

					netft_.wrench.force.x = f_data(0)-f_data_prev(0); 
					netft_.wrench.force.y = f_data(1)-f_data_prev(1); 
					netft_.wrench.force.z = f_data(2)-f_data_prev(2); 
					netft_.header.stamp = ros::Time::now();
					netft_pub.publish(netft_);

					curr_time = static_cast<double>(curr_kuka_time.data.sec) + static_cast<double>(curr_kuka_time.data.nsec)*static_cast<double>(pow(10, -9)) - init_time;
					rate.sleep();
					ros::spinOnce();

				}
				// --------------------------------------- END Force modulation sub routine ------------------------------------------------- //
				
				// if dir_1 is true, increment track_pos 
				// if dir_2 is true, decrement track_pos
				if (wait==false) {
					if (dir_1) {
						f = eval_array[track_pos+1];
						track_pos++;
						
					} else if (dir_2) {
						f = eval_array[track_pos-1];
						track_pos--;
					}


					q0.data = q_position_state_command.data;
					qf(0) = data_q(0,f); qf(1) = data_q(1,f); qf(2) = data_q(2,f); qf(3) = data_q(3,f); 
					qf(4) = data_q(4,f); qf(5) = data_q(5,f); qf(6) = data_q(6,f);

					vel_prof->solve(q0, qf, move_time);

					// spline_traj::spline_interp(splines, temp, temp.cols(), 0, move_time);
					// spline_traj::spline_piecewise_interp(splines_p1, temp, 0, move_time);

					// if direction_1 is done, make it false and true direction_2
					if (track_pos == n_linspace-1) {
						switch_iter.point.x = dir_iter;
						switch_iter.header.stamp = ros::Time::now();

						dir_1 = false; dir_2 = true;
						wait = true; wait_start_time = curr_time;

						pub_switch_dir.publish(switch_iter);
						dir_iter++;
					} else if(track_pos == 0) {
						switch_iter.point.x = dir_iter;
						switch_iter.header.stamp = ros::Time::now();

						dir_1 = true; dir_2 = false;	
						wait = true; wait_start_time = curr_time;

						pub_switch_dir.publish(switch_iter);
						dir_iter++;		
					}
				}
				
				start_time = curr_time;
			}


			if ((wait==false) && (curr_time > start_time) && (curr_time < start_time + move_time)) {	
				// Use splines to interpolate data
				for (int i = 0;i < 7;i++) {
					double temp = curr_time-start_time;
					vel_prof->get_desired_joint_pos(q_curr, temp);
					memcpy(q_pos.data.data(), q_curr.data.data(), 7*sizeof(double));
				}
			} else if((wait==true) && (curr_time > wait_start_time + 0.5)) {
				wait = false;
				ROS_INFO_STREAM("wait set to false...");
			} else {
				ROS_INFO_STREAM("waiting 1 sec...");
			}


			memcpy(command_position.position.quantity.data(), q_pos.data.data(), 7*sizeof(double));
			command_active.data = true;
		}



		//Publish the generated torque and the time stamp 
		command_position.header.stamp = ros::Time::now();

		netft_.wrench.force.x = f_data(0)-f_data_prev(0); 
		netft_.wrench.force.y = f_data(1)-f_data_prev(1); 
		netft_.wrench.force.z = f_data(2)-f_data_prev(2); 
		netft_.header.stamp = ros::Time::now();
		netft_pub.publish(netft_);

		kuka_motion_ready.publish(command_active);
		pub_position.publish(command_position);

		rate.sleep();
		ros::spinOnce();

	}

	return 0;

	}





